"""
Host-side float32 port of grblHAL planner profile math + st_prep_buffer() segment
generation with ENABLE_JERK_ACCELERATION, used to reproduce the terminal crawl bug.

Units follow grblHAL internals: mm, mm/min, mm/min^2, mm/min^3, min.
Single-axis (Z) moves only; G64 geometry is not involved.

Usage: python jerk_sim.py [--fixed]
  --fixed  simulate the patched stepper.c behaviour
"""
import sys
import numpy as np

F = np.float32
FIXED = '--fixed' in sys.argv
RECALC = '--recalc' in sys.argv
TAIL = 5.0

RAMP_ACCEL, RAMP_CRUISE, RAMP_DECEL, RAMP_DECEL_OVR = 0, 1, 2, 3
RNAME = {0: 'ACC', 1: 'CRU', 2: 'DEC', 3: 'DOV'}


class Block:
    def __init__(s, mm, feed, acc_s2, jerk_s3, steps_mm, jerk=True):
        s.millimeters = F(mm)
        s.programmed_rate = F(feed)
        s.step_event_count = int(round(mm * steps_mm))
        s.max_acceleration = F(acc_s2 * 3600.0)
        s.jerk = F(jerk_s3 * 216000.0)
        s.cond_jerk = jerk
        if jerk:
            # planner.c effective acceleration
            tma = s.max_acceleration / s.jerk
            saj = F(0.5) * s.jerk * tma * tma
            if F(0.5) * s.programmed_rate > saj:
                s.acceleration = s.programmed_rate / (F(2.0) * (tma + (F(0.5) * s.programmed_rate - saj) / s.max_acceleration))
            else:
                s.acceleration = s.programmed_rate / (F(2.0) * np.sqrt(s.programmed_rate / s.jerk))
        else:
            s.acceleration = s.max_acceleration
        s.entry_speed_sqr = F(0)
        s.max_entry_speed_sqr = F(0)


def plan(blocks, junction_sqr):
    """Optimal entry speeds for a fully known buffer, exit of last block = 0."""
    n = len(blocks)
    for i, b in enumerate(blocks):
        prev_nom = blocks[i - 1].programmed_rate if i else F(0)
        b.max_entry_speed_sqr = min(min(prev_nom, b.programmed_rate) ** 2, junction_sqr[i]) if i else F(0)
    nxt = F(0)
    for b in reversed(blocks):
        b.entry_speed_sqr = min(b.max_entry_speed_sqr, nxt + F(2) * b.acceleration * b.millimeters)
        nxt = b.entry_speed_sqr
    for i in range(n - 1):
        c, d = blocks[i], blocks[i + 1]
        if c.entry_speed_sqr < d.entry_speed_sqr:
            e = c.entry_speed_sqr + F(2) * c.acceleration * c.millimeters
            if e < d.entry_speed_sqr:
                d.entry_speed_sqr = e


class Prep:
    pass


def compute_profile(p, b, exit_speed_sqr):
    p.mm_complete = F(0)
    inv_2_accel = F(0.5) / b.acceleration
    p.ramp_type = RAMP_ACCEL
    p.accelerate_until = b.millimeters
    p.exit_speed = np.sqrt(exit_speed_sqr)
    nominal = b.programmed_rate * F(p.feed_ovr)
    nsq = nominal * nominal
    inter = F(0.5) * (b.millimeters + inv_2_accel * (b.entry_speed_sqr - exit_speed_sqr))
    if b.entry_speed_sqr > nsq:
        p.accelerate_until = b.millimeters - inv_2_accel * (b.entry_speed_sqr - nsq)
        if p.accelerate_until <= F(0):
            p.ramp_type = RAMP_DECEL
            p.exit_speed = np.sqrt(max(F(0), b.entry_speed_sqr - F(2) * b.acceleration * b.millimeters))
        else:
            p.decelerate_after = inv_2_accel * (nsq - exit_speed_sqr)
            p.maximum_speed = nominal
            p.ramp_type = RAMP_DECEL_OVR
    elif inter > F(0):
        if inter < b.millimeters:
            p.decelerate_after = inv_2_accel * (nsq - exit_speed_sqr)
            if p.decelerate_after < inter:
                p.maximum_speed = nominal
                if b.entry_speed_sqr == nsq:
                    p.ramp_type = RAMP_CRUISE
                else:
                    p.accelerate_until -= inv_2_accel * (nsq - b.entry_speed_sqr)
            else:
                p.accelerate_until = p.decelerate_after = inter
                p.maximum_speed = np.sqrt(F(2) * b.acceleration * inter + exit_speed_sqr)
        else:
            p.ramp_type = RAMP_DECEL
    else:
        p.accelerate_until = F(0)
        p.maximum_speed = p.exit_speed


def prep_segment(p, b, DT):
    """One pass of the st_prep_buffer() do-while. Returns dict of segment info."""
    dt_max = F(DT)
    dt = F(0)
    time_var = dt_max
    mm_remaining = b.millimeters
    minimum_mm = max(F(0), mm_remaining - p.req_mm_increment)
    J = b.jerk
    term = False
    acc_end = False
    while True:
        rt = p.ramp_type
        if rt == RAMP_DECEL_OVR:
            speed_var = b.acceleration * time_var
            if p.current_speed - p.maximum_speed <= speed_var:
                mm_remaining = p.accelerate_until
                time_var = F(2) * (b.millimeters - mm_remaining) / (p.current_speed + p.maximum_speed)
                p.ramp_type = RAMP_CRUISE
                p.current_speed = p.maximum_speed
            else:
                mm_remaining -= time_var * (p.current_speed - F(0.5) * speed_var)
                p.current_speed -= speed_var
            p.last_accel = F(0)
        elif rt == RAMP_ACCEL:
            if p.jerk:
                accel_var = J * time_var
                ttj = p.last_accel / J
                jr = ttj * (p.current_speed + F(0.5) * p.last_accel * ttj + J * ttj * ttj * F(1.0 / 6.0))
                if (mm_remaining - p.accelerate_until) > jr:
                    p.last_accel = min(p.last_accel + accel_var, b.max_acceleration)
                else:
                    p.last_accel = max(p.last_accel - accel_var, accel_var)
                speed_var = p.last_accel * time_var
            else:
                speed_var = b.acceleration * time_var
            mm_remaining -= time_var * (p.current_speed + F(0.5) * speed_var)
            if mm_remaining < p.accelerate_until:
                acc_end = True
                mm_remaining = p.accelerate_until
                time_var = F(2) * (b.millimeters - mm_remaining) / (p.current_speed + p.maximum_speed)
                p.ramp_type = RAMP_DECEL if mm_remaining == p.decelerate_after else RAMP_CRUISE
                p.current_speed = p.maximum_speed
                if p.jerk:
                    p.last_accel = F(0)
            else:
                p.current_speed += speed_var
        elif rt == RAMP_CRUISE:
            mm_var = mm_remaining - p.maximum_speed * time_var
            if mm_var < p.decelerate_after:
                time_var = (mm_remaining - p.decelerate_after) / p.maximum_speed
                mm_remaining = p.decelerate_after
                p.ramp_type = RAMP_DECEL
            else:
                mm_remaining = mm_var
        else:  # RAMP_DECEL
            if p.jerk:
                accel_var = J * time_var
                if FIXED:
                    a0 = p.last_accel
                    p.last_accel = decel_target(p, J, b.max_acceleration, mm_remaining - p.mm_complete, time_var)
                    speed_var = F(0.5) * (a0 + p.last_accel) * time_var
                else:
                    ttj = accel_var if p.last_accel == F(0) else p.last_accel / J
                    jr = p.exit_speed + ttj * (p.last_accel - F(0.5) * J * ttj)
                    if p.current_speed > jr:
                        p.last_accel = min(p.last_accel + accel_var, b.max_acceleration)
                    else:
                        p.last_accel = max(p.last_accel - accel_var, accel_var)
                    speed_var = p.last_accel * time_var
            else:
                speed_var = b.acceleration * time_var
            done = False
            if p.current_speed > speed_var:
                mm_var = mm_remaining - time_var * (p.current_speed - F(0.5) * speed_var)
                if mm_var > p.mm_complete:
                    mm_remaining = mm_var
                    p.current_speed -= speed_var
                    done = True
            if not done:
                term = True
                p.term_v = p.current_speed
                p.term_mm = mm_remaining
                time_var = F(2) * (mm_remaining - p.mm_complete) / (p.current_speed + p.exit_speed)
                mm_remaining = p.mm_complete
                p.current_speed = p.exit_speed
        dt += time_var
        if dt < dt_max:
            time_var = dt_max - dt
        else:
            if mm_remaining > minimum_mm:
                dt_max += F(DT)
                time_var = dt_max - dt
            else:
                break
        if not (mm_remaining > p.mm_complete):
            break
    sdr = p.steps_per_mm * mm_remaining
    nrem = int(np.ceil(sdr))
    n_step = p.steps_remaining - nrem
    dt_tot = dt + p.dt_remainder
    inv_rate = dt_tot / (F(p.steps_remaining) - sdr) if p.steps_remaining - sdr > 0 else F(0)
    seg = dict(mm=float(b.millimeters), mm_after=float(mm_remaining), v=float(p.current_speed),
               a=float(p.last_accel), ramp=RNAME[p.ramp_type], dt=float(dt), n=n_step,
               inv_rate=float(inv_rate), term=term, acc_end=acc_end)
    b.millimeters = mm_remaining
    p.steps_remaining = nrem
    p.dt_remainder = (F(nrem) - sdr) * inv_rate
    return seg


def decel_profile_distance(v, a, ve, ap, J):
    """Distance of jerk-limited decel from (v, a) to (ve, 0) via plateau deceleration ap."""
    t1 = abs(ap - a) / J
    s = F(1) if ap > a else F(-1)
    d1 = t1 * (v - t1 * (F(0.5) * a + s * J * t1 * F(1.0 / 6.0)))
    v1 = v - abs(ap * ap - a * a) * F(0.5) / J
    t3 = ap / J
    v2 = ve + F(0.5) * ap * t3
    d2 = (v1 - v2) / ap * F(0.5) * (v1 + v2)
    d3 = t3 * (ve + ap * t3 * F(1.0 / 6.0))
    return d1 + d2 + d3


JERK_PLAN = F(next((a.split('=')[1] for a in sys.argv if a.startswith('--jp=')), 0.9))
BEHIND_FULLJ = '--behind-fullj' in sys.argv


def decel_target(p, J, Amax, d, dt):
    """End-of-segment deceleration such that the jerk-limited profile ends at exit_speed exactly at d."""
    v, a, ve = p.current_speed, p.last_accel, p.exit_speed
    Jp = J * JERK_PLAN
    dv = v - ve
    if BEHIND_FULLJ and dv > F(0) and d > F(0) and dv - F(0.5) * a * a / J > F(0):
        # behind schedule: margin would only make it later, plan at full jerk
        hiJ = min(Amax, np.sqrt(J * dv + F(0.5) * a * a))
        if decel_profile_distance(v, a, ve, hiJ, J) >= d:
            Jp = J
    if dv <= F(0) or d <= F(0):
        target = F(0)
    elif dv - F(0.5) * a * a / Jp <= a * dt:
        # final jerk-out: linear ramp of a to 0 that lands exactly on (ve, d)
        a_now = F(2) * dv * (ve + dv * F(1.0 / 3.0)) / d
        target = min(a_now - F(0.5) * a_now * a_now / dv * dt, a - F(0.5) * a * a / dv * dt)
    else:
        hi = min(Amax, np.sqrt(Jp * dv + F(0.5) * a * a))
        if decel_profile_distance(v, a, ve, hi, Jp) >= d:
            target = hi
        else:
            lo = F(0)
            for _ in range(20):
                mid = F(0.5) * (lo + hi)
                if mid <= F(0) or decel_profile_distance(v, a, ve, mid, Jp) > d:
                    lo = mid
                else:
                    hi = mid
            target = hi
    return max(min(target, a + J * dt, Amax), a - J * dt, F(0))


def stop_dist(p, J, Amax):
    """Distance to reach exit speed from (current_speed, last_accel) following an S-curve
    with jerk J and acceleration limit Amax (mirrors stepper.c fix)."""
    v, a, ve = p.current_speed, p.last_accel, p.exit_speed
    dv = v - ve
    ap = np.sqrt(J * dv + F(0.5) * a * a)  # peak decel, caller guarantees dv > a^2/2J
    if ap > Amax:
        ap = Amax
    t1 = (ap - a) / J
    t3 = ap / J
    v1 = v - F(0.5) * (a + ap) * t1
    t2 = (v1 - ve - F(0.5) * ap * t3) / ap
    d1 = t1 * (v - t1 * (F(0.5) * a + J * t1 * F(1.0 / 6.0)))
    d2 = t2 * (v1 - F(0.5) * ap * t2)
    d3 = t3 * (ve + ap * t3 * F(1.0 / 6.0))
    return d1 + d2 + d3


def run(blocks, steps_mm, ticks, jerk=True, recalc_at=None, feed_ovr=1.0, log=False):
    """Execute the block list. recalc_at = (block_idx, seg_idx) to simulate
    st_update_plan_block_parameters() when the next line is streamed in."""
    DT = 1.0 / (ticks * 60.0)
    p = Prep()
    p.jerk = jerk
    p.last_accel = F(0)
    p.current_speed = F(0)
    p.feed_ovr = feed_ovr
    p.decelerate_after = F(0)
    p.maximum_speed = F(0)
    worst = dict(max_dt=0.0, crawl_mm=0.0, max_vjump=0.0, steps_err=0)
    total_t = 0.0
    executed = []  # (rate mm/s, duration s, n_step, tags) of each executed segment
    for bi, b in enumerate(blocks):
        p.steps_per_mm = F(b.step_event_count) / b.millimeters
        p.steps_remaining = b.step_event_count
        p.req_mm_increment = F(1.25) / p.steps_per_mm
        p.dt_remainder = F(0)
        p.current_speed = np.sqrt(b.entry_speed_sqr)
        exit_sqr = blocks[bi + 1].entry_speed_sqr if bi + 1 < len(blocks) else F(0)
        compute_profile(p, b, exit_sqr)
        si = 0
        steps_done = 0
        prev_v = float(p.current_speed)
        while b.millimeters > p.mm_complete:
            if recalc_at == (bi, si):
                b.entry_speed_sqr = p.current_speed * p.current_speed
                compute_profile(p, b, exit_sqr)
            seg = prep_segment(p, b, DT)
            if seg['n'] > 0:
                tags = set()
                if si == 0 and bi > 0:
                    tags.add('junction')
                if seg['acc_end']:
                    tags.add('acc_end')
                if seg['term']:
                    tags.add('blk_end')
                executed.append((1.0 / (seg['inv_rate'] * float(p.steps_per_mm)) / 60.0,
                                 seg['n'] * seg['inv_rate'] * 60.0, seg['n'], tags, float(b.max_acceleration) / 3600.0))
            steps_done += seg['n']
            total_t += seg['dt']
            worst['max_dt'] = max(worst['max_dt'], seg['dt'] * 60.0)
            if seg['term']:
                worst['term_dt'] = max(worst.get('term_dt', 0.0), seg['dt'] * 60.0)
            if seg['term'] and seg['dt'] * 60.0 > 0.1:
                worst['crawl_mm'] = max(worst['crawl_mm'], float(p.term_mm))
            worst['max_vjump'] = max(worst['max_vjump'], abs(seg['v'] - prev_v) / 60.0)
            if seg['ramp'] == 'DEC' and si and not seg['term']:
                if seg['v'] > prev_v + 1e-3:
                    worst['nonmono'] = worst.get('nonmono', 0) + 1
                if abs(seg['a'] - prev_a) > float(b.jerk) * seg['dt'] * 1.001:
                    worst['jerk_viol'] = worst.get('jerk_viol', 0) + 1
            prev_a = seg['a']
            prev_v = seg['v']
            if log and seg['mm'] < TAIL:
                print(f"  blk{bi} seg{si:4d} {seg['ramp']} mm={seg['mm']:8.4f} v={seg['v']/60:8.4f}mm/s "
                      f"a={seg['a']/3600:8.3f}mm/s2 dt={seg['dt']*60e3:9.3f}ms n={seg['n']:5d} "
                      f"t/step={seg['inv_rate']*60e6:10.1f}us{' TERM' if seg['term'] else ''}")
            si += 1
            if si > 200000:
                break
        if steps_done != b.step_event_count:
            worst['steps_err'] += 1
    worst['time'] = total_t * 60.0
    worst['snaps'] = snap_analysis(executed)
    return worst


def snap_analysis(executed):
    """Step-rate discontinuities between consecutive executed segments, beyond what max
    acceleration over the segment spacing plus one step of quantization can explain."""
    seq = [(0.0, 0.0, 1, {'start'}, executed[0][4])] + executed + [(0.0, 0.0, 1, {'stop'}, executed[-1][4])]
    snaps = []
    for (r0, d0, n0, t0, A0), (r1, d1, n1, t1, A1) in zip(seq, seq[1:]):
        span = 0.5 * (d0 + d1)
        dr = abs(r1 - r0)
        q = max(r0 / n0, r1 / n1)  # one step per segment of rate resolution
        excess = dr - max(A0, A1) * span - q
        if excess > 0.0:
            where = (t0 | t1) - {'start', 'stop'} if (t0 | t1) - {'start', 'stop'} else (t0 | t1)
            snaps.append(dict(dv=dr, sdv=r1 - r0, r0=r0, excess=excess, q=q, ratio=dr / span / max(A0, A1) if span > 0 else float('inf'),
                              where='+'.join(sorted(where)) or 'mid'))
    return snaps


def z_moves(dists, feed, acc, jerk_v, steps_mm, jerk=True, junction=None):
    bl = [Block(abs(d), feed, acc, jerk_v, steps_mm, jerk) for d in dists]
    js = [F(0)] + [F(1e38) if (dists[i] > 0) == (dists[i - 1] > 0) else F(0) for i in range(1, len(dists))]
    plan(bl, js)
    return bl


CASES = {'A': lambda d: [-d], 'B': lambda d: [-d, -min(5.0, d)], 'B2': lambda d: [-d, -0.5 * d],
         'B3': lambda d: [-d, -d, -0.3 * d], 'REV': lambda d: [-d, d, -d]}


def run_cfg(cfg):
    steps_mm, ticks, acc, jk, feed, dist, case, jerk_on, rc = cfg
    r = run(z_moves(CASES[case](dist), feed, acc, jk, steps_mm, jerk_on), steps_mm, ticks, jerk=jerk_on, recalc_at=rc)
    return dict(r, cfg=cfg)


CASES = {'A': lambda d: [-d], 'B': lambda d: [-d, -min(5.0, d)], 'B2': lambda d: [-d, -0.5 * d],
         'B3': lambda d: [-d, -d, -0.3 * d], 'REV': lambda d: [-d, d, -d]}


def run_cfg(cfg):
    steps_mm, ticks, acc, jk, feed, dist, case, jerk_on, rc = cfg
    r = run(z_moves(CASES[case](dist), feed, acc, jk, steps_mm, jerk_on), steps_mm, ticks, jerk=jerk_on, recalc_at=rc)
    return dict(r, cfg=cfg)


def two_short(d1, d2, feed, acc, jk, steps_mm, ticks, log=False, jerk=True):
    bl = z_moves([-d1, -d2], feed, acc, jk, steps_mm, jerk)
    if log:
        for i, b in enumerate(bl):
            print(f' blk{i} mm={b.millimeters} entry={np.sqrt(b.entry_speed_sqr)/60:.3f}mm/s a_eff={b.acceleration/3600:.3f}mm/s2')
    return run(bl, steps_mm, ticks, jerk=jerk, log=log)


if __name__ == '__main__' and '--two' in sys.argv:
    print('FIXED' if FIXED else 'ORIGINAL')
    a = [float(x) for x in sys.argv[sys.argv.index('--two') + 1:][:8]]
    # args: d1 d2 feed acc jerk steps_mm ticks [jerk_on]
    r = two_short(*a[:6], int(a[6]), log=True, jerk=a[7] != 0 if len(a) > 7 else True)
    print(r)
    sys.exit()

if __name__ == '__main__' and '--moves' in sys.argv:
    # args: d1,d2,... feed acc jerk steps_mm ticks [tail_mm]
    i = sys.argv.index('--moves')
    d = [float(x) for x in sys.argv[i + 1].split(',')]
    feed, acc, jk, smm, ticks = (float(x) for x in sys.argv[i + 2:i + 7])
    TAIL = float(sys.argv[i + 7]) if len(sys.argv) > i + 7 else 5.0
    jon = '--nojerk' not in sys.argv
    print(run(z_moves(d, feed, acc, jk, smm, jon), smm, int(ticks), jerk=jon, log=True))
    sys.exit()

if __name__ == '__main__':
    import itertools
    from multiprocessing import Pool
    print('FIXED' if FIXED else 'ORIGINAL')
    STEPS = [float(x) for x in next((a.split('=')[1] for a in sys.argv if a.startswith('--steps=')), '40,80,400,800').split(',')]
    jobs = []
    for steps_mm, ticks, acc, jk, feed, dist, case in itertools.product(STEPS, (100, 250, 1000), (10.0, 100.0, 500.0),
                                                                        (50.0, 1000.0, 10000.0), (300.0, 1200.0, 3000.0),
                                                                        (0.5, 2.0, 20.0), CASES):
        for jerk_on in ((True,) if '--jerk-only' in sys.argv else (True, False)):
            recs = [None] + ([(len(CASES[case](dist)) - 1, s) for s in range(0, 120, 9)] if RECALC and jerk_on else [])
            jobs += [(steps_mm, ticks, acc, jk, feed, dist, case, jerk_on, rc) for rc in recs]
    with Pool() as pool:
        rows = pool.map(run_cfg, jobs, chunksize=16)
    cases = CASES
    def summ(sel, label):
        if not sel:
            return
        print(f'{label}: runs={len(sel)} crawl(term seg>100ms)={sum(1 for r in sel if r["crawl_mm"] > 0)} '
              f'max_term_ms={max(r.get("term_dt", 0) for r in sel)*1e3:.1f} '
              f'nonmono={sum(r.get("nonmono", 0) for r in sel)} jerk_viol={sum(r.get("jerk_viol", 0) for r in sel)} '
              f'step_err={sum(r["steps_err"] for r in sel)}')
    summ([r for r in rows if r['cfg'][7]], 'jerk ON ')
    summ([r for r in rows if not r['cfg'][7]], 'jerk OFF')
    for c in cases:
        summ([r for r in rows if r['cfg'][7] and r['cfg'][6] == c], f'  case {c:3s}')
    worst = sorted([r for r in rows if r['cfg'][7]], key=lambda r: -r.get('term_dt', 0))[:8]
    for r in worst:
        print(f"  term={r.get('term_dt',0)*1e3:10.1f}ms crawl_mm={r['crawl_mm']:.3f} cfg={r['cfg']}")

    print('\nSPEED SNAPS (executed step-rate jump beyond Amax*spacing + 1 step quantization)')
    for jerk_on in (True, False):
        sel = [r for r in rows if r['cfg'][7] == jerk_on]
        if not sel:
            continue
        print(f"jerk {'ON ' if jerk_on else 'OFF'}: runs with snaps {sum(1 for r in sel if r['snaps'])}/{len(sel)}")
        by = {}
        for r in sel:
            for s in r['snaps']:
                by.setdefault(s['where'], []).append((s, r['cfg']))
        for w, lst in sorted(by.items(), key=lambda kv: -len(kv[1])):
            dv = np.array([s['dv'] for s, _ in lst])
            rel = np.array([s['dv'] / (c[4] / 60.0) for s, c in lst])
            ratio = np.array([s['ratio'] for s, _ in lst])
            print(f'  {w:22s} n={len(lst):6d} dv mm/s med={np.median(dv):7.2f} p95={np.percentile(dv, 95):7.2f} max={dv.max():7.2f}'
                  f' | %feed p95={100*np.percentile(rel, 95):5.1f} max={100*rel.max():5.1f} | x Amax p95={np.percentile(ratio, 95):7.1f}')
        if '--snapcfg' in sys.argv:
            allsn = sorted(((s, c) for lst in by.values() for s, c in lst), key=lambda sc: -sc[0]['dv'])[:12]
            for s, c in allsn:
                print(f"    dv={s['dv']:7.2f}mm/s x{s['ratio']:6.1f}Amax q={s['q']:6.2f} {s['where']:16s} cfg={c}")
        for dim, idx in (('steps/mm', 0), ('ticks', 1), ('jerk', 3), ('case', 6)):
            keys = sorted({r['cfg'][idx] for r in sel}, key=str)
            print(f"   by {dim:8s}: " + '  '.join(f"{k}:{sum(1 for r in sel if r['cfg'][idx] == k and r['snaps'])}/{sum(1 for r in sel if r['cfg'][idx] == k)}" for k in keys))

    print('\nSPEED SNAPS (executed step-rate jump beyond Amax*spacing + 1 step quantization)')
    for jerk_on in (True, False):
        sel = [r for r in rows if r['cfg'][7] == jerk_on]
        if not sel:
            continue
        print(f"jerk {'ON ' if jerk_on else 'OFF'}: runs with snaps {sum(1 for r in sel if r['snaps'])}/{len(sel)}")
        by = {}
        for r in sel:
            for s in r['snaps']:
                by.setdefault(s['where'], []).append((s, r['cfg']))
        for w, lst in sorted(by.items(), key=lambda kv: -len(kv[1])):
            dv = np.array([s['dv'] for s, _ in lst])
            rel = np.array([s['dv'] / (c[4] / 60.0) for s, c in lst])
            ratio = np.array([s['ratio'] for s, _ in lst])
            print(f'  {w:22s} n={len(lst):6d} dv mm/s med={np.median(dv):7.2f} p95={np.percentile(dv, 95):7.2f} max={dv.max():7.2f}'
                  f' | %feed p95={100*np.percentile(rel, 95):5.1f} max={100*rel.max():5.1f} | x Amax p95={np.percentile(ratio, 95):7.1f}')
        if '--snapcfg' in sys.argv:
            allsn = sorted(((s, c) for lst in by.values() for s, c in lst), key=lambda sc: -sc[0]['dv'])[:12]
            for s, c in allsn:
                print(f"    dv={s['dv']:7.2f}mm/s x{s['ratio']:6.1f}Amax q={s['q']:6.2f} {s['where']:16s} cfg={c}")
        for dim, idx in (('steps/mm', 0), ('ticks', 1), ('jerk', 3), ('case', 6)):
            keys = sorted({r['cfg'][idx] for r in sel}, key=str)
            print(f"   by {dim:8s}: " + '  '.join(f"{k}:{sum(1 for r in sel if r['cfg'][idx] == k and r['snaps'])}/{sum(1 for r in sel if r['cfg'][idx] == k)}" for k in keys))
    import json
    json.dump([dict(cfg=r['cfg'], term=r.get('term_dt', 0), time=r['time'], snaps=r['snaps']) for r in rows],
              open('result_' + ('fixed' if FIXED else 'orig') + ('_rc' if RECALC else '') + '.json', 'w'))
