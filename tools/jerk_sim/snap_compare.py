"""Compare speed snaps between result_orig.json and result_fixed.json (jerk-on sweeps)."""
import json
import collections
import numpy as np

orig = {tuple(r['cfg']): r for r in json.load(open('result_orig.json')) if r['cfg'][7]}
fixed = {tuple(r['cfg']): r for r in json.load(open('result_fixed.json')) if r['cfg'][7]}
keys = sorted(set(orig) & set(fixed), key=str)


def worst(r, where=None):
    s = [x for x in r['snaps'] if where is None or where in x['where']]
    return max(s, key=lambda x: x['dv']) if s else None


trans = collections.Counter()
for k in keys:
    o, f = orig[k], fixed[k]
    oc, fc = o['term'] > 0.1, f['term'] > 0.1
    os_, fs = bool(o['snaps']), bool(f['snaps'])
    trans[('crawl' if oc else 'snap' if os_ else 'clean') + ' -> ' + ('crawl' if fc else 'snap' if fs else 'clean')] += 1
print('per-run outcome original -> fixed:')
for t, n in sorted(trans.items(), key=lambda kv: -kv[1]):
    print(f'  {t:16s} {n}')

print('\nsnap direction at block end (fixed): drop = arrived too fast (late), rise = arrived too slow (early)')
for name, data in (('orig', orig), ('fixed', fixed)):
    c = collections.Counter()
    for r in data.values():
        for s in r['snaps']:
            if 'blk_end' in s['where']:
                c[('drop' if s['sdv'] < 0 else 'rise') + (' junction' if 'junction' in s['where'] else ' stop')] += 1
    print(f'  {name:5s} {dict(c)}')

print('\nworst snap per run, fixed, grouped by Amax*dt (accel change one segment allows) vs dv:')
rows = []
for k in keys:
    w = worst(fixed[k])
    if w:
        steps, ticks, acc, jk, feed = k[:5]
        rows.append((w['dv'], w['dv'] / (acc / ticks), w['dv'] / (feed / 60.0), w['q'], w['where'], k))
dv = np.array([r[0] for r in rows])
seg = np.array([r[1] for r in rows])
print(f'  runs={len(rows)}  dv mm/s p50={np.median(dv):.2f} p95={np.percentile(dv,95):.2f} max={dv.max():.2f}')
print(f'  dv in segments-of-Amax  p50={np.median(seg):.1f} p95={np.percentile(seg,95):.1f} max={seg.max():.1f}')
for lim in (1, 2, 5, 10):
    print(f'  dv <= {lim:2d} x (Amax*dt): {np.mean(seg <= lim)*100:5.1f}%')

print('\nfixed-run snaps by planner regime (Vp = feed, Amax^2/J = speed after full jerk ramp):')
c = collections.Counter()
for dvv, s, rel, q, where, k in rows:
    steps, ticks, acc, jk, feed, dist, case = k[:7]
    regime = 'low' if feed / 60.0 / 2 <= acc * acc / jk / 2 else 'high'
    c[(where.split('+')[0], regime, 'J*dt>=A/2' if jk / ticks >= acc / 2 else 'J*dt<A/2')] += 1
for t, n in sorted(c.items(), key=lambda kv: -kv[1]):
    print(f'  {t}: {n}')
