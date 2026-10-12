"""L8 front end: feeds source moves through geometry (L3), projection (L4) and the runtime (L6).

Corner decision (DESIGN.md section 3C): G64 P permits deviation; it does not mandate a blend. A blend is
used only if the plan from the committed tail to the end of the known queue is faster with it than with an
exact stop, and only if the runtime accepts it. Otherwise the corner is an exact stop, which the runtime
always accepts (the current stop witness ends at the vertex).
"""

from __future__ import annotations

import math
from typing import Sequence

from .constraints import ConservativeProjection, blocks
from .contracts import Lookahead, PathGeometry, Revision
from .geometry import LinePiece
from .model import Limits, Move, State
from .runtime import InvariantError, StreamingRuntime


class MotionFrontEnd:
    """Reference implementation of contracts.FrontEnd."""

    def __init__(self, geometry: PathGeometry, runtime: StreamingRuntime, lookahead: Lookahead, axes: Sequence[Limits],
                 projection: ConservativeProjection | None = None) -> None:
        self.geometry, self.runtime, self.lookahead, self.axes = geometry, runtime, lookahead, list(axes)
        self.projection = projection or ConservativeProjection()
        self.declined = 0   # blends slower than an exact stop
        self.rejected = 0   # blends the runtime could not accept

    def feed(self, move: Move) -> Revision:
        tail = self.runtime.tail.s
        revision = self.geometry.propose(move, tail)
        if not revision.ok:
            return revision
        if revision.corner is not None and revision.corner.kind == "blend":
            stop = self.geometry.propose(move, tail, blend=False)
            if self.time_to_finish(stop) <= self.time_to_finish(revision):
                self.declined += 1
                revision = stop
            elif self._revise(revision):
                self.geometry.accept(revision)
                return revision
            else:
                self.rejected += 1
                revision = stop
        if not self._revise(revision):
            raise InvariantError("an exact-stop revision must always be accepted")
        self.geometry.accept(revision)
        return revision

    def time_to_finish(self, revision: Revision) -> float:
        """Planned time from the committed tail to rest at the end of the known queue with this revision;
        while holding (no plan from a moving tail) the rest-to-rest window time instead."""
        plan = self.runtime.preview(revision.from_s, self._blocks(revision), revision.stop_at_from)
        if plan is None:
            return self.window_time(revision)
        return plan.duration if plan.ok else math.inf

    def window_time(self, revision: Revision) -> float:
        """Rest-to-rest plan time from the incoming piece's start through the revision's pieces."""
        start_s, incoming, _ = self.geometry.pieces()[-1]
        pieces, stops = list(revision.pieces), list(revision.stops)
        if revision.from_s > start_s:
            kept = LinePiece(incoming.source_id, incoming.feed, incoming.start, incoming.direction, revision.from_s - start_s) \
                if isinstance(incoming, LinePiece) else incoming
            pieces, stops = [kept] + pieces, [revision.stop_at_from] + stops
        window = blocks(pieces, stops, self.axes, self.projection)
        plan = self.lookahead.plan(State(start_s, 0.0, 0.0), window, self.runtime.limits, self.runtime.override)
        return plan.duration if plan.ok else math.inf

    def _blocks(self, revision: Revision):
        return blocks(revision.pieces, revision.stops, self.axes, self.projection)

    def _revise(self, revision: Revision) -> bool:
        return self.runtime.revise(revision.from_s, self._blocks(revision), revision.stop_at_from)
