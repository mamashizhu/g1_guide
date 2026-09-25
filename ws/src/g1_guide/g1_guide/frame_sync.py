"""ROS-independent RGB-D frame pairing and freshness checks.

Message header stamps are used only to decide whether RGB and depth belong to
the same sensor update.  Receipt monotonic time is used for freshness so a
paused simulation cannot keep an old pair looking live forever.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Any, Optional


VALID_STATUSES = frozenset({"valid"})


@dataclass(frozen=True)
class FrameSample:
    """One received image and the two clocks associated with it."""

    stream: str
    stamp_sec: float
    received_monotonic: float
    payload: Any
    sequence: int


@dataclass(frozen=True)
class SynchronizedFrame:
    """An RGB/depth pair selected by header timestamp proximity."""

    rgb: FrameSample
    depth: FrameSample
    pair_id: int

    @property
    def stamp_delta_sec(self) -> float:
        return abs(self.rgb.stamp_sec - self.depth.stamp_sec)

    @property
    def received_monotonic(self) -> float:
        # Pair age is the age of its older half.  A newly received RGB frame
        # must not make an old depth frame look fresh.
        return min(self.rgb.received_monotonic, self.depth.received_monotonic)

    @property
    def header_stamp_sec(self) -> float:
        return max(self.rgb.stamp_sec, self.depth.stamp_sec)

    def age_sec(self, now_monotonic: float) -> float:
        return max(
            now_monotonic - self.rgb.received_monotonic,
            now_monotonic - self.depth.received_monotonic,
        )


@dataclass(frozen=True)
class FrameSnapshot:
    """The externally visible health state of the synchronizer."""

    status: str
    pair: Optional[SynchronizedFrame]
    message: str
    age_sec: Optional[float] = None

    @property
    def is_valid(self) -> bool:
        return self.status in VALID_STATUSES and self.pair is not None


class FrameSynchronizer:
    """Pair RGB/depth samples without depending on ROS.

    Samples are matched by the nearest header timestamp within
    ``tolerance_sec``.  A matched pair is consumed exactly once, and a newer
    pair replaces the current pair.  ``snapshot`` never reports a pair as
    valid after ``stale_after_sec`` of monotonic receipt time.
    """

    def __init__(
        self,
        tolerance_sec: float = 0.05,
        stale_after_sec: float = 0.5,
        clock_reset_threshold_sec: float = 1.0,
        pending_limit: int = 16,
    ) -> None:
        try:
            tolerance = float(tolerance_sec)
            stale_after = float(stale_after_sec)
            clock_reset_threshold = float(clock_reset_threshold_sec)
        except (TypeError, ValueError) as error:
            raise ValueError("timing thresholds must be finite numbers") from error
        if not math.isfinite(tolerance) or tolerance < 0.0:
            raise ValueError("tolerance_sec must be non-negative")
        if not math.isfinite(stale_after) or stale_after <= 0.0:
            raise ValueError("stale_after_sec must be positive")
        if not math.isfinite(clock_reset_threshold) or clock_reset_threshold <= 0.0:
            raise ValueError("clock_reset_threshold_sec must be positive")
        if pending_limit < 1:
            raise ValueError("pending_limit must be positive")

        self.tolerance_sec = tolerance
        self.stale_after_sec = stale_after
        self.clock_reset_threshold_sec = clock_reset_threshold
        self.pending_limit = int(pending_limit)

        self._rgb_pending: list[FrameSample] = []
        self._depth_pending: list[FrameSample] = []
        self._latest_pair: Optional[SynchronizedFrame] = None
        self._sequence = 0
        self._pair_sequence = 0
        self._latest_pair_stamp_sec: Optional[float] = None
        self._last_header_stamp: Optional[float] = None
        self._last_monotonic: Optional[float] = None
        self._invalid_reason: Optional[str] = None
        self._event_status: Optional[str] = None
        self._pending_expired = False
        self._reset_guard_anchor: Optional[float] = None
        self._reset_guard_driver: Optional[str] = None
        self._reset_guard_streams: set[str] = set()

    @staticmethod
    def _stamp_value(stamp_sec: Any) -> Optional[float]:
        try:
            value = float(stamp_sec)
        except (TypeError, ValueError):
            return None
        return value if math.isfinite(value) and value >= 0.0 else None

    def _clear_for_reset(self, reason: str) -> None:
        self._rgb_pending.clear()
        self._depth_pending.clear()
        self._latest_pair = None
        self._latest_pair_stamp_sec = None
        self._invalid_reason = None
        self._event_status = "clock_reset"
        self._pending_expired = False
        self._clock_reset_reason = reason

    def _observe_header_clock(self, stream: str, stamp_sec: float) -> bool:
        if self._reset_guard_anchor is not None:
            if (
                stream == self._reset_guard_driver
                and stamp_sec >= self._reset_guard_anchor
            ):
                # Let the triggering stream move the candidate reference.  Do
                # not reject a large jump here: a valid stream may have no
                # intermediate sample while the other stream is absent.
                self._reset_guard_anchor = stamp_sec
            # Header-only data cannot distinguish every delayed old-epoch
            # sample from a valid large forward jump.  Keep all candidates
            # isolated until two streams confirm one close pair instead of
            # permanently rejecting a possibly valid new epoch.
            return True

        previous = self._last_header_stamp
        if (
            previous is not None
            and previous - stamp_sec > self.clock_reset_threshold_sec
        ):
            self._clear_for_reset(
                f"header time moved backwards by {previous - stamp_sec:.3f} s"
            )
            self._reset_guard_anchor = stamp_sec
            self._reset_guard_driver = stream
            self._reset_guard_streams.clear()
        self._last_header_stamp = stamp_sec
        return True

    def _observe_monotonic_clock(self, now_monotonic: float) -> None:
        previous = self._last_monotonic
        if previous is not None and now_monotonic < previous:
            self._clear_for_reset(
                f"monotonic time moved backwards by {previous - now_monotonic:.3f} s"
            )
        self._last_monotonic = now_monotonic

    def _record_invalid(self, reason: str) -> None:
        self._rgb_pending.clear()
        self._depth_pending.clear()
        self._latest_pair = None
        self._invalid_reason = reason
        self._event_status = "invalid"
        self._pending_expired = False

    def _add(
        self,
        stream: str,
        payload: Any,
        stamp_sec: Any,
        received_monotonic: Optional[float],
        valid: bool,
    ) -> FrameSnapshot:
        stamp = self._stamp_value(stamp_sec)
        if not valid:
            self._record_invalid(f"{stream} frame was marked invalid")
            return self.snapshot(received_monotonic)
        if stamp is None:
            self._record_invalid(f"{stream} frame has no finite header timestamp")
            return self.snapshot(received_monotonic)

        received = (
            time.monotonic()
            if received_monotonic is None
            else float(received_monotonic)
        )
        if not math.isfinite(received):
            self._record_invalid(f"{stream} frame has no finite receipt timestamp")
            return self.snapshot(received_monotonic)
        self._observe_monotonic_clock(received)
        if not self._observe_header_clock(stream, stamp):
            return self.snapshot(received)
        reset_event = self._event_status == "clock_reset"

        # After a header-clock reset, require one sample from each stream in
        # the new epoch.  A delayed pre-reset sample can otherwise pair with a
        # delayed pre-reset sample and silently reintroduce the old epoch.
        if self._reset_guard_anchor is not None:
            self._reset_guard_streams.add(stream)

        self._sequence += 1
        sample = FrameSample(stream, stamp, received, payload, self._sequence)
        pending = self._rgb_pending if stream == "rgb" else self._depth_pending
        pending.append(sample)
        pending.sort(key=lambda item: (item.stamp_sec, item.sequence))
        if len(pending) > self.pending_limit:
            del pending[: len(pending) - self.pending_limit]

        self._invalid_reason = None
        if not reset_event:
            self._event_status = None
        self._pending_expired = False
        self._discard_old_pending(received)
        pair_sequence_before = self._pair_sequence
        self._try_pair()
        paired_now = self._pair_sequence > pair_sequence_before
        if paired_now and self._reset_guard_anchor is not None:
            # The first close pair confirms the new epoch.  Any unmatched
            # samples collected while the guard was active belong to an
            # ambiguous/old epoch and must not survive to a later
            # timestamp match.
            self._rgb_pending.clear()
            self._depth_pending.clear()
            self._pending_expired = False
            self._last_header_stamp = self._latest_pair.header_stamp_sec
            self._reset_guard_anchor = None
            self._reset_guard_driver = None
            self._reset_guard_streams.clear()
        if self._latest_pair is not None and (not reset_event or paired_now):
            self._event_status = None
        return self.snapshot(received)

    def add_rgb(
        self,
        payload: Any,
        stamp_sec: Any = None,
        received_monotonic: Optional[float] = None,
        *,
        stamp: Any = None,
        valid: bool = True,
    ) -> FrameSnapshot:
        """Add an RGB sample; ``stamp`` is an alias for ``stamp_sec``."""

        if stamp_sec is not None and stamp is not None:
            raise TypeError("pass either stamp_sec or stamp, not both")
        return self._add(
            "rgb",
            payload,
            stamp if stamp is not None else stamp_sec,
            received_monotonic,
            valid,
        )

    def add_depth(
        self,
        payload: Any,
        stamp_sec: Any = None,
        received_monotonic: Optional[float] = None,
        *,
        stamp: Any = None,
        valid: bool = True,
    ) -> FrameSnapshot:
        """Add a depth sample; ``stamp`` is an alias for ``stamp_sec``."""

        if stamp_sec is not None and stamp is not None:
            raise TypeError("pass either stamp_sec or stamp, not both")
        return self._add(
            "depth",
            payload,
            stamp if stamp is not None else stamp_sec,
            received_monotonic,
            valid,
        )

    def mark_invalid(self, reason: str) -> FrameSnapshot:
        """Invalidate the current output after a conversion or transport error."""

        self._record_invalid(str(reason))
        return self.snapshot(self._last_monotonic)

    def _try_pair(self) -> bool:
        if not self._rgb_pending or not self._depth_pending:
            return False

        candidates: list[tuple[float, int, int]] = []
        for rgb_index, rgb in enumerate(self._rgb_pending):
            for depth_index, depth in enumerate(self._depth_pending):
                delta = abs(rgb.stamp_sec - depth.stamp_sec)
                if delta <= self.tolerance_sec:
                    candidates.append((delta, rgb_index, depth_index))
        if not candidates:
            return False

        _, rgb_index, depth_index = min(
            candidates,
            key=lambda candidate: (
                candidate[0],
                max(
                    self._rgb_pending[candidate[1]].stamp_sec,
                    self._depth_pending[candidate[2]].stamp_sec,
                ),
            ),
        )
        rgb = self._rgb_pending.pop(rgb_index)
        depth = self._depth_pending.pop(depth_index)
        self._pair_sequence += 1
        candidate = SynchronizedFrame(rgb, depth, self._pair_sequence)
        if (
            self._latest_pair_stamp_sec is None
            or candidate.header_stamp_sec > self._latest_pair_stamp_sec
        ):
            self._latest_pair = candidate
            self._latest_pair_stamp_sec = candidate.header_stamp_sec
        return True

    def _discard_old_pending(self, now_monotonic: float) -> None:
        had_pending = bool(self._rgb_pending or self._depth_pending)
        cutoff = now_monotonic - self.stale_after_sec
        self._rgb_pending = [
            sample
            for sample in self._rgb_pending
            if sample.received_monotonic >= cutoff
        ]
        self._depth_pending = [
            sample
            for sample in self._depth_pending
            if sample.received_monotonic >= cutoff
        ]
        if had_pending and not (self._rgb_pending or self._depth_pending):
            self._pending_expired = True

    def snapshot(self, now_monotonic: Optional[float] = None) -> FrameSnapshot:
        """Return current health; only a fresh synchronized pair is valid."""

        if now_monotonic is None:
            now = time.monotonic()
        else:
            now = float(now_monotonic)
        if not math.isfinite(now):
            return FrameSnapshot("invalid", None, "monotonic time is not finite")
        self._observe_monotonic_clock(now)
        self._discard_old_pending(now)

        if self._invalid_reason is not None:
            return FrameSnapshot("invalid", None, self._invalid_reason)
        if self._event_status == "clock_reset":
            return FrameSnapshot(
                "clock_reset", None, getattr(self, "_clock_reset_reason", "clock reset")
            )

        pair = self._latest_pair
        if pair is not None:
            age = max(0.0, pair.age_sec(now))
            if age > self.stale_after_sec:
                return FrameSnapshot(
                    "stale",
                    None,
                    f"last synchronized pair is {age:.3f} s old",
                    age,
                )
            return FrameSnapshot("valid", pair, "synchronized RGB-D frame", age)

        if self._rgb_pending or self._depth_pending:
            return FrameSnapshot(
                "waiting",
                None,
                "waiting for RGB/depth headers within tolerance",
            )
        if self._pending_expired:
            return FrameSnapshot(
                "stale",
                None,
                "unmatched RGB/depth samples expired before pairing",
            )
        return FrameSnapshot("invalid", None, "no synchronized RGB-D frame")
