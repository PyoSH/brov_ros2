"""MAVLink boot-clock tracking without a pymavlink dependency."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import operator


_UINT32_MODULUS = 1 << 32


def normalize_time_boot_ms(value) -> int | None:
    """Return a valid MAVLink ``uint32 time_boot_ms`` value or ``None``."""

    if value is None or isinstance(value, bool):
        return None
    try:
        timestamp = operator.index(value)
    except TypeError:
        return None
    if not 0 <= timestamp < _UINT32_MODULUS:
        return None
    return int(timestamp)


class BootTimeDisposition(Enum):
    """Decision made for one boot-timestamped MAVLink payload."""

    INVALID = "invalid"
    ACCEPT = "accept"
    DROP_REORDERED = "drop_reordered"
    RESET_CANDIDATE = "reset_candidate"
    RESET = "reset"
    WRAP = "wrap"


@dataclass(frozen=True)
class BootTimeObservation:
    """Normalized boot timestamp and the receiver action it implies."""

    disposition: BootTimeDisposition
    time_boot_ms: int | None

    @property
    def accept_payload(self) -> bool:
        return self.disposition in {
            BootTimeDisposition.ACCEPT,
            BootTimeDisposition.RESET,
            BootTimeDisposition.WRAP,
        }


@dataclass(frozen=True)
class _ResetCandidate:
    time_boot_ms: int
    rx_time: float


class MavlinkBootTimeTracker:
    """Classify MAVLink boot-clock samples and detect a confirmed reboot.

    A small backwards step is normal UDP reordering and is dropped.  A large
    backwards step is only promoted to :attr:`BootTimeDisposition.RESET` after
    two independent telemetry streams report a compatible low boot time.  The
    first report is a dropped ``RESET_CANDIDATE``.  This fail-closed handshake
    prevents a delayed packet from a single stream from changing the odometry
    session.

    A reset candidate must also be near the beginning of a boot.  Packets that
    jump implausibly far ahead immediately after a confirmed reset are dropped
    as stale packets from the previous boot epoch.
    """

    def __init__(
        self,
        *,
        reorder_tolerance_ms: int = 250,
        reset_coalesce_window_s: float = 2.0,
        reset_candidate_max_boot_ms: int = 300_000,
        reset_candidate_skew_ms: int = 5_000,
        wrap_window_ms: int = 600_000,
        max_forward_jump_ms: int = 10_000,
        forward_clock_slack: float = 4.0,
    ) -> None:
        if reorder_tolerance_ms < 0:
            raise ValueError("reorder_tolerance_ms must be non-negative")
        if reset_coalesce_window_s < 0.0:
            raise ValueError("reset_coalesce_window_s must be non-negative")
        if reset_candidate_max_boot_ms < 0:
            raise ValueError("reset_candidate_max_boot_ms must be non-negative")
        if reset_candidate_skew_ms < 0:
            raise ValueError("reset_candidate_skew_ms must be non-negative")
        if not 0 < wrap_window_ms < (_UINT32_MODULUS // 2):
            raise ValueError("wrap_window_ms must be inside the uint32 half range")
        if max_forward_jump_ms < 0:
            raise ValueError("max_forward_jump_ms must be non-negative")
        if forward_clock_slack < 1.0:
            raise ValueError("forward_clock_slack must be at least 1.0")

        self._reorder_tolerance_ms = int(reorder_tolerance_ms)
        # Keep the old argument name for API compatibility.  It now defines
        # the window in which two streams may confirm one reset.
        self._reset_confirmation_window_s = float(reset_coalesce_window_s)
        self._reset_candidate_max_boot_ms = int(reset_candidate_max_boot_ms)
        self._reset_candidate_skew_ms = int(reset_candidate_skew_ms)
        self._wrap_window_ms = int(wrap_window_ms)
        self._max_forward_jump_ms = int(max_forward_jump_ms)
        self._forward_clock_slack = float(forward_clock_slack)

        self._last_by_stream: dict[str, int] = {}
        self._last_rx_by_stream: dict[str, float] = {}
        self._reset_candidates: dict[str, _ResetCandidate] = {}
        self._reset_detected = False
        self._reset_count = 0
        self._wrap_count = 0
        self._last_reset_rx_time: float | None = None
        self._last_wrap_rx_time: float | None = None
        self._epoch_anchor_boot_ms: int | None = None
        self._epoch_anchor_rx_time: float | None = None

    @property
    def reset_detected(self) -> bool:
        return self._reset_detected

    @property
    def reset_count(self) -> int:
        return self._reset_count

    @property
    def wrap_count(self) -> int:
        return self._wrap_count

    @property
    def last_reset_rx_time(self) -> float | None:
        return self._last_reset_rx_time

    def _drop_expired_reset_candidates(self, rx_time: float) -> None:
        cutoff = rx_time - self._reset_confirmation_window_s
        self._reset_candidates = {
            stream: candidate
            for stream, candidate in self._reset_candidates.items()
            if candidate.rx_time >= cutoff
        }

    def _plausible_forward_progress(
        self,
        current: int,
        previous: int,
        rx_time: float,
        previous_rx_time: float,
    ) -> bool:
        """Reject an old-epoch high timestamp shortly after a reboot."""

        if self._epoch_anchor_boot_ms is None:
            return True
        elapsed_ms = max(0.0, rx_time - previous_rx_time) * 1000.0
        allowed = self._max_forward_jump_ms + self._forward_clock_slack * elapsed_ms
        return current - previous <= allowed

    def _plausible_first_sample_in_epoch(self, current: int, rx_time: float) -> bool:
        if self._epoch_anchor_boot_ms is None or self._epoch_anchor_rx_time is None:
            return True
        if current < self._epoch_anchor_boot_ms:
            return self._epoch_anchor_boot_ms - current <= self._reset_candidate_skew_ms
        elapsed_ms = max(0.0, rx_time - self._epoch_anchor_rx_time) * 1000.0
        allowed = self._max_forward_jump_ms + self._forward_clock_slack * elapsed_ms
        return current - self._epoch_anchor_boot_ms <= allowed

    def observe(
        self,
        stream: str,
        time_boot_ms,
        rx_time: float,
    ) -> BootTimeObservation:
        """Classify one timestamp and update tracking state when accepted."""

        if not stream:
            raise ValueError("stream must be non-empty")
        if not math.isfinite(rx_time):
            raise ValueError("rx_time must be finite")
        current = normalize_time_boot_ms(time_boot_ms)
        if current is None:
            return BootTimeObservation(BootTimeDisposition.INVALID, None)

        self._drop_expired_reset_candidates(rx_time)
        previous = self._last_by_stream.get(stream)
        if previous is None:
            if not self._plausible_first_sample_in_epoch(current, rx_time):
                return BootTimeObservation(
                    BootTimeDisposition.DROP_REORDERED, current
                )
            self._last_by_stream[stream] = current
            self._last_rx_by_stream[stream] = float(rx_time)
            self._reset_candidates.pop(stream, None)
            return BootTimeObservation(BootTimeDisposition.ACCEPT, current)

        previous_rx_time = self._last_rx_by_stream[stream]
        if current >= previous:
            if not self._plausible_forward_progress(
                current, previous, rx_time, previous_rx_time
            ):
                return BootTimeObservation(
                    BootTimeDisposition.DROP_REORDERED, current
                )
            self._last_by_stream[stream] = current
            self._last_rx_by_stream[stream] = float(rx_time)
            self._reset_candidates.pop(stream, None)
            return BootTimeObservation(BootTimeDisposition.ACCEPT, current)

        # A wrap is only possible when both samples are close to the uint32
        # boundary.  This is stricter than modular arithmetic alone and avoids
        # treating an arbitrary large regression as a 49.7-day wrap.
        if (
            previous >= _UINT32_MODULUS - self._wrap_window_ms
            and current <= self._wrap_window_ms
        ):
            if (
                self._last_wrap_rx_time is None
                or rx_time - self._last_wrap_rx_time
                > self._reset_confirmation_window_s
            ):
                self._wrap_count += 1
                self._last_wrap_rx_time = float(rx_time)
            self._last_by_stream[stream] = current
            self._last_rx_by_stream[stream] = float(rx_time)
            self._reset_candidates.pop(stream, None)
            return BootTimeObservation(BootTimeDisposition.WRAP, current)

        if previous - current <= self._reorder_tolerance_ms:
            return BootTimeObservation(
                BootTimeDisposition.DROP_REORDERED, current
            )

        # A reboot reported many minutes into its alleged new boot is more
        # safely treated as stale UDP data.  With a live connection a true
        # reboot is observed near time zero by both required pose streams.
        if current > self._reset_candidate_max_boot_ms:
            return BootTimeObservation(
                BootTimeDisposition.DROP_REORDERED, current
            )

        candidate = _ResetCandidate(current, float(rx_time))
        self._reset_candidates[stream] = candidate
        confirmed = any(
            other_stream != stream
            and abs(other.time_boot_ms - current) <= self._reset_candidate_skew_ms
            for other_stream, other in self._reset_candidates.items()
        )
        if not confirmed:
            return BootTimeObservation(
                BootTimeDisposition.RESET_CANDIDATE, current
            )

        self._reset_detected = True
        self._reset_count += 1
        self._last_reset_rx_time = float(rx_time)
        self._epoch_anchor_boot_ms = current
        self._epoch_anchor_rx_time = float(rx_time)
        self._last_by_stream.clear()
        self._last_rx_by_stream.clear()
        self._reset_candidates.clear()
        # The confirming payload is the first accepted sample of the new
        # epoch.  The earlier candidate remains dropped and must arrive again.
        self._last_by_stream[stream] = current
        self._last_rx_by_stream[stream] = float(rx_time)
        return BootTimeObservation(BootTimeDisposition.RESET, current)
