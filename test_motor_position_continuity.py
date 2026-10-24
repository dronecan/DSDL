#!/usr/bin/env python3
"""Pure Python reference safety policy for MotorPosition; no packages needed.

This checks consumer-side acceptance policy, NOT a DroneCAN firmware implementation.
Run: python3 test_motor_position_continuity.py
"""
import math
from dataclasses import dataclass

MASK32 = (1 << 32) - 1


def ticks_delta(new, old):
    """Signed modular delta; valid only when the physical bound disambiguates wrap."""
    return ((new - old + (1 << 31)) & MASK32) - (1 << 31)


@dataclass(frozen=True)
class Sample:
    node_id: int
    esc_index: int
    origin_id: int
    sample_seq: int
    recv_s: float
    value: float | int
    valid: bool = True
    kind: str = 'ticks'
    tpr: int = 42


class Receiver:
    """Fail-closed reference: returns accepted mechanical revolutions or None.

    All discontinuities discard the delta and use the current valid sample as
    a NEW baseline; one cannot retroactively recover the distance during a gap.
    """

    def __init__(self, max_rpm=1800.0, max_interval_s=0.25,
                 margin_revs=0.01, max_publish_hz=100):
        assert max_rpm > 0 and max_interval_s > 0 and max_publish_hz > 0
        # If this is false, an entire 256-count sequence wrap could be lost
        # inside the trusted time window, and a gap could look consecutive.
        assert max_interval_s * max_publish_hz < 255
        self.last = {}
        self.max_rps = max_rpm / 60.0
        self.max_interval_s = max_interval_s
        self.margin_revs = margin_revs

    def receive(self, s):
        key = (s.node_id, s.esc_index)
        if not s.valid or not 0 <= s.esc_index <= 31 or not 0 <= s.origin_id <= MASK32 \
                or not 0 <= s.sample_seq <= 255 or not math.isfinite(s.recv_s) \
                or s.kind not in ('ticks', 'revolutions') or \
                (s.kind == 'ticks' and (not isinstance(s.value, int) or
                 not -(1 << 31) <= s.value < (1 << 31) or not 1 <= s.tpr <= 65535)) \
                or (s.kind == 'revolutions' and not math.isfinite(s.value)):
            self.last.pop(key, None)
            return None

        prev = self.last.get(key)
        if prev is None:
            self.last[key] = s
            return None

        dt = s.recv_s - prev.recv_s
        seq_delta = (s.sample_seq - prev.sample_seq) % 256
        if seq_delta == 0:  # duplicate; retain last accepted baseline
            return None
        if (s.origin_id != prev.origin_id or s.kind != prev.kind or
            (s.kind == 'ticks' and s.tpr != prev.tpr) or
            seq_delta != 1 or not 0 < dt <= self.max_interval_s):
            self.last[key] = s
            return None

        if s.kind == 'ticks':
            delta_ticks = ticks_delta(s.value, prev.value)
            delta = delta_ticks / s.tpr
        else:
            delta = s.value - prev.value
        # A huge delta can be a counter reset, wrap ambiguity, or an error.
        # Reject, then re-baseline. max_rpm is configured for this motor.
        if not math.isfinite(delta) or abs(delta) > self.max_rps * dt + self.margin_revs:
            self.last[key] = s
            return None
        self.last[key] = s
        return delta


def s(seq, count, t, *, origin=17, node=12, esc=0, valid=True,
      kind='ticks', tpr=42):
    return Sample(node, esc, origin, seq, t, count, valid, kind, tpr)


def main():
    checks = 0

    def case(actual, expected):
        nonlocal checks
        assert actual == expected, f'{checks + 1}: {actual!r} != {expected!r}'
        checks += 1

    r = Receiver()
    case(r.receive(s(10, 1000, 0.00)), None)             # first baseline
    case(r.receive(s(11, 1042, 0.05)), 1.0)              # forward
    case(r.receive(s(12, 1000, 0.10)), -1.0)             # reverse
    case(r.receive(s(12, 999999, 0.11)), None)           # duplicate
    case(r.receive(s(13, 1042, 0.15)), 1.0)              # baseline retained
    case(r.receive(s(14, -1000, 0.20, origin=18)), None) # reboot/re-zero
    case(r.receive(s(15, -958, 0.25, origin=18)), 1.0)
    case(r.receive(s(17, 10000, 0.35, origin=18)), None) # missed seq => rebase
    case(r.receive(s(18, 10042, 0.40, origin=18)), 1.0)
    case(r.receive(s(19, 99999, 0.45, origin=18)), None) # implausible jump
    case(r.receive(s(20, 100041, 0.50, origin=18)), 1.0)
    case(r.receive(s(21, 0, 0.55, origin=18, valid=False)), None)
    case(r.receive(s(22, 999999, 0.60, origin=18)), None)
    case(r.receive(s(23, 1000041, 0.65, origin=18)), 1.0)
    case(r.receive(s(24, 1, 2.00, origin=18)), None)     # source stale
    case(r.receive(s(25, 43, 2.05, origin=18)), 1.0)
    case(r.receive(s(26, 43, 2.10, origin=18, tpr=84)), None) # scaling change
    case(r.receive(s(27, 127, 2.15, origin=18, tpr=84)), 1.0)
    case(r.receive(s(28, 1.25, 2.20, origin=18, kind='revolutions')), None)
    case(r.receive(s(29, 2.25, 2.25, origin=18, kind='revolutions')), 1.0)
    case(r.receive(s(30, float('nan'), 2.30, origin=18, kind='revolutions')), None)
    case(r.receive(s(31, 999.0, 2.35, origin=18, kind='revolutions')), None)
    case(r.receive(s(32, 1000.0, 2.40, origin=18, kind='revolutions')), 1.0)

    # Two's complement int32 wrap forwards and backwards.
    case(ticks_delta(-(1 << 31), (1 << 31) - 1), 1)
    case(ticks_delta((1 << 31) - 1, -(1 << 31)), -1)
    w = Receiver(max_rpm=2000)
    case(w.receive(s(254, (1 << 31) - 10, 0.00)), None)
    case(w.receive(s(255, -(1 << 31) + 32, 0.05)), 1.0)
    case(w.receive(s(0, -(1 << 31) + 74, 0.10)), 1.0)
    case(w.receive(s(1, (1 << 31) - 10, 0.15)), None) # huge reverse unwrap

    # A different node or esc index always has an independent baseline.
    case(r.receive(s(33, 1000, 2.45, node=13, origin=18)), None)
    case(r.receive(s(33, 1000, 2.45, esc=1, origin=18)), None)

    # Same-looking counter after a new origin is not integrated.
    case(r.receive(s(34, 1099, 2.50, origin=19, kind='revolutions')), None)

    # Two resets between received messages cannot cancel each other as they
    # would with the old parity-based `origin_toggle` bit.
    double_reset = Receiver()
    case(double_reset.receive(s(1, 3000, 1.00, origin=100)), None)
    case(double_reset.receive(s(2, -3000, 1.05, origin=102)), None)
    case(double_reset.receive(s(3, -2958, 1.10, origin=102)), 1.0)
    # Publisher MUST change origin_id on reboot, including if it reboots twice.
    print(f'OK: {checks} fail-closed continuity/odometry checks')


if __name__ == '__main__':
    main()
