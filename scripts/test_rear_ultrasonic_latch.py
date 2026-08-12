#!/usr/bin/env python3
"""Deterministic tests for the final rear-ultrasonic motion gate."""

import importlib.util
import math
import sys
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("nav_motion_safety_gate.py")
SPEC = importlib.util.spec_from_file_location("nav_motion_safety_gate", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def update(latch, value, now):
    latch.update(
        value=value,
        now_sec=now,
        stop_distance=0.22,
        clear_distance=0.35,
        hold_sec=1.50,
        finite_clear_samples=3,
        no_echo_clear_samples=8,
    )


def main():
    latch = MODULE.UltrasonicLatch()
    update(latch, math.inf, 0.0)
    assert not latch.blocked

    update(latch, 0.20, 1.0)
    assert latch.blocked
    for index in range(7):
        update(latch, math.inf, 2.6 + index * 0.25)
        assert latch.blocked
    update(latch, math.inf, 4.35)
    assert not latch.blocked

    update(latch, 0.18, 5.0)
    assert latch.blocked
    update(latch, 0.40, 6.6)
    update(latch, 0.40, 6.85)
    assert latch.blocked
    update(latch, 0.40, 7.10)
    assert not latch.blocked

    update(latch, math.nan, 8.0)
    assert latch.blocked
    update(latch, 0.30, 10.0)
    assert latch.blocked

    # A calibrated vehicle self-return is ignored: it cannot create a false
    # obstacle, and it cannot clear an already confirmed real obstacle.
    gate = object.__new__(MODULE.NavMotionSafetyGate)
    gate.ultrasonic_self_echo_enabled = True
    gate.ultrasonic_self_echo_min_distance = 0.08
    gate.ultrasonic_self_echo_max_distance = 0.12
    assert gate.is_ultrasonic_self_echo(0.089)
    assert not gate.is_ultrasonic_self_echo(0.079)
    assert not gate.is_ultrasonic_self_echo(0.121)

    classify = MODULE.NavMotionSafetyGate.guarded_ultrasonic_indices
    assert classify(-0.04, 0.0, "reverse", True) == (0, 1)
    assert classify(0.04, 0.0, "reverse", True) == ()
    assert classify(0.0, 0.08, "reverse", True) == (0, 1)
    assert classify(0.0, -0.08, "reverse", True) == (0, 1)
    assert classify(0.04, 0.08, "reverse", True) == (0, 1)
    assert classify(-0.04, -0.08, "reverse", True) == (0, 1)

    print(
        "REAR_ULTRASONIC_UNIT_PASS "
        "stop=0.22m clear=0.35m hold=1.50s "
        "finite_clear=3 no_echo_clear=8 dual_side_turn_guard=PASS"
    )


if __name__ == "__main__":
    main()
