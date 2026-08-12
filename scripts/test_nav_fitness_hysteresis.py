#!/usr/bin/env python3
"""Deterministic tests for runtime NDT fitness hysteresis."""

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
        block_threshold=0.22,
        clear_threshold=0.18,
        bad_hold_sec=2.0,
        clear_samples=3,
    )


def main():
    latch = MODULE.FitnessHysteresis()
    for index in range(3):
        update(latch, 0.16, index * 0.05)
    assert not latch.blocked

    # Regional score variation below the clear limit remains safe.
    for index, value in enumerate((0.141, 0.124, 0.169, 0.178)):
        update(latch, value, 1.0 + index * 0.1)
        assert not latch.blocked

    # Even a value above the hard threshold is tolerated when it is brief.
    update(latch, 0.24, 2.0)
    update(latch, 0.23, 3.8)
    assert not latch.blocked
    update(latch, 0.17, 3.9)
    assert not latch.blocked

    update(latch, 0.24, 4.0)
    update(latch, 0.23, 6.01)
    assert latch.blocked
    for index in range(2):
        update(latch, 0.17, 6.1 + index * 0.05)
        assert latch.blocked
    update(latch, 0.17, 6.3)
    assert not latch.blocked

    update(latch, math.nan, 7.0)
    assert latch.blocked

    print(
        "NAV_FITNESS_HYSTERESIS_UNIT_PASS "
        "startup<=0.18 runtime_block>0.22@2.0s "
        "clear<=0.18x3 regional_jitter=PASS invalid=BLOCK"
    )


if __name__ == "__main__":
    main()
