#!/usr/bin/env python3
"""Deterministic tests for the three-stage navigation speed profile."""

import importlib.util
import math
import sys
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("nav_motion_safety_gate.py")
SPEC = importlib.util.spec_from_file_location("nav_motion_safety_gate", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def decide(traveled, remaining):
    return MODULE.compute_speed_profile(
        traveled=traveled,
        remaining=remaining,
        start_distance=0.25,
        start_speed=0.08,
        approach_distance=0.70,
        approach_min_speed=0.05,
        cruise_speed=0.30,
    )


def main():
    start = decide(0.0, 4.0)
    assert start.state == "START" and math.isclose(start.cap, 0.08)

    cruise = decide(0.26, 2.0)
    assert cruise.state == "CRUISE" and math.isclose(cruise.cap, 0.30)

    approach = decide(1.0, 0.35)
    assert approach.state == "APPROACH"
    assert math.isclose(approach.cap, 0.175)

    final = decide(1.0, 0.0)
    assert final.state == "APPROACH" and math.isclose(final.cap, 0.05)

    short_goal = decide(0.0, 0.20)
    assert short_goal.state == "START_APPROACH"
    assert math.isclose(short_goal.cap, 0.08)

    print(
        "NAV_SPEED_PROFILE_UNIT_PASS "
        "start=0.080m/s@0.25m cruise=0.300m/s "
        "approach=0.70m->0.050m/s short_goal=LOW_SPEED"
    )


if __name__ == "__main__":
    main()
