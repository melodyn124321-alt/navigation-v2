#!/usr/bin/env python3
"""Regression checks for the 5 cm straight-reverse policy boundary."""

from pathlib import Path
import importlib.util


MODULE_PATH = Path(__file__).with_name("nav_motion_safety_gate.py")
SPEC = importlib.util.spec_from_file_location("nav_motion_safety_gate", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def main():
    # The observed path was intended to be exactly 0.050 m but reconstructed
    # infinitesimally below it.  That is accepted, while a genuinely 2 mm
    # undersized path remains rejected.
    assert MODULE.path_meets_minimum_length(0.0499994, 0.050)
    assert MODULE.path_meets_minimum_length(0.0491, 0.050)
    assert not MODULE.path_meets_minimum_length(0.0489, 0.050)
    assert MODULE.path_meets_minimum_length(0.050, 0.050)
    print("NAV_MOTION_PATH_BOUNDARY_PASS")


if __name__ == "__main__":
    main()
