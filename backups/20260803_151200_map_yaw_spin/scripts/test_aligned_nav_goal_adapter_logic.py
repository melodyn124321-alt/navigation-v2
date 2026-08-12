#!/usr/bin/env python3
"""Pure logic checks for measured segmented final-heading alignment."""

import asyncio
import math
from pathlib import Path
from types import SimpleNamespace
import importlib.util


MODULE_PATH = Path(__file__).with_name("aligned_nav_goal_adapter.py")
SPEC = importlib.util.spec_from_file_location("aligned_nav_goal_adapter", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class FakeOuterHandle:
    is_cancel_requested = False


class FakeAdapter:
    run_final_alignment = MODULE.AlignedNavGoalAdapter.run_final_alignment

    def __init__(self, x, y, yaw, slip_factor=1.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.slip_factor = slip_factor
        self.small_spin_min_angle = 0.08
        self.small_spin_max_angle = 0.52
        self.final_alignment_max_angle = math.pi
        self.terminal_position_tolerance = 0.10
        self.commands = []
        self.statuses = []

    def current_base_pose(self):
        return self.x, self.y, self.yaw

    def publish_status(self, text):
        self.statuses.append(text)

    async def run_segmented_spin(self, _outer_handle, angle, _phase):
        self.commands.append(angle)
        self.yaw = wrap(self.yaw + angle * self.slip_factor)
        return True


def make_target(x, y):
    return SimpleNamespace(
        pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y),
        ),
    )


async def run_checks():
    # Automatic goals must choose forward travel even when reverse would need
    # a much smaller heading correction. Reverse remains selectable only when
    # the explicit compatibility parameter is enabled.
    forward_turn = math.radians(170.0)
    reverse_turn = math.radians(-10.0)
    mode, turn = MODULE.choose_direct_route_mode(
        forward_turn, reverse_turn, math.pi, False)
    assert mode == "forward"
    assert abs(turn - forward_turn) < 1.0e-9
    compatibility_mode, compatibility_turn = MODULE.choose_direct_route_mode(
        forward_turn, reverse_turn, math.pi, True)
    assert compatibility_mode == "straight_reverse"
    assert abs(compatibility_turn - reverse_turn) < 1.0e-9

    # The observed collision-slowed angular command was about 0.04 rad/s.
    # A 25.8 degree step needs more than 11 seconds before acceleration and
    # settling, so the old fixed 12 second allowance was guaranteed to be
    # marginal. The dynamic allowance must leave a safe completion margin.
    observed_step = math.radians(25.8)
    dynamic_allowance = MODULE.calculate_spin_time_allowance(
        observed_step, 12.0, 0.035, 1.35, 3.0)
    assert dynamic_allowance >= 20.0
    assert dynamic_allowance > observed_step / 0.04 + 5.0

    # This reproduces the latest goal's roughly -139 degree residual and
    # deliberately models 10 percent under-rotation per step.  Re-measuring
    # between steps must still converge without any step exceeding 0.52 rad.
    adapter = FakeAdapter(-1.552, 0.234, -0.828, slip_factor=0.90)
    target = make_target(-1.552, 0.234)
    succeeded = await adapter.run_final_alignment(
        FakeOuterHandle(), target, 3.01973)
    assert succeeded
    assert len(adapter.commands) >= 5
    assert max(abs(command) for command in adapter.commands) <= 0.52 + 1.0e-9
    assert abs(wrap(3.01973 - adapter.yaw)) < 0.08
    assert any(
        status.startswith("FINAL_HEADING_REACHED")
        for status in adapter.statuses
    )
    assert not any(
        status.startswith("FINAL_HEADING_SKIPPED")
        for status in adapter.statuses
    )
    print(
        "SPIN_THEN_FORWARD_POLICY_PASS automatic_reverse=False "
        "explicit_reverse_compatibility=True "
        f"spin_allowance_25.8deg={dynamic_allowance:.1f}s "
        "FINAL_ALIGNMENT_LOGIC_PASS "
        f"steps={len(adapter.commands)} "
        f"max_step_deg="
        f"{math.degrees(max(abs(c) for c in adapter.commands)):.1f} "
        f"yaw_error_deg={math.degrees(wrap(3.01973 - adapter.yaw)):+.2f}"
    )


if __name__ == "__main__":
    asyncio.run(run_checks())
