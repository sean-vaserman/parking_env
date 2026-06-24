"""
Headless test runner for the parking simulation.

Usage:
    python test_cases.py           # run all cases, suppress planner output
    python test_cases.py --verbose # show planner progress per case

Exit code: 0 if all cases pass, 1 otherwise.
"""
import contextlib
import io
import math
import sys
import time

import matplotlib
matplotlib.use('Agg')  # must be set before main is imported (prevents TkAgg GUI)
import numpy as np

from kinematics import VehicleModel
from stanley_controller import StanleyController
from hybrid_a_star import plan_path
from main import OBSTACLES, advance_target, ADVANCE_DIST, BRAKE_DIST, STOP_V

GOAL = (11.5, 15.5, math.pi / 2)

# Each entry: display name and (x, y, yaw_deg) start pose.
# All starts are within the safe slider range: x∈[7.1,16.9], y∈[1.5,3.0].
TEST_CASES = [
    ("centre, straight north",  11.5, 2.5,  90),
    ("left,   60° heading",      8.5, 2.5,  60),
    ("right,  120° heading",    14.5, 2.5, 120),
    ("left,   45° heading",      9.0, 2.0,  45),
    ("right,  135° heading",    13.5, 2.0, 135),
    ("centre, 75° heading",     11.0, 2.5,  75),
    ("centre, 105° heading",    12.0, 2.5, 105),
]


def run_case(start, goal, *, verbose=False, max_steps=3000, dt=0.1):
    """
    Run one simulation headlessly.
    Returns (parked: bool, sim_elapsed_s: float, note: str).
    """
    vehicle    = VehicleModel(*start)
    controller = StanleyController(k=1.2)

    _null = io.StringIO()
    ctx   = contextlib.nullcontext() if verbose else contextlib.redirect_stdout(_null)
    with ctx:
        px, py, pyaw, pgear = plan_path(start, goal, OBSTACLES, vehicle)

    if not px:
        return False, 0.0, "no path found"

    target_idx              = 0
    last_idx                = len(px) - 1
    braking_for_gear_change = False

    for step in range(max_steps):
        prev_idx   = target_idx
        target_idx = advance_target(vehicle, px, py, pgear, target_idx)
        if target_idx != prev_idx:
            braking_for_gear_change = False

        if braking_for_gear_change and abs(vehicle.v) < STOP_V and target_idx < last_idx:
            target_idx += 1
            braking_for_gear_change = False

        gear  = pgear[target_idx]
        steer = controller.control(vehicle, px, py, pyaw, pgear, target_idx)

        dist_to_target      = math.hypot(vehicle.x - px[target_idx], vehicle.y - py[target_idx])
        next_is_gear_change = target_idx < last_idx and pgear[target_idx + 1] != gear

        if target_idx >= last_idx:
            desired_v = 0.0
        elif next_is_gear_change:
            if dist_to_target < BRAKE_DIST:
                braking_for_gear_change = True
            desired_v = 0.0 if braking_for_gear_change else 2.0 * gear
        else:
            braking_for_gear_change = False
            desired_v = 2.0 * gear

        v_err    = desired_v - vehicle.v
        throttle = (3.0 if desired_v * vehicle.v < 0 and abs(vehicle.v) > 0.5 else 1.0) * v_err
        throttle = np.clip(throttle, -3.0, 2.0)

        if desired_v == 0.0 and abs(vehicle.v) < 0.1 and target_idx >= last_idx:
            dist_goal = math.hypot(vehicle.x - goal[0], vehicle.y - goal[1])
            yaw_err   = abs((vehicle.yaw - goal[2] + math.pi) % (2 * math.pi) - math.pi)
            return True, step * dt, f"dist={dist_goal:.2f}m  yaw_err={math.degrees(yaw_err):.1f}°"

        vehicle.update(throttle, steer, dt)

    dist_final = math.hypot(vehicle.x - goal[0], vehicle.y - goal[1])
    return False, max_steps * dt, f"timeout (dist_to_goal={dist_final:.1f} m)"


if __name__ == "__main__":
    verbose = "--verbose" in sys.argv

    col_w = 30
    print(f"\n{'Case':<{col_w}}  {'Result':<6}  {'Sim':>8}  {'Wall':>7}  Note")
    print("─" * (col_w + 34))

    passed = 0
    for name, sx, sy, syaw_deg in TEST_CASES:
        start   = (sx, sy, math.radians(syaw_deg))
        t0      = time.perf_counter()
        ok, sim_t, note = run_case(start, GOAL, verbose=verbose)
        wall_t  = time.perf_counter() - t0
        tag     = "PASS" if ok else "FAIL"
        passed += ok
        print(f"{name:<{col_w}}  {tag:<6}  {sim_t:>6.1f}s  {wall_t:>5.1f}s  {note}")

    print("─" * (col_w + 34))
    print(f"{passed}/{len(TEST_CASES)} passed\n")
    sys.exit(0 if passed == len(TEST_CASES) else 1)
