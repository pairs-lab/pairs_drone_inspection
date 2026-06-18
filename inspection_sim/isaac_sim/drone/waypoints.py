"""Aisle-aware waypoint planner for the narrow-corridor warehouse demo.

Route (for a bin on the primary rack):
    home
    -> aisle_entrance  (open end of aisle, aligned with aisle centreline)
    -> aisle_column    (travel along centreline to the target column's X)
    -> scan            (step laterally to face the bin label)
    -> aisle_column    (back to centreline)
    -> aisle_exit      (exit end of aisle)
    -> home

This makes the drone visibly thread the narrow corridor rather than flying
straight through rack geometry.

The aisle_entrance / aisle_exit are placed BEFORE the racks start in X
(i.e. at x < RACK_X_MIN) so the drone approaches from the open end.
"""
from dataclasses import dataclass
from sim.bin_map import load_bin_map
from sim.config import (
    HOME_POSE,
    RACK_X_MIN, RACK_X_MAX,
    AISLE_CENTER_Y,
    COLUMNS, COLUMN_SPACING,
)

APPROACH_EXTRA_STANDOFF = 0.8  # m further back than scan pose for a safe approach

# Z altitude used while travelling through the aisle (clear of low obstacles)
AISLE_CRUISE_Z = 1.2   # m — above the 0.6 m floor obstacle

# X offsets for the aisle entrance / exit points (outside the rack span)
AISLE_ENTRY_X = RACK_X_MIN - 0.5   # m — clear of rack front posts
AISLE_EXIT_X  = RACK_X_MAX + 0.5   # m — clear of rack back posts


@dataclass(frozen=True)
class Waypoint:
    position: tuple   # (x, y, z) world meters
    yaw_deg: float
    label: str        # "home" | "aisle_entry" | "aisle_col" | "scan" | "aisle_exit"


def plan_waypoints(bin_id):
    """Plan an aisle-threading route to bin_id and back to home.

    Returns a list of Waypoints: home -> aisle -> scan -> aisle -> home.
    The scan waypoint position EXACTLY matches bin_map scan_pose.position
    (unchanged from M2 — bin_map and perception are unaffected).

    Route design:
      home  → pre_entry: fly to (HOME_X, AISLE_CENTER_Y, CRUISE_Z) — this is at the
              home X position but inside the aisle Y band, avoiding the second rack face.
              Home is at y=-2 (below second rack y_max=-1.35), so we must enter the
              aisle in +Y; doing this at HOME_X (far from rack X boundary) avoids
              any APF rack-face equilibrium.
      → aisle_entry: fly in +X to the rack entrance (AISLE_ENTRY_X, AISLE_CENTER_Y)
      → aisle_col: traverse aisle to target column X
      → scan: descend + step to scan the bin label
      → aisle_col: return to centreline
      → aisle_exit: exit the aisle in +X
      → home: return (via direct path)
    """
    bins = load_bin_map()
    if bin_id not in bins:
        raise KeyError(f"unknown bin {bin_id}")

    b = bins[bin_id]
    scan = b["scan_pose"]
    sx, sy, sz = scan["position"]
    yaw = scan["yaw_deg"]

    home_xyz = tuple(HOME_POSE["position"])
    home_x = home_xyz[0]

    home = Waypoint(home_xyz, HOME_POSE["yaw_deg"], "home")

    # 0b. Pre-entry: move +Y at HOME_X to reach the aisle Y band BEFORE crossing
    #     the rack X boundary.  This avoids the symmetric APF corner trap at x=RACK_X_MIN.
    pre_entry = Waypoint((home_x, AISLE_CENTER_Y, AISLE_CRUISE_Z), yaw, "aisle_entry")

    # 1. Aisle entrance: travel in +X along aisle centerline to rack boundary
    entry = Waypoint((AISLE_ENTRY_X, AISLE_CENTER_Y, AISLE_CRUISE_Z), yaw, "aisle_entry")

    # 2. Aisle column: traverse to the target column X
    col_x = sx
    aisle_col = Waypoint((col_x, AISLE_CENTER_Y, AISLE_CRUISE_Z), yaw, "aisle_col")

    # 3. Scan: descend + step to face label (uses exact bin_map position)
    scan_wp = Waypoint((sx, sy, sz), yaw, "scan")

    # 4. Return to aisle centreline at cruise altitude
    aisle_col_return = Waypoint((col_x, AISLE_CENTER_Y, AISLE_CRUISE_Z), yaw, "aisle_col")

    # 5. Aisle exit: +X end of aisle
    exit_wp = Waypoint((AISLE_EXIT_X, AISLE_CENTER_Y, AISLE_CRUISE_Z), yaw, "aisle_exit")

    # 6. Post-exit: move to home Y while at EXIT_X, BEFORE flying diagonally back.
    #    This ensures the return-home path doesn't clip back through the second rack
    #    aisle-face boundary (which is within the rack X span [-0.5..2.9]).
    post_exit = Waypoint((AISLE_EXIT_X, HOME_POSE["position"][1], AISLE_CRUISE_Z), yaw, "aisle_exit")

    return [home, pre_entry, entry, aisle_col, scan_wp, aisle_col_return, exit_wp, post_exit, home]
