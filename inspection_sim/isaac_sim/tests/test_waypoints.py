"""Unit tests for drone/waypoints.py — aisle-aware route planner.

Pure-Python, no Isaac Sim required.
Run with: conda run -n isaac6 python -m pytest tests/test_waypoints.py -v
"""
from drone.waypoints import plan_waypoints, Waypoint


def test_plan_has_home_scan_home():
    wps = plan_waypoints("B3")
    assert len(wps) >= 5
    assert wps[0].label == "home"
    assert wps[-1].label == "home"
    assert any(w.label == "scan" for w in wps)


def test_scan_waypoint_matches_bin_map():
    from sim.bin_map import load_bin_map
    wps = plan_waypoints("A1")
    scan = next(w for w in wps if w.label == "scan")
    expected = load_bin_map()["A1"]["scan_pose"]["position"]
    assert scan.position == tuple(expected)


def test_approach_is_further_back_than_scan():
    """The aisle column waypoint (before scan) must be at a different Y than the scan pose."""
    wps = plan_waypoints("A1")
    scan = next(w for w in wps if w.label == "scan")
    # Find the aisle_col waypoint that precedes the scan
    scan_idx = next(i for i, w in enumerate(wps) if w.label == "scan")
    pre_scan = wps[scan_idx - 1]
    # The aisle_col Y (aisle centreline) should be further from the rack than the scan Y
    assert pre_scan.position[1] != scan.position[1], (
        "aisle_col Y should differ from scan Y (centreline vs. label standoff)"
    )


def test_unknown_bin_raises():
    import pytest
    with pytest.raises(KeyError):
        plan_waypoints("Z9")


def test_aisle_waypoints_present():
    """Route must include aisle_entry and aisle_exit waypoints."""
    wps = plan_waypoints("C3")
    labels = [w.label for w in wps]
    assert "aisle_entry" in labels, "Missing aisle_entry waypoint"
    assert "aisle_exit" in labels, "Missing aisle_exit waypoint"


def test_aisle_col_at_cruise_z():
    """Aisle-col waypoints should be at AISLE_CRUISE_Z (above floor obstacles)."""
    from drone.waypoints import AISLE_CRUISE_Z
    wps = plan_waypoints("B3")
    aisle_cols = [w for w in wps if w.label == "aisle_col"]
    assert len(aisle_cols) >= 1
    for w in aisle_cols:
        assert abs(w.position[2] - AISLE_CRUISE_Z) < 0.01, (
            f"aisle_col z={w.position[2]:.3f} != AISLE_CRUISE_Z={AISLE_CRUISE_Z}"
        )


def test_aisle_entry_before_scan():
    """aisle_entry must come before the scan waypoint in the route."""
    wps = plan_waypoints("A1")
    labels = [w.label for w in wps]
    assert labels.index("aisle_entry") < labels.index("scan")


def test_aisle_exit_after_scan():
    """aisle_exit must come after the scan waypoint."""
    wps = plan_waypoints("A1")
    labels = [w.label for w in wps]
    assert labels.index("aisle_exit") > labels.index("scan")
