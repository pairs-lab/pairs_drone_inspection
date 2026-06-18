"""Tests for backend.sap_mock (M5).

Run with:
    conda run -n perception python -m pytest tests/test_sap_mock.py -v
"""

import os
import pytest
import tempfile

# Use a fresh temp DB for each test to avoid cross-test pollution
@pytest.fixture()
def db_path(tmp_path):
    return str(tmp_path / "test_sap.db")


def test_seed_returns_18_rows(db_path):
    from backend.sap_mock import seed_from_bin_map
    n = seed_from_bin_map(db_path=db_path)
    assert n == 18, f"Expected 18 rows, got {n}"


def test_seed_idempotent(db_path):
    """Calling seed_from_bin_map twice without force=True should not re-insert."""
    from backend.sap_mock import seed_from_bin_map
    n1 = seed_from_bin_map(db_path=db_path)
    n2 = seed_from_bin_map(db_path=db_path)   # second call
    assert n1 == 18
    assert n2 == 0, "Expected 0 (already seeded) on second call without force"


def test_seed_force_reseeds(db_path):
    from backend.sap_mock import seed_from_bin_map
    seed_from_bin_map(db_path=db_path)
    n = seed_from_bin_map(db_path=db_path, force=True)
    assert n == 18


def test_get_inventory_matching_bin(db_path):
    """A non-discrepancy bin should return ground-truth values."""
    from backend.sap_mock import seed_from_bin_map, get_inventory
    seed_from_bin_map(db_path=db_path)
    rec = get_inventory("A1", db_path=db_path)
    assert rec is not None
    assert rec["bin_id"] == "A1"
    assert rec["part_no"] == "PN-A01"
    assert rec["qty"] == 11


def test_get_inventory_discrepancy_b2(db_path):
    """B2 should have SAP qty=99 (intentional discrepancy)."""
    from backend.sap_mock import seed_from_bin_map, get_inventory, DISCREPANCY_BINS
    seed_from_bin_map(db_path=db_path)
    rec = get_inventory("B2", db_path=db_path)
    assert rec is not None
    assert rec["qty"] == DISCREPANCY_BINS["B2"]["qty"], \
        f"B2 qty mismatch: expected {DISCREPANCY_BINS['B2']['qty']}, got {rec['qty']}"


def test_get_inventory_discrepancy_c4(db_path):
    """C4 should have SAP part_no='PN-WRONG' (intentional discrepancy)."""
    from backend.sap_mock import seed_from_bin_map, get_inventory, DISCREPANCY_BINS
    seed_from_bin_map(db_path=db_path)
    rec = get_inventory("C4", db_path=db_path)
    assert rec is not None
    assert rec["part_no"] == DISCREPANCY_BINS["C4"]["part_no"]


def test_get_inventory_discrepancy_a5(db_path):
    """A5 should have SAP qty=50 (intentional discrepancy)."""
    from backend.sap_mock import seed_from_bin_map, get_inventory, DISCREPANCY_BINS
    seed_from_bin_map(db_path=db_path)
    rec = get_inventory("A5", db_path=db_path)
    assert rec is not None
    assert rec["qty"] == DISCREPANCY_BINS["A5"]["qty"]


def test_get_inventory_not_found(db_path):
    """Querying an unknown bin returns None."""
    from backend.sap_mock import seed_from_bin_map, get_inventory
    seed_from_bin_map(db_path=db_path)
    rec = get_inventory("Z9", db_path=db_path)
    assert rec is None


def test_set_inventory_creates_and_updates(db_path):
    """set_inventory should insert and allow update."""
    from backend.sap_mock import seed_from_bin_map, get_inventory, set_inventory
    seed_from_bin_map(db_path=db_path)

    # Correct B2 after re-verification
    set_inventory("B2", "PN-B02", 18, db_path=db_path)
    rec = get_inventory("B2", db_path=db_path)
    assert rec["part_no"] == "PN-B02"
    assert rec["qty"] == 18


def test_set_inventory_new_bin(db_path):
    """set_inventory should also work for a new bin not in seed."""
    from backend.sap_mock import set_inventory, get_inventory
    set_inventory("X9", "PN-X09", 42, db_path=db_path)
    rec = get_inventory("X9", db_path=db_path)
    assert rec is not None
    assert rec["part_no"] == "PN-X09"
    assert rec["qty"] == 42


def test_discrepancy_bins_constant_has_three_entries():
    from backend.sap_mock import DISCREPANCY_BINS
    assert len(DISCREPANCY_BINS) == 3, "Expected exactly 3 intentional discrepancy bins"
    assert "B2" in DISCREPANCY_BINS
    assert "C4" in DISCREPANCY_BINS
    assert "A5" in DISCREPANCY_BINS
