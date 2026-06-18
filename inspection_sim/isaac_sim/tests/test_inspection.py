"""Tests for backend.inspection (M4 core compare logic).

Pure tests — no I/O, no DB, no HTTP.

Run with:
    conda run -n perception python -m pytest tests/test_inspection.py -v
"""

import pytest
from backend.inspection import compare, inspect_from_perception, InspectionResult
from perception.types import PerceptionResult


# ---------------------------------------------------------------------------
# compare() — direct field-level tests
# ---------------------------------------------------------------------------

def test_compare_match():
    """Identical part_no and qty -> match=True, status='completed'."""
    r = compare(
        bin_id="A1",
        scanned_part="PN-A01",
        scanned_qty=11,
        system_part="PN-A01",
        system_qty=11,
        latency_s=0.5,
    )
    assert isinstance(r, InspectionResult)
    assert r.match is True
    assert r.status == "completed"
    assert r.bin_id == "A1"
    assert r.latency_s == pytest.approx(0.5)


def test_compare_qty_mismatch():
    """Same part_no but different qty -> mismatch, status='discrepancy'."""
    r = compare(
        bin_id="B2",
        scanned_part="PN-B02",
        scanned_qty=18,
        system_part="PN-B02",
        system_qty=99,
        latency_s=1.2,
    )
    assert r.match is False
    assert r.status == "discrepancy"
    assert r.scanned_qty == 18
    assert r.system_qty == 99


def test_compare_part_no_mismatch():
    """Different part_no -> mismatch, status='discrepancy'."""
    r = compare(
        bin_id="C4",
        scanned_part="PN-C04",
        scanned_qty=26,
        system_part="PN-WRONG",
        system_qty=26,
        latency_s=0.8,
    )
    assert r.match is False
    assert r.status == "discrepancy"
    assert r.scanned_part == "PN-C04"
    assert r.system_part == "PN-WRONG"


def test_compare_both_mismatched():
    """Both part_no and qty differ -> discrepancy."""
    r = compare(
        bin_id="A5",
        scanned_part="PN-A05",
        scanned_qty=15,
        system_part="PN-A05",
        system_qty=50,
        latency_s=0.7,
    )
    assert r.match is False
    assert r.status == "discrepancy"


def test_compare_scanned_none():
    """If perception returns None part_no, result is discrepancy."""
    r = compare(
        bin_id="B1",
        scanned_part=None,
        scanned_qty=None,
        system_part="PN-B01",
        system_qty=17,
        latency_s=2.0,
    )
    assert r.match is False
    assert r.status == "discrepancy"


def test_compare_system_none():
    """If SAP record is missing, result is discrepancy."""
    r = compare(
        bin_id="B1",
        scanned_part="PN-B01",
        scanned_qty=17,
        system_part=None,
        system_qty=None,
        latency_s=1.0,
    )
    assert r.match is False
    assert r.status == "discrepancy"


def test_compare_timestamp_auto():
    """timestamp is set automatically when not provided."""
    r = compare("A1", "PN-A01", 11, "PN-A01", 11, latency_s=0.1)
    assert r.timestamp is not None
    assert "T" in r.timestamp   # ISO-8601 format


def test_compare_timestamp_custom():
    """Custom timestamp is preserved."""
    ts = "2026-06-06T00:00:00+00:00"
    r = compare("A1", "PN-A01", 11, "PN-A01", 11, latency_s=0.1, timestamp=ts)
    assert r.timestamp == ts


# ---------------------------------------------------------------------------
# inspect_from_perception() — convenience wrapper
# ---------------------------------------------------------------------------

def _make_perception(part_no, qty, confidence=0.9):
    return PerceptionResult(
        part_no=part_no,
        qty=qty,
        confidence=confidence,
        bbox=(0, 0, 100, 100),
        source="qr",
    )


def test_inspect_from_perception_match():
    p = _make_perception("PN-A01", 11)
    sap = {"part_no": "PN-A01", "qty": 11}
    r = inspect_from_perception("A1", p, sap, latency_s=0.3)
    assert r.match is True
    assert r.status == "completed"
    assert r.scanned_part == "PN-A01"
    assert r.scanned_qty == 11


def test_inspect_from_perception_mismatch():
    p = _make_perception("PN-B02", 18)
    sap = {"part_no": "PN-B02", "qty": 99}   # B2 discrepancy
    r = inspect_from_perception("B2", p, sap, latency_s=1.0)
    assert r.match is False
    assert r.status == "discrepancy"
    assert r.system_qty == 99


def test_inspect_from_perception_none_result():
    """Perception failed (None) -> discrepancy."""
    sap = {"part_no": "PN-C01", "qty": 23}
    r = inspect_from_perception("C1", None, sap, latency_s=0.5)
    assert r.match is False
    assert r.scanned_part is None


def test_inspect_from_perception_none_sap():
    """SAP missing -> discrepancy."""
    p = _make_perception("PN-A01", 11)
    r = inspect_from_perception("A1", p, None, latency_s=0.5)
    assert r.match is False
    assert r.system_part is None
