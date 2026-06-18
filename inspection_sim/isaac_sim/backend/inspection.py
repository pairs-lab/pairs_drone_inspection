"""M4 — Inspection core logic (pure, no I/O).

Given a PerceptionResult (scanned from camera) and a SAP inventory record
(system of record), produce an InspectionResult with match/mismatch decision.

Rules
-----
- match = True  iff scanned_part == system_part AND scanned_qty == system_qty
- status = "completed"    when match is True
- status = "discrepancy"  when match is False (triggers alert in app layer)
- SOR KPI: latency must be < 10 s (asserted/logged by caller)

This module has zero I/O: pass in the data, get the result.  Ideal for unit tests.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class InspectionResult:
    """Result of a single bin inspection."""

    bin_id: str
    scanned_part: Optional[str]     # part_no from camera perception
    scanned_qty: Optional[int]      # qty from camera perception
    system_part: Optional[str]      # part_no from SAP
    system_qty: Optional[int]       # qty from SAP
    match: bool                     # True iff scanned == system on both fields
    status: str                     # "completed" | "discrepancy"
    latency_s: float                # wall-clock seconds from scan start to result
    timestamp: str                  # ISO-8601 UTC timestamp of inspection
    # Annotated camera frame URL (set after visualize step; None before first inspect)
    annotated_url: Optional[str] = None
    # Perception metadata for UI display
    bbox: Optional[tuple] = None
    confidence: Optional[float] = None
    source: Optional[str] = None


def compare(
    bin_id: str,
    scanned_part: Optional[str],
    scanned_qty: Optional[int],
    system_part: Optional[str],
    system_qty: Optional[int],
    latency_s: float,
    timestamp: Optional[str] = None,
    annotated_url: Optional[str] = None,
    bbox: Optional[tuple] = None,
    confidence: Optional[float] = None,
    source: Optional[str] = None,
) -> InspectionResult:
    """Compare scanned perception result against SAP system record.

    Args:
        bin_id: Bin identifier (e.g. "A1").
        scanned_part: Part number detected by camera pipeline.
        scanned_qty: Quantity detected by camera pipeline.
        system_part: Part number from SAP inventory.
        system_qty: Quantity from SAP inventory.
        latency_s: Elapsed seconds from scan start to result ready.
        timestamp: Optional ISO-8601 UTC string; defaults to now().
        annotated_url: URL to annotated camera frame image.
        bbox: Bounding box tuple (x1, y1, x2, y2) of detected label.
        confidence: Perception confidence score.
        source: Perception source ("qr" | "ocr").

    Returns:
        InspectionResult with match=True/False and status accordingly.
    """
    if timestamp is None:
        from datetime import datetime, timezone
        timestamp = datetime.now(timezone.utc).isoformat()

    match = (
        scanned_part is not None
        and scanned_qty is not None
        and scanned_part == system_part
        and scanned_qty == system_qty
    )
    status = "completed" if match else "discrepancy"

    return InspectionResult(
        bin_id=bin_id,
        scanned_part=scanned_part,
        scanned_qty=scanned_qty,
        system_part=system_part,
        system_qty=system_qty,
        match=match,
        status=status,
        latency_s=latency_s,
        timestamp=timestamp,
        annotated_url=annotated_url,
        bbox=bbox,
        confidence=confidence,
        source=source,
    )


def inspect_from_perception(
    bin_id: str,
    perception_result,
    sap_record: Optional[dict],
    latency_s: float,
    timestamp: Optional[str] = None,
    annotated_url: Optional[str] = None,
) -> InspectionResult:
    """Convenience wrapper that unpacks PerceptionResult and SAP dict.

    Args:
        bin_id: Bin identifier.
        perception_result: PerceptionResult dataclass (or None if scan failed).
        sap_record: dict with keys "part_no" and "qty" (or None if not found).
        latency_s: Elapsed seconds.
        timestamp: Optional ISO-8601 UTC string.
        annotated_url: URL to annotated camera frame image.

    Returns:
        InspectionResult.
    """
    if perception_result is not None:
        scanned_part = perception_result.part_no
        scanned_qty = perception_result.qty
        bbox = perception_result.bbox
        confidence = perception_result.confidence
        source = perception_result.source
    else:
        scanned_part = None
        scanned_qty = None
        bbox = None
        confidence = None
        source = None

    if sap_record is not None:
        system_part = sap_record.get("part_no")
        system_qty = sap_record.get("qty")
    else:
        system_part = None
        system_qty = None

    return compare(
        bin_id=bin_id,
        scanned_part=scanned_part,
        scanned_qty=scanned_qty,
        system_part=system_part,
        system_qty=system_qty,
        latency_s=latency_s,
        timestamp=timestamp,
        annotated_url=annotated_url,
        bbox=bbox,
        confidence=confidence,
        source=source,
    )
