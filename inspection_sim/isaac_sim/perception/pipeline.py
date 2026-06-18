"""Perception fusion pipeline.

Merges YOLO detections + QR decode + OCR into a single PerceptionResult.
QR is the authoritative source (higher confidence); OCR is the fallback.

The fuse() function is pure logic — testable with fake detections/readers.
"""
from perception.types import PerceptionResult
from perception.qr import decode_qr_crop


def fuse(image_rgb, detections, ocr_reader=None):
    """Fuse YOLO detections with QR decode and optional OCR.

    For each detected label region:
    1. Try pyzbar QR decode on the cropped region (authoritative).
    2. If QR fails and ocr_reader is provided, try PaddleOCR on the crop.
    3. Keep the result with highest confidence.

    If no detections are provided, fall back to whole-image QR decode so the
    pipeline still works even without a trained YOLO model.

    Args:
        image_rgb: HxWx3 numpy ndarray.
        detections: list of Detection objects (from LabelDetector.detect).
        ocr_reader: optional LabelOCR instance; if None, OCR is skipped.
    Returns:
        PerceptionResult, or None if nothing decoded.
    """
    best = None

    # --- Try detection-cropped regions first ---
    for d in detections:
        qr = decode_qr_crop(image_rgb, d.bbox)
        if qr:
            cand = PerceptionResult(
                part_no=qr[0],
                qty=qr[1],
                confidence=d.conf,
                bbox=d.bbox,
                source="qr",
            )
        elif ocr_reader is not None:
            pn, qty, _ = ocr_reader.read(image_rgb, d.bbox)
            if pn is None and qty is None:
                continue
            cand = PerceptionResult(
                part_no=pn,
                qty=qty,
                confidence=d.conf * 0.7,
                bbox=d.bbox,
                source="ocr",
            )
        else:
            continue

        if best is None or cand.confidence > best.confidence:
            best = cand

    # --- Fallback: whole-image QR decode (when YOLO misses the label) ---
    if best is None:
        import numpy as np
        h, w = image_rgb.shape[:2]
        qr = decode_qr_crop(image_rgb, (0, 0, w, h))
        if qr:
            best = PerceptionResult(
                part_no=qr[0],
                qty=qr[1],
                confidence=0.5,
                bbox=(0, 0, w, h),
                source="qr",
            )

    return best
