"""End-to-end perception verification on sim/assets/capture_A1.png.

Runs pre-trained YOLOv8 QR detection (qrdet) → fuse QR decode + PaddleOCR →
prints PERCEPTION_OK. Uses REAL pixels from the rendered image (no faking).
No custom training — the QR detector ships pre-trained weights.

Expected output:
  detections=<n>
  PERCEPTION_OK source=qr part_no=PN-A01 qty=11 conf=...
"""
import os
import sys

import numpy as np
from PIL import Image

# Ensure repo root is on the path when running directly
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from perception.detector import QRCodeDetector
from perception.ocr import LabelOCR
from perception.pipeline import fuse

IMAGE = "sim/assets/capture_A1.png"

assert os.path.exists(IMAGE), f"Image not found: {IMAGE}"

img = np.array(Image.open(IMAGE).convert("RGB"))
print(f"image shape: {img.shape}")

# Pre-trained QR detection (qrdet) — no training required
det = QRCodeDetector(conf=0.3)
dets = det.detect(img)
print(f"detections={len(dets)}")
for d in dets:
    print(f"  detection: cls={d.cls} bbox={d.bbox} conf={d.conf:.3f}")

# Fuse with QR + LabelOCR
ocr = LabelOCR()
res = fuse(img, dets, ocr_reader=ocr)

assert res is not None, (
    "No perception result — QR decode + OCR both failed. "
    "Check that capture_A1.png contains a readable QR code."
)

print(f"PERCEPTION_OK source={res.source} part_no={res.part_no} qty={res.qty} conf={res.confidence:.2f}")

assert res.part_no == "PN-A01", (
    f"Wrong part_no: expected 'PN-A01', got {res.part_no!r}"
)
print("Assertion PN-A01: PASS")
