import numpy as np
from PIL import Image
from perception.types import Detection, PerceptionResult
from perception.pipeline import fuse


def test_detection_fields():
    d = Detection(cls="qr", bbox=(10, 20, 100, 120), conf=0.9)
    assert d.cls == "qr" and d.conf == 0.9 and len(d.bbox) == 4


def test_result_defaults():
    r = PerceptionResult(part_no="PN-A01", qty=11, confidence=0.95,
                         bbox=(0, 0, 1, 1), source="qr")
    assert r.part_no == "PN-A01" and r.qty == 11 and r.source == "qr"


def test_fuse_prefers_qr_from_real_label():
    arr = np.array(Image.open("sim/assets/labels/A1.png").convert("RGB"))
    h, w = arr.shape[:2]
    dets = [Detection("label", (0, 0, w, h), 0.9)]
    r = fuse(arr, dets)            # no ocr_reader -> QR path
    assert r is not None and r.part_no == "PN-A01" and r.qty == 11 and r.source == "qr"


class _FakeOCR:
    def read(self, img, bbox):
        return ("PN-Z9", 5, ["Part No: PN-Z9", "Qty: 5"])


def test_fuse_falls_back_to_ocr_when_no_qr():
    blank = np.full((50, 50, 3), 255, np.uint8)   # no QR
    dets = [Detection("label", (0, 0, 50, 50), 0.8)]
    r = fuse(blank, dets, ocr_reader=_FakeOCR())
    assert r.source == "ocr" and r.part_no == "PN-Z9" and r.qty == 5


def test_fuse_whole_image_fallback_no_detections():
    """If no detections given, fuse should still decode QR from full image."""
    arr = np.array(Image.open("sim/assets/labels/A1.png").convert("RGB"))
    r = fuse(arr, detections=[])
    assert r is not None and r.part_no == "PN-A01" and r.qty == 11 and r.source == "qr"


def test_fuse_returns_none_for_blank_image():
    blank = np.full((50, 50, 3), 255, np.uint8)
    r = fuse(blank, detections=[])
    assert r is None
