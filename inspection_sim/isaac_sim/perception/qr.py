"""QR decode module — pure Python, no Isaac/sim dependency.

NOTE: decode_payload is inlined here (instead of importing sim.gr_label)
so that the perception env does not need qrcode or other sim tooling.
The logic is identical to sim.gr_label.decode_payload.
"""
import json

import numpy as np
from PIL import Image
from pyzbar.pyzbar import decode


def _decode_payload(s):
    """Parse QR JSON payload: '{"part_no": ..., "qty": ...}' -> (part_no, qty)."""
    d = json.loads(s)
    return d["part_no"], int(d["qty"])


def decode_qr_crop(image_rgb, bbox):
    """Decode QR code from a crop of image_rgb.

    Args:
        image_rgb: HxWx3 numpy ndarray (uint8, RGB).
        bbox: (x1, y1, x2, y2) integer crop coordinates.

    Returns:
        (part_no, qty) tuple, or None if no QR found / decode fails.
    """
    x1, y1, x2, y2 = [int(v) for v in bbox]
    crop = image_rgb[max(0, y1):y2, max(0, x1):x2]
    if crop.size == 0:
        return None
    found = decode(Image.fromarray(crop))
    if not found:
        return None
    try:
        return _decode_payload(found[0].data.decode())
    except Exception:
        return None


def decode_qr_in_image(path):
    """Open image at path and decode the first QR code found.

    Returns:
        (part_no, qty) tuple, or None.
    """
    arr = np.array(Image.open(path).convert("RGB"))
    h, w = arr.shape[:2]
    return decode_qr_crop(arr, (0, 0, w, h))
