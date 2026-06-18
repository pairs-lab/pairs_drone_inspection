"""PaddleOCR wrapper + tolerant label-text parser.

PaddleOCR install notes:
  - paddleocr==2.7.3 + paddlepaddle==2.6.2 + numpy<2 (1.26.4)
  - paddlepaddle 3.x fails with NotImplementedError on Intel OneDNN
  - paddle writes model cache to ~/.paddleocr; if that dir is root-owned,
    set HOME=/tmp/paddle_home before importing (LabelOCR._ensure does this)
  - ocr.ocr(img, cls=True) returns list-of-pages; page 0 is list of text blocks;
    each block: [[[x,y],...], (text, conf)]
    if no text, page is None → guard with `for block in (page or [])`

PaddleOCR 2.7.3 init params (compared to 3.x which dropped these):
  use_angle_cls=True, lang='en', show_log=False
"""
import os
import re


def parse_label_text(lines):
    """Extract (part_no, qty) from OCR text lines. Tolerant to case/spacing.

    Args:
        lines: list of str, each one OCR-detected text line.
    Returns:
        (part_no, qty) where either may be None if not found.
    """
    text = " \n ".join(lines)
    pn = None
    qty = None

    # Part No: match "part no" (with optional dots/colons/spaces) followed by value
    # Value pattern: letters+digits with optional hyphen separating prefix from number
    m = re.search(
        r"part\s*no\.?\s*:?\s*([A-Za-z]{1,4}-?[A-Za-z0-9]+)",
        text, re.I
    )
    if m:
        pn = m.group(1).upper().replace(" ", "")

    # Qty: match digits
    m = re.search(r"qty\s*:?\s*(\d+)", text, re.I)
    if m:
        qty = int(m.group(1))

    return pn, qty


class LabelOCR:
    """Lazy PaddleOCR wrapper. Loads model on first call.

    Note: HOME is overridden to /tmp/paddle_home to avoid permission issues
    with root-owned ~/.paddleocr on this machine. Safe for POC use.
    """

    def __init__(self, lang="en"):
        self._ocr = None
        self._lang = lang

    def _ensure(self):
        if self._ocr is None:
            # Override HOME so paddle can write model cache
            if not os.access(os.path.expanduser("~/.paddleocr"), os.W_OK):
                os.environ["HOME"] = "/tmp/paddle_home"
                os.makedirs("/tmp/paddle_home", exist_ok=True)
            try:
                from paddleocr import PaddleOCR
                self._ocr = PaddleOCR(
                    use_angle_cls=True,
                    lang=self._lang,
                    show_log=False,
                )
            except Exception as e:
                # Graceful degrade: PaddleOCR unavailable
                self._ocr = None
                self._unavailable = str(e)

    def read(self, image_rgb, bbox):
        """Run OCR on a crop of image_rgb within bbox.

        Args:
            image_rgb: HxWx3 numpy ndarray.
            bbox: (x1, y1, x2, y2) crop coordinates.
        Returns:
            (part_no, qty, raw_lines) — part_no/qty may be None if not found.
            If PaddleOCR is unavailable, returns (None, None, []).
        """
        self._ensure()
        if self._ocr is None:
            return None, None, []

        import numpy as np
        x1, y1, x2, y2 = [int(v) for v in bbox]
        crop = image_rgb[max(0, y1):y2, max(0, x1):x2]
        if crop.size == 0:
            return None, None, []

        try:
            result = self._ocr.ocr(crop, cls=True)
        except Exception:
            return None, None, []

        # result: list-of-pages; page 0 is list of text blocks or None
        # Each block: [[[x,y], ...], (text, conf)]
        lines = []
        for page in (result or []):
            for block in (page or []):
                try:
                    lines.append(block[1][0])
                except (IndexError, TypeError):
                    pass

        pn, qty = parse_label_text(lines)
        return pn, qty, lines
