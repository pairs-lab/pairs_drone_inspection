"""backend/visualize.py — Annotate drone camera captures with detection overlays.

Draws:
  - A bounding box around the detected QR/label region (GREEN if completed, RED if discrepancy).
  - A confidence label near the top-left of the box ("QR 0.91").
  - A text info panel (semi-transparent backdrop) with decoded part/qty/source.
  - Raw OCR lines if available.

Saves output to sim/assets/annotated/<bin_id>.png and returns the path.
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
from PIL import Image, ImageDraw, ImageFont

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
REPO_ROOT = Path(os.getcwd())
CAPTURES_DIR = REPO_ROOT / "sim" / "assets" / "captures"
ANNOTATED_DIR = REPO_ROOT / "sim" / "assets" / "annotated"

# ---------------------------------------------------------------------------
# Color palette
# ---------------------------------------------------------------------------
GREEN     = (34, 197, 94)       # #22c55e
RED       = (239, 68, 68)       # #ef4444
YELLOW    = (234, 179, 8)       # #eab308
WHITE     = (255, 255, 255)
BLACK     = (0, 0, 0)
DARK_BG   = (10, 14, 22, 200)   # semi-transparent dark background for text panel


def _load_font(size: int) -> ImageFont.FreeTypeFont:
    """Load a TrueType font with fallback to PIL default."""
    candidates = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationMono-Bold.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
        "/usr/share/fonts/truetype/ubuntu/Ubuntu-B.ttf",
    ]
    for path in candidates:
        if Path(path).exists():
            try:
                return ImageFont.truetype(path, size)
            except Exception:
                continue
    # Absolute fallback — PIL built-in (no size control)
    return ImageFont.load_default()


def _load_font_small(size: int) -> ImageFont.FreeTypeFont:
    """Load a smaller regular font for OCR lines."""
    candidates = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf",
    ]
    for path in candidates:
        if Path(path).exists():
            try:
                return ImageFont.truetype(path, size)
            except Exception:
                continue
    return ImageFont.load_default()


def _draw_rect_with_border(draw: ImageDraw.ImageDraw, bbox: Tuple[int, int, int, int],
                            color: Tuple[int, int, int], width: int = 3) -> None:
    """Draw a rectangle outline with the given color and line width."""
    x1, y1, x2, y2 = bbox
    for i in range(width):
        draw.rectangle([x1 - i, y1 - i, x2 + i, y2 + i], outline=color)


def _draw_text_with_shadow(draw: ImageDraw.ImageDraw, xy: Tuple[int, int],
                            text: str, font: ImageFont.FreeTypeFont,
                            color: Tuple[int, int, int],
                            shadow: Tuple[int, int, int] = BLACK) -> None:
    """Draw text with a 1-pixel shadow for legibility."""
    sx, sy = xy
    draw.text((sx + 1, sy + 1), text, font=font, fill=shadow)
    draw.text((sx - 1, sy + 1), text, font=font, fill=shadow)
    draw.text((sx, sy), text, font=font, fill=color)


def annotate_capture(
    bin_id: str,
    perception_result,          # PerceptionResult | None
    detections: list,           # list[Detection]
    ocr_lines: Optional[List[str]],
    status: str = "discrepancy",
) -> Path:
    """Annotate the drone camera capture for bin_id and save to annotated/.

    Args:
        bin_id: Bin identifier (e.g. "A1"). Used to find the capture PNG.
        perception_result: PerceptionResult from the pipeline (may be None).
        detections: List of Detection objects from QRCodeDetector.
        ocr_lines: Raw OCR text lines (may be None or empty).
        status: "completed" | "discrepancy" — controls overlay color.

    Returns:
        Path to the annotated image (sim/assets/annotated/<bin_id>.png).
    """
    ANNOTATED_DIR.mkdir(parents=True, exist_ok=True)

    # Load capture image
    capture_path = CAPTURES_DIR / f"{bin_id}.png"
    if not capture_path.exists():
        # Fallback to label image
        capture_path = REPO_ROOT / "sim" / "assets" / "labels" / f"{bin_id}.png"
    if not capture_path.exists():
        raise FileNotFoundError(f"No capture image found for {bin_id}")

    img = Image.open(capture_path).convert("RGBA")
    overlay = Image.new("RGBA", img.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)

    W, H = img.size
    box_color = GREEN if status == "completed" else RED
    text_color = GREEN if status == "completed" else RED

    font_main  = _load_font(max(14, H // 28))
    font_small = _load_font_small(max(12, H // 36))

    # ------------------------------------------------------------------
    # 1. Draw detection bounding boxes
    # ------------------------------------------------------------------
    bbox_used = None

    # If we have detections, draw all of them; highlight the one used by result
    if perception_result is not None and perception_result.bbox:
        bbox_used = perception_result.bbox

    # Draw all raw detections in a slightly dimmer border first
    for det in detections:
        x1, y1, x2, y2 = det.bbox
        # Dim outline for all detections
        dim_color = tuple(int(c * 0.6) for c in box_color)
        _draw_rect_with_border(draw, (x1, y1, x2, y2), dim_color, width=2)

    # Draw the result bbox prominently
    if bbox_used is not None:
        x1, y1, x2, y2 = bbox_used
        _draw_rect_with_border(draw, (x1, y1, x2, y2), box_color, width=4)

        # Confidence label near top-left of box
        conf_val = perception_result.confidence if perception_result else 0.0
        source_label = (perception_result.source or "qr").upper() if perception_result else "QR"
        conf_text = f"{source_label} {conf_val:.2f}"

        # Small backdrop for confidence text
        label_x = max(x1, 2)
        label_y = max(y1 - font_main.size - 4, 2)
        bbox_text = draw.textbbox((label_x, label_y), conf_text, font=font_main)
        pad = 3
        draw.rectangle(
            [bbox_text[0] - pad, bbox_text[1] - pad,
             bbox_text[2] + pad, bbox_text[3] + pad],
            fill=(0, 0, 0, 180),
        )
        _draw_text_with_shadow(draw, (label_x, label_y), conf_text, font_main, box_color)

    # ------------------------------------------------------------------
    # 2. Info panel (top-left corner)
    # ------------------------------------------------------------------
    panel_lines: List[str] = []

    if perception_result is not None:
        part = perception_result.part_no or "N/A"
        qty  = str(perception_result.qty) if perception_result.qty is not None else "N/A"
        src  = perception_result.source or "?"
        conf_str = f"{perception_result.confidence:.2f}"
        panel_lines += [
            f"Part No: {part}",
            f"Qty:     {qty}",
            f"Source:  {src}",
            f"Conf:    {conf_str}",
            f"Status:  {status}",
        ]
    else:
        panel_lines += ["No label detected", f"Status: {status}"]

    # OCR raw lines section
    if ocr_lines:
        panel_lines.append("── OCR lines ──")
        for line in ocr_lines[:6]:
            panel_lines.append(f"  {line}")

    line_h = font_small.size + 4
    panel_w = max(
        (draw.textlength(ln, font=font_small) for ln in panel_lines),
        default=200,
    )
    panel_w = int(panel_w) + 16
    panel_h = line_h * len(panel_lines) + 10
    px, py = 8, 8

    # Semi-transparent dark backdrop
    panel_overlay = Image.new("RGBA", img.size, (0, 0, 0, 0))
    panel_draw = ImageDraw.Draw(panel_overlay)
    panel_draw.rectangle(
        [px, py, px + panel_w, py + panel_h],
        fill=(10, 14, 22, 210),
    )
    # Colored left border strip
    panel_draw.rectangle([px, py, px + 4, py + panel_h], fill=box_color + (230,))

    # Merge panel backdrop
    overlay = Image.alpha_composite(overlay, panel_overlay)
    draw = ImageDraw.Draw(overlay)

    # Draw text lines on panel
    for i, line in enumerate(panel_lines):
        ty = py + 5 + i * line_h
        color = box_color if i == 0 or "Status" in line or "──" in line else WHITE
        _draw_text_with_shadow(draw, (px + 10, ty), line, font_small, color)

    # ------------------------------------------------------------------
    # 3. "DRONE AI CAMERA" watermark top-right
    # ------------------------------------------------------------------
    watermark = "DRONE AI CAMERA"
    wm_font = _load_font(max(13, H // 34))
    wm_w = int(draw.textlength(watermark, font=wm_font))
    wm_x = W - wm_w - 10
    wm_y = 8
    wm_overlay = Image.new("RGBA", img.size, (0, 0, 0, 0))
    wm_draw = ImageDraw.Draw(wm_overlay)
    bbox_wm = wm_draw.textbbox((wm_x, wm_y), watermark, font=wm_font)
    wm_draw.rectangle(
        [bbox_wm[0] - 4, bbox_wm[1] - 2, bbox_wm[2] + 4, bbox_wm[3] + 2],
        fill=(0, 0, 0, 160),
    )
    overlay = Image.alpha_composite(overlay, wm_overlay)
    draw = ImageDraw.Draw(overlay)
    _draw_text_with_shadow(draw, (wm_x, wm_y), watermark, wm_font, YELLOW)

    # ------------------------------------------------------------------
    # 4. Composite and save
    # ------------------------------------------------------------------
    result_img = Image.alpha_composite(img, overlay).convert("RGB")
    out_path = ANNOTATED_DIR / f"{bin_id}.png"
    result_img.save(str(out_path))
    return out_path
