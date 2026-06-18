from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class Detection:
    cls: str                              # "qr" | "label"
    bbox: Tuple[int, int, int, int]       # x1, y1, x2, y2
    conf: float


@dataclass(frozen=True)
class PerceptionResult:
    part_no: Optional[str]
    qty: Optional[int]
    confidence: float
    bbox: Tuple[int, int, int, int]
    source: str                           # "qr" | "ocr" | "qr+ocr"
