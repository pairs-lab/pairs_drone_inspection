"""QR detector using a PRE-TRAINED off-the-shelf YOLOv8 QR model (qrdet).

No custom training / synthetic dataset needed — `qrdet` ships a YOLOv8 model
trained to detect QR codes in real photos, and downloads its weights on first use.
Returns Detection objects (see perception.types).
"""
from perception.types import Detection


class QRCodeDetector:
    """Detect QR-code regions in an image with the pre-trained qrdet YOLOv8 model."""

    def __init__(self, model_size="n", conf=0.3):
        """Args:
            model_size: qrdet model size 'n'|'s'|'m'|'l' (n = fastest).
            conf: confidence threshold (0-1); detections below are dropped.
        """
        from qrdet import QRDetector
        self.model = QRDetector(model_size=model_size)
        self.conf = conf

    def detect(self, image_rgb):
        """Run QR detection on image_rgb.

        Args:
            image_rgb: HxWx3 numpy ndarray (uint8, RGB).
        Returns:
            list of Detection(cls='qr', bbox=(x1,y1,x2,y2), conf=float),
            sorted by confidence descending.
        """
        results = self.model.detect(image=image_rgb, is_bgr=False)
        out = []
        for r in results:
            c = float(r["confidence"])
            if c < self.conf:
                continue
            x1, y1, x2, y2 = [int(v) for v in r["bbox_xyxy"]]
            out.append(Detection(cls="qr", bbox=(x1, y1, x2, y2), conf=c))
        out.sort(key=lambda d: d.conf, reverse=True)
        return out


# Backwards-compatible alias (older code referenced LabelDetector)
LabelDetector = QRCodeDetector
