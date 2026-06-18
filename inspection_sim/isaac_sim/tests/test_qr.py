import os
from perception.qr import decode_qr_in_image, decode_qr_crop

LABEL = "sim/assets/labels/A1.png"


def test_decode_full_label():
    assert os.path.exists(LABEL), "run `conda run -n isaac6 python -m sim.gr_label` first"
    res = decode_qr_in_image(LABEL)
    assert res is not None
    assert res[0] == "PN-A01" and res[1] == 11


def test_decode_crop_bbox():
    # whole image as bbox still decodes
    from PIL import Image
    import numpy as np
    arr = np.array(Image.open(LABEL).convert("RGB"))
    h, w = arr.shape[:2]
    res = decode_qr_crop(arr, (0, 0, w, h))
    assert res == ("PN-A01", 11)
