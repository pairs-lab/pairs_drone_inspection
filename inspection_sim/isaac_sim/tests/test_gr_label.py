from pathlib import Path
from pyzbar.pyzbar import decode
from PIL import Image
from sim.gr_label import make_label_image, encode_payload, decode_payload

def test_payload_roundtrip():
    p = encode_payload("PN-A01", 11)
    pn, qty = decode_payload(p)
    assert pn == "PN-A01" and qty == 11

def test_label_qr_decodes(tmp_path):
    out = tmp_path / "A1.png"
    make_label_image("PN-A01", 11, str(out))
    assert out.exists()
    decoded = decode(Image.open(out))
    assert decoded, "no QR found in rendered label"
    pn, qty = decode_payload(decoded[0].data.decode())
    assert pn == "PN-A01" and qty == 11
