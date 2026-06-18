import json
import qrcode
from PIL import Image, ImageDraw, ImageFont

def encode_payload(part_no, qty):
    return json.dumps({"part_no": part_no, "qty": int(qty)})

def decode_payload(s):
    d = json.loads(s)
    return d["part_no"], int(d["qty"])

def make_label_image(part_no, qty, out_path, size=512):
    canvas = Image.new("RGB", (size, size), "white")
    qr = qrcode.make(encode_payload(part_no, qty)).convert("RGB").resize((size // 2, size // 2), Image.NEAREST)
    canvas.paste(qr, (size // 4, 0))
    draw = ImageDraw.Draw(canvas)
    try:
        font = ImageFont.truetype("DejaVuSans-Bold.ttf", 40)
    except OSError:
        font = ImageFont.load_default()
    draw.text((20, size // 2 + 20), "GR LABEL", fill="black", font=font)
    draw.text((20, size // 2 + 80), f"Part No: {part_no}", fill="black", font=font)
    draw.text((20, size // 2 + 140), f"Qty: {qty}", fill="black", font=font)
    canvas.save(out_path)
    return out_path

def generate_all_labels(out_dir="sim/assets/labels"):
    import os
    from sim.bin_map import load_bin_map
    os.makedirs(out_dir, exist_ok=True)
    paths = {}
    for bid, b in load_bin_map().items():
        paths[bid] = make_label_image(b["part_no"], b["qty"], f"{out_dir}/{bid}.png")
    return paths

if __name__ == "__main__":
    print("labels:", len(generate_all_labels()))
