import yaml
from sim.config import (COLUMNS, LEVELS, COLUMN_SPACING, LEVEL_HEIGHT,
                        RACK_ORIGIN, SCAN_STANDOFF, BOX_D, BOX_H)

BIN_IDS = [f"{c}{l}" for c in COLUMNS for l in LEVELS]
REQUIRED = {"pallet_pose", "scan_pose", "part_no", "qty"}

def _pallet_position(col_idx, level):
    ox, oy, oz = RACK_ORIGIN
    return [ox + col_idx * COLUMN_SPACING, oy, oz + (level - 1) * LEVEL_HEIGHT]

def generate_bin_map():
    m = {}
    for ci, col in enumerate(COLUMNS):
        for level in LEVELS:
            bid = f"{col}{level}"
            px, py, pz = _pallet_position(ci, level)
            # pz = shelf top = (level-1)*LEVEL_HEIGHT
            # Box center z = pz + BOX_H/2; label is at box center z (mid-height of box)
            box_center_z = pz + BOX_H / 2
            # Box front face is at y = py - BOX_D/2 = -0.25
            # Camera at y = -(BOX_D/2 + SCAN_STANDOFF) from origin = -0.25 - 0.75 = -1.0
            scan_y = py - BOX_D / 2 - SCAN_STANDOFF
            m[bid] = {
                "pallet_pose": {"position": [px, py, pz], "yaw_deg": 0.0},
                "scan_pose": {"position": [px, scan_y, box_center_z], "yaw_deg": 90.0},
                "part_no": f"PN-{col}{level:02d}",
                "qty": 10 + ci * 6 + level,
            }
    return m

def validate_bin_map(m):
    if len(m) != len(BIN_IDS) or set(m) != set(BIN_IDS):
        raise ValueError(f"bin_map must have exactly {len(BIN_IDS)} bins: {BIN_IDS}")
    for bid, b in m.items():
        if not REQUIRED <= b.keys():
            raise ValueError(f"bin {bid} missing fields {REQUIRED - b.keys()}")
        if len(b["scan_pose"]["position"]) != 3:
            raise ValueError(f"bin {bid} scan_pose.position must be xyz")
    return True

def write_bin_map(path="sim/bin_map.yaml"):
    m = generate_bin_map(); validate_bin_map(m)
    with open(path, "w") as f:
        yaml.safe_dump(m, f, sort_keys=True)
    return path

def load_bin_map(path="sim/bin_map.yaml"):
    with open(path) as f:
        m = yaml.safe_load(f)
    validate_bin_map(m)
    return m

if __name__ == "__main__":
    print("wrote", write_bin_map())
