#!/usr/bin/env python3
"""Generate the warehouse_rack Gazebo model (SDF) for the inspection sim."""

# --- rack parameters (metres) ----------------------------------------------
BAYS         = 3       # bays along the aisle (x)
BAY_WIDTH    = 2.70    # width of one bay
LEVELS       = 6       # storage levels (z)
LEVEL_HEIGHT = 1.20    # vertical pitch between decks
DEPTH        = 1.00    # rack depth (y); front face at -DEPTH/2
UPRIGHT      = 0.10    # square upright cross-section
DECK_T       = 0.08    # deck thickness
BEAM_T       = 0.10    # aisle-facing beam thickness
PALLET       = (1.00, 0.80, 0.90)   # pallet (x, y, z)
PALLET_LEVELS = (0, 1, 2, 3, 4, 5)  # pallets on all 6 levels (18 Bins)
PALLET_COLS   = (-2.70, 0.0, 2.70)  # pallet x positions

LENGTH = BAYS * BAY_WIDTH
X_MIN, X_MAX = -LENGTH / 2.0, LENGTH / 2.0
FRONT_Y = -DEPTH / 2.0
BACK_Y = DEPTH / 2.0
DECK_Z = [0.5 + i * LEVEL_HEIGHT for i in range(LEVELS)]

def box_link(name, size, pose, color):
    sx, sy, sz = size
    return f"""    <link name='{name}'>
      <pose>{pose[0]:.4f} {pose[1]:.4f} {pose[2]:.4f} 0 0 0</pose>
      <collision name='{name}_col'>
        <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
      </collision>
      <visual name='{name}_vis'>
        <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        <material><script>
          <uri>file://media/materials/scripts/gazebo.material</uri>
          <name>Gazebo/{color}</name>
        </script></material>
      </visual>
    </link>"""

def tag_link(name, pose):
    return f"""    <link name='{name}'>
      <pose>{pose[0]:.4f} {pose[1]:.4f} {pose[2]:.4f} 0 0 0</pose>
      <visual name='{name}_vis'>
        <geometry><box><size>0.30 0.001 0.30</size></box></geometry>
        <material>
          <script>
            <uri>model://apriltag_marker/materials/scripts</uri>
            <uri>model://apriltag_marker/materials/textures</uri>
            <name>Apriltag_marker</name>
          </script>
        </material>
      </visual>
    </link>"""

def main():
    links = []

    # uprights
    for i in range(BAYS + 1):
        x = X_MIN + i * BAY_WIDTH
        for y, tag in ((FRONT_Y, "f"), (BACK_Y, "b")):
            links.append(box_link(
                f"upright_{i}_{tag}", (UPRIGHT, UPRIGHT, DECK_Z[-1] + LEVEL_HEIGHT),
                (x, y, (DECK_Z[-1] + LEVEL_HEIGHT) / 2.0), "Blue"))

    # Decks & Front Beams
    for i, z in enumerate(DECK_Z):
        links.append(box_link(f"deck_{i}", (LENGTH, DEPTH, DECK_T), (0.0, 0.0, z), "Orange"))
        links.append(box_link(f"frontbeam_{i}", (LENGTH, BEAM_T, BEAM_T), (0.0, FRONT_Y, z), "Orange"))
        
        # Tags
        for b in range(BAYS):
            bay_center_x = X_MIN + (b + 0.5) * BAY_WIDTH
            tag_y = -0.40 - 0.001
            tag_z = z + (LEVEL_HEIGHT / 2.0)
            links.append(tag_link(f"tag_beam_level_{i}_bay_{b}", (bay_center_x, tag_y, tag_z)))

    # Pallets
    for i in PALLET_LEVELS:
        z = DECK_Z[i] + DECK_T / 2.0 + PALLET[2] / 2.0
        for j, x in enumerate(PALLET_COLS):
            links.append(box_link(f"pallet_{i}_{j}", PALLET, (x, 0.0, z), "Grey"))

    body = "\n".join(links)
    print(f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='warehouse_rack'>
    <static>1</static>
{body}
  </model>
</sdf>""")

if __name__ == "__main__":
    main()