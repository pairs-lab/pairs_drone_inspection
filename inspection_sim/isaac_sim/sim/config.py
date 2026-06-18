COLUMNS = ["A", "B", "C"]      # 3 cot
LEVELS = [1, 2, 3, 4, 5, 6]    # 6 tang
COLUMN_SPACING = 1.2           # m
LEVEL_HEIGHT = 0.8             # m
RACK_ORIGIN = (0.0, 0.0, 0.0)  # m
SCAN_STANDOFF = 0.75           # m, drone standoff from y=0 origin; scan_pose.y = -(BOX_D/2 + standoff)
# Drone LANDS on a home pad on the ground at the mouth of the aisle (centered in the
# aisle Y, ~3 m clear of the racks in -X). The onboard camera sits just above the
# landed drone; on inspect it takes off, flies down the aisle, scans, and lands back.
# z is the home CAMERA height; the drone mesh rests on the floor (see GROUND_DRONE_Z).
HOME_POSE = {"position": [-3.0, -0.9, 0.22], "yaw_deg": 0.0}

# Resting height of the drone mesh when landed (mesh-center z so it sits on the floor).
GROUND_DRONE_Z = 0.06

# World-space offset applied to the /World/Rack Xform when using the real
# warehouse env backdrop.  warehouse.usd is an open shell with the floor at Z=0
# and open floor area in the +X/+Y quadrant, so (0,0,0) is on open floor and
# no offset is needed.  Set to (0, 0, 0) to place our rack at the warehouse
# origin; change here and M2/M3 scripts will pick it up automatically.
RACK_WORLD_OFFSET = (0.0, 0.0, 0.0)  # (x, y, z) metres

# ---------------------------------------------------------------------------
# Cardboard box dimensions (used by rack.py AND bin_map.py for scan_pose z)
# ---------------------------------------------------------------------------
BOX_W = 0.70   # x-dimension (wide; fills most of 1.2 m column)
BOX_D = 0.50   # y-dimension (depth; front face at y = -BOX_D/2 = -0.25)
BOX_H = 0.50   # z-dimension (tall; box center z = shelf_top_z + BOX_H/2)

# ---------------------------------------------------------------------------
# GR-label dimensions (small sticker on box front face)
# ---------------------------------------------------------------------------
LABEL_W = 0.28   # half-width  → full width  0.28 m
LABEL_H = 0.20   # half-height → full height 0.20 m

# ---------------------------------------------------------------------------
# Rack structural footprint (shared with rack.py and obstacles.py)
# ---------------------------------------------------------------------------
RACK_X_MIN       = -0.5
RACK_X_MAX       =  2.9   # = 2 * COLUMN_SPACING + 0.5
RACK_Y_MIN       = -0.45
RACK_Y_MAX       =  0.45
RACK_TOTAL_HEIGHT = len(LEVELS) * LEVEL_HEIGHT   # 4.8 m

# ---------------------------------------------------------------------------
# Narrow-aisle layout constants
# ---------------------------------------------------------------------------
# Existing (primary) rack: front face at y = -(BOX_D/2) = -0.25 m (boxes face -Y).
# Second rack is placed on the -Y side of the aisle, mirrored so its front faces +Y.
# Aisle = gap between primary rack front and second rack front.
# Drone (~0.45 m wide) flies down the aisle centerline.
#
# Geometry:
#   Primary rack front (box face):          y_primary_front = -0.25 m
#   Aisle width:                            AISLE_WIDTH     = 1.30 m
#   Second rack front (box face, +Y side):  y_second_front  = y_primary_front - AISLE_WIDTH
#                                                            = -1.55 m
#   Second rack box center y (boxes face +Y, depth BOX_D=0.5):
#                                           SECOND_RACK_Y   = y_second_front - BOX_D/2
#                                                            = -1.80 m
#   Aisle centerline (between fronts):      AISLE_CENTER_Y  = y_primary_front - AISLE_WIDTH/2
#                                                            = -0.90 m

AISLE_WIDTH      = 1.30   # m — corridor width between the two rack fronts
SECOND_RACK_Y    = -1.80  # m — box-center Y of the second rack (boxes centered at this Y)
AISLE_CENTER_Y   = -0.90  # m — aisle centerline Y (midpoint between rack fronts)

# Aisle obstacle: stacked boxes / pallet sitting on the floor near column B
OBSTACLE_X       =  1.20  # m — obstacle center X (column B center)
OBSTACLE_Y       = -0.90  # m — obstacle center Y (aisle center)
OBSTACLE_Z       =  0.30  # m — obstacle half-height above floor (box 0.6 m tall, center at 0.30)
OBSTACLE_HALF_W  =  0.35  # m — half-size in X
OBSTACLE_HALF_D  =  0.35  # m — half-size in Y
OBSTACLE_HALF_H  =  0.30  # m — half-size in Z (full height 0.60 m)

# Drone safety radius (used by APF avoidance)
DRONE_RADIUS     =  0.25  # m
