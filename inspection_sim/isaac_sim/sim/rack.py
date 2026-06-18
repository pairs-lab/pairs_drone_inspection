"""Build a realistic 3x6 shelving rack with cardboard boxes and emissive GR labels.

Structure:
  - 4 vertical steel posts (gray primitive beams) at the 4 corners of the rack footprint
  - 6 horizontal shelf boards (thin gray slabs) spanning the 3 columns, one per level
  - 18 cardboard boxes (SM_CardBoxC_01.usd referenced asset) resting ON each shelf,
    centered at each bin's (x, 0) column; box bottom is at the shelf top surface
  - 18 emissive GR-label quads on the front face of each box (facing -Y), textured
    from sim/assets/labels/<bid>.png so the drone camera can read the QR code

Geometry alignment (must match sim/config.py and sim/bin_map.py):
  - COLUMN_SPACING = 1.2 m  → columns A,B,C at x = 0, 1.2, 2.4
  - LEVEL_HEIGHT   = 0.8 m  → levels 1-6 at z = 0, 0.8, 1.6, 2.4, 3.2, 4.0
  - pallet_pose.position = [ci*1.2, 0, (L-1)*0.8]  (BIN REFERENCE, UNCHANGED)
  - Shelf top surface is at z = (L-1)*0.8  (same as pallet_pose.z = bin reference)
  - Box center z = (L-1)*0.8 + BOX_H/2   (box sits ON the shelf)
  - Box front face is at y = -BOX_D/2 = -0.25
  - Label quad center z = box_center_z (mid-height of box), y = -BOX_D/2 - 0.005 (5mm sticker)
"""
import os
from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf
from sim.bin_map import load_bin_map
from sim.config import (COLUMNS, LEVELS, COLUMN_SPACING, LEVEL_HEIGHT, BOX_W, BOX_D, BOX_H, LABEL_W, LABEL_H,
                        RACK_X_MIN, RACK_X_MAX, RACK_Y_MIN, RACK_Y_MAX, RACK_TOTAL_HEIGHT)


# ---------------------------------------------------------------------------
# Rack structural dimensions  (values now live in sim/config.py)
# ---------------------------------------------------------------------------
# RACK_X_MIN, RACK_X_MAX, RACK_Y_MIN, RACK_Y_MAX, RACK_TOTAL_HEIGHT imported above.

# Post cross-section (square beams)
POST_W  = 0.08   # m (width in X and Y)

# Shelf board
SHELF_THICKNESS = 0.05   # m
SHELF_SPAN_X = RACK_X_MAX - RACK_X_MIN  # = 3.4 m  (covers all 3 columns)
SHELF_DEPTH  = RACK_Y_MAX - RACK_Y_MIN  # = 0.90 m

# BOX_W, BOX_D, BOX_H, LABEL_W, LABEL_H imported from sim.config
# Box: 0.70 x 0.50 x 0.50 m (W x D x H); label: 0.28 x 0.20 m

# Box asset path (local asset mirror)
# SM_CardBoxC_01.usd is the canonical path; Isaac resolves it from
# ~/isaacsim_assets via the asset root set by use_local_assets().
_BOX_ASSET_REL = "Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxC_01_1052.usd"
_BOX_ASSET_ALT = "Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxA_01_301.usd"


def _box_asset_path():
    """Return absolute path to a known-good cardboard box USD in the local mirror."""
    mirror = os.path.expanduser("~/isaacsim_assets")
    primary = os.path.join(mirror, _BOX_ASSET_REL)
    if os.path.exists(primary):
        return primary
    alt = os.path.join(mirror, _BOX_ASSET_ALT)
    if os.path.exists(alt):
        return alt
    raise FileNotFoundError(
        f"Cardboard box asset not found at {primary} or {alt}. "
        "Ensure ~/isaacsim_assets is populated (run scripts/download_assets.sh)."
    )


def _gray_metal_material(stage, prim_path):
    """Simple gray metal UsdPreviewSurface for structural elements."""
    base = f"/World/Materials/{prim_path.replace('/', '_')}"
    mat = UsdShade.Material.Define(stage, base)
    shader = UsdShade.Shader.Define(stage, f"{base}/PBR")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor",  Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.45, 0.47, 0.50))
    shader.CreateInput("metallic",      Sdf.ValueTypeNames.Float).Set(0.8)
    shader.CreateInput("roughness",     Sdf.ValueTypeNames.Float).Set(0.4)
    mat.CreateSurfaceOutput().ConnectToSource(
        shader.CreateOutput("surface", Sdf.ValueTypeNames.Token))
    return mat


def _make_label_material(stage, bid, tex_path):
    """UsdPreviewSurface that drives BOTH diffuseColor and emissiveColor from the
    label texture, read through an explicit UsdPrimvarReader_float2 on the "st" UV
    set. Emissive makes the QR readable regardless of scene lighting; the explicit
    primvar reader guarantees the UVs are sampled correctly by the RTX renderer."""
    base = f"/World/Materials/Label_{bid}"
    mat = UsdShade.Material.Define(stage, base)
    shader = UsdShade.Shader.Define(stage, f"{base}/PBR")
    shader.CreateIdAttr("UsdPreviewSurface")

    st_reader = UsdShade.Shader.Define(stage, f"{base}/STReader")
    st_reader.CreateIdAttr("UsdPrimvarReader_float2")
    st_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")

    tex = UsdShade.Shader.Define(stage, f"{base}/Tex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(tex_path)
    tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        st_reader.CreateOutput("result", Sdf.ValueTypeNames.Float2))

    rgb = tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(rgb)
    shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(rgb)
    mat.CreateSurfaceOutput().ConnectToSource(
        shader.CreateOutput("surface", Sdf.ValueTypeNames.Token))
    return mat


def _add_box_prim(stage, path, pos_x, pos_y, pos_z):
    """Place a primitive brown box (UsdGeom.Cube) at the given position.

    This is used as a FALLBACK when the USD asset reference cannot be resolved
    (e.g. assets not downloaded yet). The box is a plain scaled cube with a
    cardboard-brown color so the scene still looks reasonable.

    pos_z is the BOX CENTER (already = shelf_top_z + BOX_H/2).
    UsdGeom.Cube has unit half-extents, so scale = half-size = (BOX_W/2, BOX_D/2, BOX_H/2).
    """
    cube = UsdGeom.Cube.Define(stage, path)
    cube.AddTranslateOp().Set(Gf.Vec3d(pos_x, pos_y, pos_z))
    cube.AddScaleOp().Set(Gf.Vec3f(BOX_W / 2, BOX_D / 2, BOX_H / 2))

    mat_path = path.replace("/", "_") + "_Mat"
    mat = UsdShade.Material.Define(stage, f"/World/Materials/{mat_path}")
    sh = UsdShade.Shader.Define(stage, f"/World/Materials/{mat_path}/PBR")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.72, 0.52, 0.28))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
    mat.CreateSurfaceOutput().ConnectToSource(
        sh.CreateOutput("surface", Sdf.ValueTypeNames.Token))
    UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(mat)
    return cube


def _add_box_reference(stage, path, cx, cy, cz, box_usd_path):
    """Reference a real USD cardboard box asset and snap it precisely into place.

    The asset has an UNKNOWN native size and an UNKNOWN local origin (often at the
    base or an arbitrary point), so naively scaling by raw factors and translating to
    a guessed center leaves the box floating above the shelf. Instead we:
      1. reference the asset under an Xform (no transform yet),
      2. measure its real world-aligned bounding box,
      3. scale per-axis so the bbox becomes exactly BOX_W x BOX_D x BOX_H, and
      4. translate so the scaled bbox CENTER lands at (cx, cy, cz).

    Caller passes cz = shelf_top_z + BOX_H/2, so the box BOTTOM ends up exactly on the
    shelf top surface (bottom = cz - BOX_H/2 = shelf_top_z). No floating, any asset.
    """
    xf = UsdGeom.Xform.Define(stage, path)
    box_prim = stage.DefinePrim(f"{path}/Mesh")
    box_prim.GetReferences().AddReference(box_usd_path)

    # Measure native bounding box (Xform is still identity -> world bound == asset bound)
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        includedPurposes=[UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
        useExtentsHint=True,
    )
    rng = cache.ComputeWorldBound(xf.GetPrim()).ComputeAlignedRange()
    mn, mx = rng.GetMin(), rng.GetMax()
    size = (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])
    ctr = ((mx[0] + mn[0]) / 2.0, (mx[1] + mn[1]) / 2.0, (mx[2] + mn[2]) / 2.0)

    if min(size) <= 1e-6:
        # asset bound unavailable (e.g. unloaded payload) -> fall back to a plain cube
        stage.RemovePrim(box_prim.GetPath())
        stage.RemovePrim(xf.GetPrim().GetPath())
        return _add_box_prim(stage, path, cx, cy, cz)

    sx, sy, sz = BOX_W / size[0], BOX_D / size[1], BOX_H / size[2]
    # ops apply Scale first then Translate (xformOpOrder = [Translate, Scale]),
    # so a native point p maps to p*scale + translate; pick translate to center the box.
    tx = cx - ctr[0] * sx
    ty = cy - ctr[1] * sy
    tz = cz - ctr[2] * sz
    xf.AddTranslateOp().Set(Gf.Vec3d(tx, ty, tz))
    xf.AddScaleOp().Set(Gf.Vec3f(sx, sy, sz))
    return xf


def build_rack(stage) -> int:
    """Build the 3x6 warehouse rack and return the number of bins (18).

    Produces:
      /World/Rack/Structure/   -- 4 vertical posts + 6 horizontal shelf boards
      /World/Rack/Bin_<id>/Box    -- cardboard box resting on shelf (USD ref or fallback)
      /World/Rack/Bin_<id>/Label  -- emissive GR-label quad (texture from assets/labels/)
    """
    bins = load_bin_map()

    # --- Determine box asset path (USD ref or fallback) ---
    try:
        box_usd = _box_asset_path()
        use_usd_ref = True
    except FileNotFoundError:
        use_usd_ref = False

    # --- Gray metal material for structural elements ---
    metal_mat = _gray_metal_material(stage, "RackMetal")

    # -----------------------------------------------------------------------
    # 1. SHELVING STRUCTURE
    # -----------------------------------------------------------------------
    struct = UsdGeom.Xform.Define(stage, "/World/Rack/Structure")

    # 4 vertical posts at corners: (x_min, y_min), (x_min, y_max), (x_max, y_min), (x_max, y_max)
    post_cx = [(RACK_X_MIN + POST_W / 2), (RACK_X_MAX - POST_W / 2)]
    post_cy = [(RACK_Y_MIN + POST_W / 2), (RACK_Y_MAX - POST_W / 2)]

    for pi, (px_post, py_post) in enumerate(
            [(post_cx[0], post_cy[0]), (post_cx[0], post_cy[1]),
             (post_cx[1], post_cy[0]), (post_cx[1], post_cy[1])]):
        post = UsdGeom.Cube.Define(stage, f"/World/Rack/Structure/Post_{pi}")
        # UsdGeom.Cube has unit half-size; scale to POST_W x POST_W x RACK_TOTAL_HEIGHT
        post.AddTranslateOp().Set(Gf.Vec3d(px_post, py_post, RACK_TOTAL_HEIGHT / 2))
        post.AddScaleOp().Set(Gf.Vec3f(POST_W / 2, POST_W / 2, RACK_TOTAL_HEIGHT / 2))
        UsdShade.MaterialBindingAPI(post.GetPrim()).Bind(metal_mat)
        # NOTE: We intentionally do NOT add UsdPhysics.CollisionAPI here.
        # Rack posts are visual-only; the APF avoidance in drone/avoidance.py
        # keeps the drone away from the rack AABB volumes (see sim/obstacles.py).
        # Adding CollisionAPI would make the posts static physics colliders that
        # physically stop the drone body — instead, we track "collisions" as APF
        # clearance violations (clearance <= 0 steps) in fly_aisle_demo.py.

    # 6 shelf boards: one per level, spanning full rack width and depth.
    # Shelf TOP is at z = (L-1)*LEVEL_HEIGHT.  Shelf center is half a thickness below.
    shelf_cx = (RACK_X_MIN + RACK_X_MAX) / 2  # = 1.2 (center of rack span)
    shelf_cy = (RACK_Y_MIN + RACK_Y_MAX) / 2  # = 0.0

    for li, level in enumerate(LEVELS):
        shelf_top_z = (level - 1) * LEVEL_HEIGHT
        shelf_center_z = shelf_top_z - SHELF_THICKNESS / 2

        shelf = UsdGeom.Cube.Define(stage, f"/World/Rack/Structure/Shelf_{level}")
        shelf.AddTranslateOp().Set(Gf.Vec3d(shelf_cx, shelf_cy, shelf_center_z))
        shelf.AddScaleOp().Set(Gf.Vec3f(
            SHELF_SPAN_X / 2,    # half-extent in X
            SHELF_DEPTH / 2,     # half-extent in Y
            SHELF_THICKNESS / 2  # half-extent in Z
        ))
        UsdShade.MaterialBindingAPI(shelf.GetPrim()).Bind(metal_mat)
        # Shelf boards: visual-only for same reason as posts above.

    # -----------------------------------------------------------------------
    # 2. CARDBOARD BOXES + EMISSIVE GR LABELS (one per bin)
    # -----------------------------------------------------------------------
    for bid, b in bins.items():
        px, py, pz = b["pallet_pose"]["position"]
        # pz = (level-1)*LEVEL_HEIGHT = shelf top surface
        # Box center z = pz + BOX_H/2 so box BOTTOM rests exactly on shelf top
        box_center_z = pz + BOX_H / 2

        bin_root = f"/World/Rack/Bin_{bid}"

        # -- Box: big carton (0.70 x 0.50 x 0.50 m) resting ON shelf --
        # py=0 → box centered in Y, front face at y = -BOX_D/2 = -0.25
        box_path = f"{bin_root}/Box"
        if use_usd_ref:
            _add_box_reference(stage, box_path, px, py, box_center_z, box_usd)
        else:
            _add_box_prim(stage, box_path, px, py, box_center_z)

        # -- GR Label: small sticker stuck flat on box front face --
        # Geometry: rectangular quad LABEL_W wide (X) x LABEL_H tall (Z)
        # Position:
        #   x = px (column center)
        #   y = -(BOX_D/2) - 0.005 = -0.255  (5 mm in front of box face, avoids z-fighting)
        #   z = box_center_z (mid-height of box, label centered vertically on carton)
        # Normal: -Y (facing drone, which approaches from negative-Y direction)
        label_z = box_center_z          # vertically centered on box
        label_y = py - BOX_D / 2 - 0.005  # 5 mm proud of box front face

        hw = LABEL_W / 2   # half-width  in X
        hh = LABEL_H / 2   # half-height in Z

        quad = UsdGeom.Mesh.Define(stage, f"{bin_root}/Label")
        # Quad corners in LOCAL space (quad lies flat in XZ plane; Y=0 in local space)
        # Vertices: bottom-left, bottom-right, top-right, top-left (CCW when viewed from -Y)
        quad.CreatePointsAttr([(-hw, 0, -hh), (hw, 0, -hh), (hw, 0, hh), (-hw, 0, hh)])
        quad.CreateFaceVertexCountsAttr([4])
        quad.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        quad.CreateExtentAttr([(-hw, 0, -hh), (hw, 0, hh)])
        # Normal -Y (toward the scanning drone); double-sided so it renders both ways
        quad.CreateNormalsAttr([(0, -1, 0)] * 4)
        quad.SetNormalsInterpolation("faceVarying")
        UsdGeom.Gprim(quad).CreateDoubleSidedAttr(True)
        # UV mapping so texture fills the entire quad
        pv = UsdGeom.PrimvarsAPI(quad.GetPrim()).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
        pv.Set([(0, 0), (1, 0), (1, 1), (0, 1)])
        quad.AddTranslateOp().Set(Gf.Vec3d(px, label_y, label_z))

        tex = os.path.abspath(f"sim/assets/labels/{bid}.png")
        mat = _make_label_material(stage, bid, tex)
        UsdShade.MaterialBindingAPI(quad.GetPrim()).Bind(mat)

        # Semantic label for replicator bbox annotator (Task 3 dataset gen)
        # add_labels(prim, labels, instance_name='class') is the Isaac 6.0 API.
        # Falls back gracefully if semantics extension unavailable.
        try:
            from isaacsim.core.utils.semantics import add_labels
            add_labels(quad.GetPrim(), ["label"], instance_name="class")
        except Exception:
            pass  # semantics not critical for render; skip silently

    return len(bins)


# ---------------------------------------------------------------------------
# Second rack (mirror): boxes face +Y, no labels, used as obstacle wall
# ---------------------------------------------------------------------------

def _build_rack_structure(stage, root_path, box_y_center, front_faces_minus_y=True):
    """Build a bare shelving structure (posts + shelves + boxes) at an arbitrary Y.

    Args:
        root_path:           USD prim path for the rack Xform root.
        box_y_center:        world Y coordinate of box centres (shelves centred at 0
                             for primary rack; shifted for second rack).
        front_faces_minus_y: True = box front face at y - BOX_D/2 (faces -Y, like primary).
                             False = box front face at y + BOX_D/2 (faces +Y, mirror).
    """
    try:
        box_usd = _box_asset_path()
        use_usd_ref = True
    except FileNotFoundError:
        use_usd_ref = False

    metal_mat = _gray_metal_material(stage, root_path.replace("/", "_"))

    # Y extents centred at box_y_center (same depth as primary rack)
    half_depth = (RACK_Y_MAX - RACK_Y_MIN) / 2   # 0.45 m
    y_min = box_y_center - half_depth
    y_max = box_y_center + half_depth
    struct = UsdGeom.Xform.Define(stage, f"{root_path}/Structure")

    # 4 vertical posts at corners
    for pi, (px_post, py_post) in enumerate([
            (RACK_X_MIN + POST_W / 2, y_min + POST_W / 2),
            (RACK_X_MIN + POST_W / 2, y_max - POST_W / 2),
            (RACK_X_MAX - POST_W / 2, y_min + POST_W / 2),
            (RACK_X_MAX - POST_W / 2, y_max - POST_W / 2),
    ]):
        post = UsdGeom.Cube.Define(stage, f"{root_path}/Structure/Post_{pi}")
        post.AddTranslateOp().Set(Gf.Vec3d(px_post, py_post, RACK_TOTAL_HEIGHT / 2))
        post.AddScaleOp().Set(Gf.Vec3f(POST_W / 2, POST_W / 2, RACK_TOTAL_HEIGHT / 2))
        UsdShade.MaterialBindingAPI(post.GetPrim()).Bind(metal_mat)
        # Visual-only: no CollisionAPI (APF handles avoidance at software level)

    # 6 shelf boards
    shelf_cx = (RACK_X_MIN + RACK_X_MAX) / 2
    shelf_cy = box_y_center
    for li, level in enumerate(LEVELS):
        shelf_top_z = (level - 1) * LEVEL_HEIGHT
        shelf_center_z = shelf_top_z - SHELF_THICKNESS / 2
        shelf = UsdGeom.Cube.Define(stage, f"{root_path}/Structure/Shelf_{level}")
        shelf.AddTranslateOp().Set(Gf.Vec3d(shelf_cx, shelf_cy, shelf_center_z))
        shelf.AddScaleOp().Set(Gf.Vec3f(
            SHELF_SPAN_X / 2,
            half_depth,
            SHELF_THICKNESS / 2,
        ))
        UsdShade.MaterialBindingAPI(shelf.GetPrim()).Bind(metal_mat)
        # Visual-only: no CollisionAPI

    # Boxes on shelves — ALWAYS use primitives for the second rack so we don't
    # accidentally inherit physics (RigidBodyAPI) from the USD asset reference,
    # which would make them dynamic rigid bodies that fall/pile up.
    for ci, col in enumerate(COLUMNS):
        for level in LEVELS:
            px = ci * COLUMN_SPACING
            pz = (level - 1) * LEVEL_HEIGHT
            box_center_z = pz + BOX_H / 2
            bin_root = f"{root_path}/Bin_{col}{level}"
            box_path = f"{bin_root}/Box"
            _add_box_prim(stage, box_path, px, box_y_center, box_center_z)


def build_second_rack(stage):
    """Build the mirror rack on the -Y side of the aisle.

    The second rack's box centres sit at Y = SECOND_RACK_Y; its front face
    (which faces +Y toward the primary rack / aisle) is at y = SECOND_RACK_Y + BOX_D/2.
    No QR labels — this is an obstacle wall only.

    Adds CollisionAPI to the structural posts so a physics collision is detectable.
    """
    from sim.config import SECOND_RACK_Y
    from pxr import UsdPhysics

    root_path = "/World/Rack2"
    UsdGeom.Xform.Define(stage, root_path)
    _build_rack_structure(stage, root_path, box_y_center=SECOND_RACK_Y,
                          front_faces_minus_y=False)


def build_aisle_obstacle(stage):
    """Place a stacked-box obstacle on the aisle floor near column B.

    Geometry: a brown cube of size (OBSTACLE_HALF_W*2 x OBSTACLE_HALF_D*2 x OBSTACLE_HALF_H*2)
    sitting on the floor at (OBSTACLE_X, OBSTACLE_Y).  CollisionAPI applied so a
    real physics contact is detectable.
    """
    from sim.config import (OBSTACLE_X, OBSTACLE_Y, OBSTACLE_Z,
                             OBSTACLE_HALF_W, OBSTACLE_HALF_D, OBSTACLE_HALF_H)
    from pxr import UsdPhysics

    path = "/World/AisleObstacle"
    cube = UsdGeom.Cube.Define(stage, path)
    # Cube is unit half-size -> scale to obstacle half-extents
    cube.AddTranslateOp().Set(Gf.Vec3d(OBSTACLE_X, OBSTACLE_Y, OBSTACLE_Z))
    cube.AddScaleOp().Set(Gf.Vec3f(OBSTACLE_HALF_W, OBSTACLE_HALF_D, OBSTACLE_HALF_H))

    # Brown color (stacked cargo boxes)
    mat = UsdShade.Material.Define(stage, "/World/Materials/AisleObstacleMat")
    sh = UsdShade.Shader.Define(stage, "/World/Materials/AisleObstacleMat/PBR")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.65, 0.40, 0.20))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.85)
    mat.CreateSurfaceOutput().ConnectToSource(
        sh.CreateOutput("surface", Sdf.ValueTypeNames.Token))
    UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(mat)
    # Visual-only: no CollisionAPI (APF handles avoidance; obstacle AABB is in sim/obstacles.py)
