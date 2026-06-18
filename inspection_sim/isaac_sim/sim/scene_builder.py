"""M1 entrypoint: build the full warehouse POC scene and save it to USD.

Run with the Isaac Sim 6.0 BINARY via the wrapper (from repo root):
    scripts/run_isaac.sh -m sim.scene_builder            # headless, real warehouse
    scripts/run_isaac.sh -m sim.scene_builder -- --gui   # open a window to inspect
    scripts/run_isaac.sh -m sim.scene_builder -- --no-warehouse  # primitive ground

Builds: warehouse env (or primitive ground) + lighting, 3x6 rack = 18 bins each
with a pallet and an emissive GR-label quad, and a drone parked at HOME_POSE.
Saves sim/warehouse_poc.usd.

--warehouse (default): loads ~/isaacsim_assets/Isaac/Environments/Simple_Warehouse/
    warehouse.usd as the backdrop floor/walls/lighting.
--no-warehouse / --primitive: falls back to the original primitive ground plane
    (no asset dependency; useful for offline/CI).
"""
import argparse


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gui", action="store_true", help="open Isaac Sim window")
    # Warehouse backdrop: ON by default; --no-warehouse or --primitive disables it
    wh_group = ap.add_mutually_exclusive_group()
    wh_group.add_argument("--warehouse", dest="warehouse", action="store_true",
                          default=True, help="use real warehouse USD backdrop (default)")
    wh_group.add_argument("--no-warehouse", "--primitive", dest="warehouse",
                          action="store_false",
                          help="use primitive ground plane (no asset dependency)")
    args = ap.parse_args()

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": not args.gui, "active_gpu": 0, "physics_gpu": 0})

    import omni.usd
    from pxr import UsdGeom, Gf
    from sim.rack import build_rack, build_second_rack, build_aisle_obstacle
    from sim.drone_asset import spawn_drone
    from sim.config import HOME_POSE, RACK_WORLD_OFFSET

    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, "Z")

    if args.warehouse:
        from sim.warehouse import use_local_assets, build_warehouse_env
        use_local_assets()
        UsdGeom.Xform.Define(stage, "/World")
        build_warehouse_env(stage, kind="warehouse")
        print("SCENE using real warehouse.usd backdrop")
    else:
        from sim.warehouse import build_warehouse
        build_warehouse(stage)
        print("SCENE using primitive ground (--no-warehouse)")

    # Place rack under a parent Xform so RACK_WORLD_OFFSET can shift it later
    rack_root = UsdGeom.Xform.Define(stage, "/World/Rack")
    ox, oy, oz = RACK_WORLD_OFFSET
    if ox != 0.0 or oy != 0.0 or oz != 0.0:
        rack_root.AddTranslateOp().Set(Gf.Vec3d(ox, oy, oz))

    n = build_rack(stage)

    # Narrow-aisle additions: second (mirror) rack + floor obstacle
    build_second_rack(stage)
    build_aisle_obstacle(stage)

    # Drone home position shifts with the rack offset
    hp = HOME_POSE["position"]
    drone_pos = (hp[0] + ox, hp[1] + oy, hp[2] + oz)
    spawn_drone(stage, drone_pos)

    out = "sim/warehouse_poc.usd"
    omni.usd.get_context().save_as_stage(out, None)
    print(f"BUILT bins={n} saved={out}")

    if args.gui:
        while app.is_running():
            app.update()
    app.close()


if __name__ == "__main__":
    main()
