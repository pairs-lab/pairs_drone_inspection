from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import omni.usd  # noqa
from sim.warehouse import build_warehouse
from sim.rack import build_rack

stage = omni.usd.get_context().get_stage()
build_warehouse(stage)
n = build_rack(stage)
prims = len(list(stage.Traverse()))
print(f"SCENE_OK bins={n} prims={prims}")
assert n == 18, "expected 18 bins"
app.close()
