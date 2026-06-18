"""Workaround for a known pip Isaac Sim 6.0.0 packaging bug.

The default experience app `isaacsim.exp.base.kit` declares several schema /
animation / metropolis extensions that are NOT shipped in the pip wheels and are
NOT in the public extension registry (e.g. `isaacsim.anim.robot.schema`). The
dependency solver then aborts and SimulationApp fails to launch.

Our warehouse POC (primitives + camera + replicator) does not need any of those
extensions, so we mark them `optional = true`. Optional deps that cannot be
resolved are skipped instead of aborting the solver. Essential extensions we DO
need (omni.replicator.core, omni.syntheticdata, omni.graph.*, omni.hydra.*) are
left untouched so they still download from the registry on first launch.

Idempotent: keeps a one-time .bak and is safe to re-run after reinstall.
Ref: github.com/isaac-sim/IsaacLab/issues/5435 and NVIDIA dev forums.
"""
import glob
import os
import shutil
import sys

# Missing-and-unnecessary extensions to mark optional in base.kit
OPTIONAL_EXTS = [
    "isaacsim.anim.robot.schema",
    "isaacsim.replicator.agent.schema",
    "omni.metropolis.schema",
    "omni.behavior.tree.schema",
    "omni.anim.behavior.schema",
    "omni.anim.curve.core",
    "omni.anim.graph.schema",
    "omni.anim.navigation.schema",
    "isaacsim.util.debug_draw",
]


def find_base_kit():
    matches = glob.glob(os.path.expanduser(
        "~/miniconda3/envs/isaac6/lib/python3.12/site-packages/"
        "isaacsim/apps/isaacsim.exp.base.kit"))
    if not matches:
        raise FileNotFoundError("isaacsim.exp.base.kit not found in isaac6 env")
    return matches[0]


def patch(path):
    bak = path + ".bak"
    if not os.path.exists(bak):
        shutil.copy2(path, bak)
    text = open(path).read()
    changed = []
    for ext in OPTIONAL_EXTS:
        plain = f'"{ext}" = {{}}'
        opt = f'"{ext}" = {{ optional = true }}'
        if plain in text:
            text = text.replace(plain, opt)
            changed.append(ext)
    open(path, "w").write(text)
    return changed


if __name__ == "__main__":
    p = find_base_kit()
    changed = patch(p)
    print(f"PATCHED {p}")
    print(f"made optional ({len(changed)}): {changed}")
    print("already-optional / not-found:",
          [e for e in OPTIONAL_EXTS if e not in changed])
