"""
Download the forest assets (trees, shrubs, rocks, forest floor, sky) from NVIDIA's asset
library into sim/assets/forest/, once. After that the forest worlds load from the disk.

Run it with Isaac Sim's Python (it needs the network, about 5 minutes and 400 MB):

    sim/run_isaac.sh --fetch-forest
    (or: cd /data/Programs/isaacsim && ./python.sh <project>/sim/fetch_forest_assets.py)

What it writes:

    sim/assets/forest/palette.usd     one prim per asset under /Palette, in metres, plus the
                                      ground material and the sky; every file it needs is
                                      copied next to it
    sim/assets/forest/catalog.json    for every asset: its size and the radius of what stands
                                      below the robot's height, i.e. the obstacle it is

The worlds are then made by sim/make_forest_world.py (no Isaac Sim needed) from the catalog.
"""
import argparse
import asyncio
import json
import os
import sys
import tempfile

sys.stdout.reconfigure(line_buffering=True)

HERE = os.path.dirname(os.path.abspath(__file__))
parser = argparse.ArgumentParser(description="Download the forest assets for the simulator")
parser.add_argument("--out", default=os.path.join(HERE, "assets", "forest"))
parser.add_argument("--force", action="store_true", help="download again what is already there")
parser.add_argument("--measure-only", action="store_true", help="only write catalog.json again")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import numpy as np
import omni.client
from isaacsim.storage.native import get_assets_root_path
from omni.kit.usd.collect import Collector
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade

VEG = "/NVIDIA/Assets/Vegetation/"
# name: (path under the assets root, kind). kind: tree and shrub and rock are obstacles,
# decor is walked through (grass, fallen leaves).
ASSETS = {
    "Scarlet_Oak": (VEG + "Trees/Scarlet_Oak.usd", "tree"),
    "Red_Ash": (VEG + "Trees/Red_Ash.usd", "tree"),
    "Sycamore": (VEG + "Trees/Sycamore.usd", "tree"),
    "White_Ash": (VEG + "Trees/White_Ash.usd", "tree"),
    "Red_Oak": (VEG + "Trees/Red_Oak.usd", "tree"),
    "Shumard_Oak": (VEG + "Trees/Shumard_Oak.usd", "tree"),
    "Sugar_Maple": (VEG + "Trees/Sugar_Maple.usd", "tree"),
    "Douglas_Fir": (VEG + "Trees/Douglas_Fir.usd", "tree"),
    "Yellow_Pine": (VEG + "Trees/Yellow_Pine.usd", "tree"),
    "Elm_Sapling": (VEG + "Trees/Elm_Sapling.usd", "tree"),
    "White_Pine": (VEG + "Trees/White_Pine.usd", "shrub"),
    "Chinese_Juniper": (VEG + "Trees/Chinese_Juniper.usd", "shrub"),
    "Barberry": (VEG + "Shrub/Barberry.usd", "shrub"),
    "Rhododendron": (VEG + "Shrub/Rhododendron.usd", "shrub"),
    "Goldflame_Spirea": (VEG + "Shrub/Goldflame_Spirea.usd", "shrub"),
    "Switchgrass": (VEG + "Shrub/Switchgrass.usd", "shrub"),
    "Grass_Short_A": (VEG + "Shrub/Grass_Short_A.usd", "decor"),
    "Grass_Short_B": (VEG + "Shrub/Grass_Short_B.usd", "decor"),
    "Grass_Short_C": (VEG + "Shrub/Grass_Short_C.usd", "decor"),
    "Lupin": (VEG + "Shrub/Lupin.usd", "shrub"),
    "Leaves_Oak": (VEG + "Debris/oakfall1.usd", "decor"),
    "Leaves_Maple": (VEG + "Debris/maplefall1.usd", "decor"),
    "Leaves_Cluster": (VEG + "Debris/fallcluster2.usd", "decor"),
}
for _i in (1, 3, 5, 8, 11, 14):
    ASSETS["Rock_%02d" % _i] = (VEG + "Rocks/rock_small_%02d.usda" % _i, "rock")
GROUND_MDL = ("/NVIDIA/Materials/Base/Natural/Leaves.mdl", "Leaves")
SKY_HDR = "/NVIDIA/Assets/Skies/Clear/noon_grass_4k.hdr"
# What is below this height is in the robot's way (the body top is at about 0.7 m).
OBSTACLE_BELOW_M = 0.9


def build_palette(root, path):
    """A stage with every asset under /Palette, scaled to metres and Z up."""
    stage = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    palette = UsdGeom.Xform.Define(stage, "/Palette")
    stage.SetDefaultPrim(palette.GetPrim())
    missing = []
    for name, (rel, _kind) in ASSETS.items():
        url = root + rel
        result, _ = omni.client.stat(url)
        if result != omni.client.Result.OK:
            missing.append(name)
            print(f"[FETCH] {name}: not found in the asset library ({rel}), skipped.")
            continue
        src = Usd.Stage.Open(url)
        mpu = UsdGeom.GetStageMetersPerUnit(src)
        up = UsdGeom.GetStageUpAxis(src)
        del src
        # Three levels: the named prim stays without transforms (the worlds put theirs on
        # it), "m" converts to metres and Z up, "asset" is the library file untouched.
        prim = UsdGeom.Xform.Define(stage, f"/Palette/{name}")
        metres = UsdGeom.Xform.Define(stage, f"/Palette/{name}/m")
        inner = stage.DefinePrim(f"/Palette/{name}/m/asset")
        inner.GetReferences().AddReference(url)
        xf = UsdGeom.Xformable(metres.GetPrim())
        if up == UsdGeom.Tokens.y:
            xf.AddRotateXOp().Set(90.0)
        xf.AddScaleOp().Set(Gf.Vec3f(mpu, mpu, mpu))
        prim.GetPrim().SetCustomDataByKey("source", rel)
    # Ground material and sky, so that the collection copies their files too.
    material = UsdShade.Material.Define(stage, "/Palette/Looks/Ground")
    shader = UsdShade.Shader.Define(stage, "/Palette/Looks/Ground/Shader")
    shader.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset)
    shader.SetSourceAsset(Sdf.AssetPath(root + GROUND_MDL[0]), "mdl")
    shader.SetSourceAssetSubIdentifier(GROUND_MDL[1], "mdl")
    out = shader.CreateOutput("out", Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput("mdl").ConnectToSource(out)
    material.CreateDisplacementOutput("mdl").ConnectToSource(out)
    material.CreateVolumeOutput("mdl").ConnectToSource(out)
    sky = UsdLux.DomeLight.Define(stage, "/Palette/Sky")
    sky.CreateTextureFileAttr(Sdf.AssetPath(root + SKY_HDR))
    sky.CreateTextureFormatAttr(UsdLux.Tokens.latlong)
    sky.CreateIntensityAttr(1000.0)
    stage.GetRootLayer().Save()
    return missing


def _points(stage, prim):
    """The points of the asset where they are SEEN, in metres: (trunk and other plain meshes,
    instanced foliage). The foliage of the library trees is a handful of branch meshes drawn
    many times by point instancers: the branch meshes themselves sit near the origin."""
    cache = UsdGeom.XformCache()
    rng = np.random.RandomState(0)
    prototypes, foliage, plain = set(), [], []
    for p in Usd.PrimRange(prim):
        if not p.IsA(UsdGeom.PointInstancer):
            continue
        inst = UsdGeom.PointInstancer(p)
        targets = inst.GetPrototypesRel().GetTargets()
        prototypes.update(targets)
        indices = inst.GetProtoIndicesAttr().Get() or []
        xforms = inst.ComputeInstanceTransformsAtTime(Usd.TimeCode.Default(), Usd.TimeCode.Default())
        to_world = np.array(cache.GetLocalToWorldTransform(p))              # row vectors
        proto_pts = []
        for t in targets:
            pts = []
            for q in Usd.PrimRange(stage.GetPrimAtPath(t)):
                if q.IsA(UsdGeom.Mesh):
                    v = UsdGeom.Mesh(q).GetPointsAttr().Get()
                    if v:
                        v = np.asarray(v, dtype=np.float64)
                        pts.append(v[rng.choice(len(v), size=min(len(v), 1500), replace=False)])
            proto_pts.append(np.concatenate(pts) if pts else np.zeros((0, 3)))
        for k, m in zip(indices, xforms):
            v = proto_pts[k]
            if len(v):
                m = np.array(m) @ to_world
                foliage.append(v @ m[:3, :3] + m[3, :3])
    for p in Usd.PrimRange(prim):
        if p.IsA(UsdGeom.Mesh) and not any(p.GetPath().HasPrefix(t) for t in prototypes):
            v = UsdGeom.Mesh(p).GetPointsAttr().Get()
            if v:
                m = np.array(cache.GetLocalToWorldTransform(p))
                plain.append(np.asarray(v, dtype=np.float64) @ m[:3, :3] + m[3, :3])
    cat = lambda parts: np.concatenate(parts) if parts else np.zeros((0, 3))  # noqa: E731
    return cat(plain), cat(foliage)


def measure(stage, name, kind):
    """Size of the asset in metres, and the obstacle it makes at the robot's height.

        obstacle_r_m    radius of the obstacle around its axis (cx_m, cy_m). Tree: the trunk
                        between 0.2 m and the robot's height. Shrub: everything in that band.
                        Rock and the rest: the whole footprint.
        low_r_m         tree only: the same band with the foliage, for a tree whose branches
                        come down to the robot
        foliage_low_m   height where the foliage starts (its lowest 3%); 0 if there is none
    """
    prim = stage.GetPrimAtPath(f"/Palette/{name}")
    plain, foliage = _points(stage, prim)
    everything = np.concatenate([plain, foliage])
    if not len(everything):
        return None
    lo, hi = everything.min(axis=0), everything.max(axis=0)
    info = {"height_m": round(float(hi[2]), 2),
            "spread_m": round(float(np.percentile(np.hypot(everything[:, 0], everything[:, 1]), 98)), 2),
            "foliage_low_m": round(float(np.percentile(foliage[:, 2], 3)), 2) if len(foliage) else 0.0}

    def band(points):
        return points[(points[:, 2] > 0.2) & (points[:, 2] < OBSTACLE_BELOW_M)]

    def radius(points, c):
        return float(np.percentile(np.hypot(points[:, 0] - c[0], points[:, 1] - c[1]), 98)) if len(points) else 0.0

    low = band(everything)
    trunk = band(plain) if len(plain) else plain
    if kind == "tree" and len(trunk) >= 10:
        c = np.median(trunk[:, :2], axis=0)
        r, r_low = radius(trunk, c), radius(low, c)
    elif kind in ("tree", "shrub") and len(low) >= 10:
        c = np.median(low[:, :2], axis=0)
        r = r_low = radius(low, c)
    else:
        c = 0.5 * (lo[:2] + hi[:2])
        r = r_low = radius(everything, c)
    info.update({"cx_m": round(float(c[0]), 3), "cy_m": round(float(c[1]), 3),
                 "obstacle_r_m": round(r, 3), "low_r_m": round(r_low, 3)})
    return info


async def collect(src, out):
    collector = Collector(src, out, usd_only=False, flat_collection=False, skip_existing=not args.force)
    last = [-1]

    def progress(step, total):
        pct = int(100 * step / max(total, 1))
        if pct // 10 != last[0]:
            last[0] = pct // 10
            print(f"[FETCH] downloading... {pct}% ({step}/{total} files)")

    return await collector.collect(progress, None)


def main():
    root = get_assets_root_path()
    if not root:
        sys.exit("[FETCH] The Isaac Sim asset library is not reachable (network?).")
    print(f"[FETCH] Asset library: {root}")
    os.makedirs(args.out, exist_ok=True)
    palette = os.path.join(args.out, "palette.usd")
    missing = []
    if args.measure_only and os.path.exists(palette):
        print("[FETCH] --measure-only: nothing downloaded, the catalog is measured again.")
    else:
        tmp = os.path.join(tempfile.mkdtemp(prefix="forest_palette_"), "palette.usd")
        missing = build_palette(root, tmp)
        task = asyncio.ensure_future(collect(tmp, args.out))
        while not task.done():
            simulation_app.update()
        ok, collected = task.result()
        if not ok:
            sys.exit("[FETCH] The download failed: " + str(collected))
    if not os.path.exists(palette):
        sys.exit(f"[FETCH] palette.usd not found in {args.out} after the download ({collected}).")
    stage = Usd.Stage.Open(palette)
    catalog = {"obstacle_below_m": OBSTACLE_BELOW_M, "ground_material": "/Palette/Looks/Ground",
               "sky": "/Palette/Sky", "assets": {}}
    for name, (rel, kind) in ASSETS.items():
        if name in missing or not stage.GetPrimAtPath(f"/Palette/{name}").IsValid():
            continue
        info = measure(stage, name, kind)
        if info is None:
            print(f"[FETCH] {name}: no geometry, skipped.")
            continue
        info.update({"kind": kind, "source": rel})
        catalog["assets"][name] = info
        print(f"[FETCH] {name:18s} {kind:6s} height {info['height_m']:5.2f} m, spread {info['spread_m']:5.2f} m, "
              f"obstacle r {info['obstacle_r_m']:.2f} m, with foliage {info['low_r_m']:.2f} m, "
              f"foliage from {info['foliage_low_m']:.2f} m up")
    with open(os.path.join(args.out, "catalog.json"), "w") as f:
        json.dump(catalog, f, indent=1)
    size = sum(os.path.getsize(os.path.join(d, n)) for d, _, names in os.walk(args.out) for n in names)
    print(f"[FETCH] Done: {len(catalog['assets'])} assets, {size / 1e6:.0f} MB in {args.out}")


main()
simulation_app.close()
