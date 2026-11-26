"""
The scene of the simulator in Isaac Sim, built from a world file: ground, light, obstacles
and what only makes the place look like what it is (grass, leaves). Imported by
isaac_spot_server.py after the simulation app has started.

Two things are kept apart on purpose:

  what the robot bumps into   one simple solid per shape of the world file (a cylinder or a
                              box): exactly the geometry the mission's grid is computed from
  what is seen                for a shape with an "asset", the tree, shrub or rock of the
                              downloaded library drawn in its place (sim/assets/forest/); the
                              solid is then hidden. Cameras and lidar see these.

The ground is flat (the grid floor of Isaac Sim, or the forest floor) or, when the world file
has a "terrain", the hills sampled from world.height(): the same function the mission's grid
uses, so slopes seen, walked and mapped agree. Solids and plants stand on the ground under
their centre.

Without the downloaded assets a forest world still runs: plain cylinders and boxes, and a line
in the output says so.
"""
import json
import math
import os

import carb
import numpy as np
import omni.usd
from isaacsim.core.experimental.utils.stage import define_prim
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade

GROUND_TILE_M = 2.0             # side of one repetition of the forest floor texture
TRUNK_COLOR = Gf.Vec3f(0.35, 0.25, 0.15)
BOX_COLOR = Gf.Vec3f(0.75, 0.35, 0.20)
LOG_COLOR = Gf.Vec3f(0.30, 0.20, 0.12)
TERRAIN_DEFAULT_EXTENT_M = (-15.0, 45.0, -30.0, 30.0)     # x min, x max, y min, y max
TERRAIN_MESH_CELL_M = 0.25
SINK_M = 0.3                    # solids go this far into the ground, so none floats on a slope


class ForestAssets:
    """The downloaded library: palette.usd and catalog.json (see fetch_forest_assets.py)."""

    def __init__(self, folder):
        self.palette = os.path.join(folder, "palette.usd")
        self.assets = {}
        self.ok = False
        catalog = os.path.join(folder, "catalog.json")
        if os.path.exists(self.palette) and os.path.exists(catalog):
            with open(catalog) as f:
                self.assets = json.load(f).get("assets", {})
            self.ok = True

    def has(self, name):
        return self.ok and name in self.assets


def _place(stage, path, assets, name, x, y, z, scale, spin_deg):
    """The library asset `name` with its trunk axis on (x, y): the assets are not all centred."""
    info = assets.assets[name]
    a = math.radians(spin_deg)
    cx, cy = info.get("cx_m", 0.0) * scale, info.get("cy_m", 0.0) * scale
    px = x - (math.cos(a) * cx - math.sin(a) * cy)
    py = y - (math.sin(a) * cx + math.cos(a) * cy)
    prim = UsdGeom.Xform.Define(stage, path).GetPrim()
    prim.GetReferences().AddReference(assets.palette, f"/Palette/{name}")
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(px, py, z))
    xf.AddRotateZOp().Set(float(spin_deg))
    xf.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    prim.SetInstanceable(True)          # one copy of each tree in memory, however many are planted
    return prim


def _solid(stage, path, s, base_z, hidden):
    """The collider of one shape, standing on the ground at base_z (sunk SINK_M into it).
    Hidden = drawn only as a guide (the asset is what is seen)."""
    ground_z = base_z - SINK_M
    h = float(s["h"]) + SINK_M
    if s["type"] == "cylinder":
        geom = UsdGeom.Cylinder.Define(stage, path)
        geom.CreateAxisAttr("Z")
        geom.CreateRadiusAttr(float(s["r"]))
        geom.CreateHeightAttr(h)
        scale, yaw_deg, color, z = Gf.Vec3f(1.0, 1.0, 1.0), 0.0, TRUNK_COLOR, ground_z + 0.5 * h
    elif s.get("look") == "log":        # a fallen trunk: a lying cylinder as long as the box
        geom = UsdGeom.Cylinder.Define(stage, path)
        geom.CreateAxisAttr("X")
        geom.CreateRadiusAttr(0.5 * float(s["sy"]))
        geom.CreateHeightAttr(float(s["sx"]))
        scale, yaw_deg, color, z = Gf.Vec3f(1.0, 1.0, 1.0), float(s["yaw_deg"]), LOG_COLOR, base_z + 0.5 * float(s["sy"])
    else:
        geom = UsdGeom.Cube.Define(stage, path)
        geom.CreateSizeAttr(1.0)
        scale, yaw_deg, color, z = Gf.Vec3f(float(s["sx"]), float(s["sy"]), h), float(s["yaw_deg"]), BOX_COLOR, ground_z + 0.5 * h
    xf = UsdGeom.Xformable(geom.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(float(s["x"]), float(s["y"]), z))
    xf.AddRotateZOp().Set(yaw_deg)
    xf.AddScaleOp().Set(scale)
    geom.CreateDisplayColorAttr([color])
    UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
    if hidden:
        UsdGeom.Imageable(geom.GetPrim()).CreatePurposeAttr(UsdGeom.Tokens.guide)


def _forest_ground(stage, world, assets):
    size = float(world.ground.get("size_m", 90.0))
    half, z = 0.5 * size, world.ground_z
    # What is seen: one textured quad. What is walked on: a thick slab under it.
    mesh = UsdGeom.Mesh.Define(stage, "/World/Ground/Floor")
    mesh.CreatePointsAttr([(-half, -half, z), (half, -half, z), (half, half, z), (-half, half, z)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([(0, 0, 1)] * 4)
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    mesh.CreateExtentAttr([(-half, -half, z), (half, half, z)])
    n = size / GROUND_TILE_M
    UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex).Set([(0, 0), (n, 0), (n, n), (0, n)])
    mesh.CreateDisplayColorAttr([Gf.Vec3f(0.25, 0.22, 0.12)])
    looks = stage.DefinePrim("/World/Looks/Ground")
    looks.GetReferences().AddReference(assets.palette, "/Palette/Looks/Ground")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(UsdShade.Material(looks))

    slab = UsdGeom.Cube.Define(stage, "/World/Ground/Slab")
    slab.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(slab.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, z - 0.5))
    xf.AddScaleOp().Set(Gf.Vec3f(size, size, 1.0))
    UsdPhysics.CollisionAPI.Apply(slab.GetPrim())
    UsdGeom.Imageable(slab.GetPrim()).CreatePurposeAttr(UsdGeom.Tokens.guide)
    grip = UsdShade.Material.Define(stage, "/World/Looks/GroundGrip")
    api = UsdPhysics.MaterialAPI.Apply(grip.GetPrim())
    api.CreateStaticFrictionAttr(1.0)
    api.CreateDynamicFrictionAttr(1.0)
    api.CreateRestitutionAttr(0.0)
    UsdShade.MaterialBindingAPI.Apply(slab.GetPrim()).Bind(grip, UsdShade.Tokens.weakerThanDescendants, "physics")



def _lights(stage, assets, forest):
    """Sky and sun. Under the trees a little even light stands for what the real-time renderer
    does not bounce off leaves and ground; without it the forest floor is almost black.
    (Automatic exposure, /rtx/post/histogram/enabled, would be the natural choice, but with
    the tiled sensor cameras it crashed the GPU driver here: Isaac Sim 6.2, RTX 5070, driver 580.)"""
    if forest:
        sky = stage.DefinePrim("/World/Sky")
        sky.GetReferences().AddReference(assets.palette, "/Palette/Sky")
        settings = carb.settings.get_settings()
        settings.set("/rtx/sceneDb/ambientLightIntensity", 1.0)
        settings.set("/rtx/sceneDb/ambientLightColor", (0.55, 0.65, 0.55))
    else:
        UsdLux.DomeLight.Define(stage, "/World/SkyLight").CreateIntensityAttr(700.0)
    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    sun.CreateIntensityAttr(4000.0 if forest else 2500.0)
    sun.CreateAngleAttr(1.0)
    sun.CreateColorAttr(Gf.Vec3f(1.0, 0.96, 0.88))
    UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(42.0, 18.0, 35.0))


def _terrain(stage, world, assets, forest):
    """The ground of a world with hills: a triangle mesh sampled from world.height(), with a
    collider; with the forest floor material when the assets are there."""
    x0, x1, y0, y1 = [float(v) for v in world.terrain.get("extent_m", TERRAIN_DEFAULT_EXTENT_M)]
    step = float(world.terrain.get("mesh_cell_m", TERRAIN_MESH_CELL_M))
    xs = np.arange(x0, x1 + 0.5 * step, step)
    ys = np.arange(y0, y1 + 0.5 * step, step)
    gx, gy = np.meshgrid(xs, ys)
    gz = world.height(gx, gy)
    nx, ny = len(xs), len(ys)
    points = np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)
    # Two triangles per cell, counterclockwise seen from above (normals up).
    i, j = np.meshgrid(np.arange(nx - 1), np.arange(ny - 1))
    a = (j * nx + i).ravel()
    b, c, d = a + 1, a + nx + 1, a + nx
    faces = np.stack([a, b, c, a, c, d], axis=1).ravel()

    mesh = UsdGeom.Mesh.Define(stage, "/World/Ground/Terrain")
    mesh.CreatePointsAttr([Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])) for p in points])
    mesh.CreateFaceVertexCountsAttr([3] * (len(faces) // 3))
    mesh.CreateFaceVertexIndicesAttr([int(v) for v in faces])
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateDisplayColorAttr([Gf.Vec3f(0.33, 0.42, 0.24)])
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("none")
    if forest:
        st = (points[:, :2] - [x0, y0]) / GROUND_TILE_M
        UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex).Set(
            [Gf.Vec2f(float(u), float(v)) for u, v in st])
        looks = stage.DefinePrim("/World/Looks/Ground")
        looks.GetReferences().AddReference(assets.palette, "/Palette/Looks/Ground")
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(UsdShade.Material(looks))
    print(f"[SIM] Terrain: {nx} x {ny} points every {step:.2f} m over x {x0:.0f}..{x1:.0f}, "
          f"y {y0:.0f}..{y1:.0f} m, heights {gz.min():+.2f}..{gz.max():+.2f} m. Outside this "
          f"area there is no ground.")
    print("[SIM] NOTE: the locomotion policy was trained on FLAT ground. Gentle slopes may "
          "work, steep ones may make the robot stumble or fall.")


def build_scene(world, assets_folder, isaac_assets_root):
    """Ground, light, obstacles and decor of `world` (spot_explorer.sim.world.World)."""
    stage = omni.usd.get_context().get_stage()
    assets = ForestAssets(assets_folder)
    wants_assets = any("asset" in s for s in world.shapes) or bool(world.decor)
    forest_floor = world.ground.get("style") == "forest" and assets.ok

    define_prim("/World/Ground", "Xform")
    if not world.is_flat:
        _terrain(stage, world, assets, forest_floor)
        _lights(stage, assets, forest_floor)
    elif forest_floor:
        _forest_ground(stage, world, assets)
        _lights(stage, assets, True)
    else:                               # the grid floor brings its own lights
        stage.GetPrimAtPath("/World/Ground").GetReferences().AddReference(
            isaac_assets_root + "/Isaac/Environments/Grid/default_environment.usd")

    define_prim("/World/Obstacles", "Xform")
    define_prim("/World/Forest", "Xform")
    drawn = 0
    for i, s in enumerate(world.shapes):
        asset = s.get("asset")
        seen_as_asset = asset is not None and assets.has(asset)
        base_z = world.shape_base_z(s)
        _solid(stage, f"/World/Obstacles/shape_{i:03d}", s, base_z, hidden=seen_as_asset)
        if seen_as_asset:
            _place(stage, f"/World/Forest/{asset}_{i:03d}", assets, asset, float(s["x"]), float(s["y"]),
                   base_z, float(s.get("scale", 1.0)), float(s.get("spin_deg", 0.0)))
            drawn += 1
    decor = 0
    for i, d in enumerate(world.decor):
        if assets.has(d.get("asset")):
            _place(stage, f"/World/Forest/decor_{i:04d}", assets, d["asset"], float(d["x"]), float(d["y"]),
                   world.height_at(float(d["x"]), float(d["y"])), float(d.get("scale", 1.0)),
                   float(d.get("spin_deg", 0.0)))
            decor += 1
    line = f"[SIM] World '{world.name or 'empty'}': {len(world.shapes)} obstacles built"
    if drawn or decor:
        line += f", {drawn} drawn as trees, shrubs and rocks, {decor} tufts of grass and leaves"
    print(line + ".")
    if wants_assets and not assets.ok:
        print(f"[SIM] The forest assets are not in {assets_folder}: plain solids on the grid floor. "
              f"Download them once with: sim/run_isaac.sh --fetch-forest")
