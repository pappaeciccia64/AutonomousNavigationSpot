#!/usr/bin/env python3
"""
capture_scene.py -- standing capture: scan layers + photos of what is in front of the robot,
with a label written by you. For the grass test: robot still in front of tall grass, a bush,
a solid object; one capture each, so the layers get TRUE labels instead of ones rebuilt
from memory.

    python3 tools/capture_scene.py                 # interactive: label + Enter = one capture
    python3 tools/capture_scene.py --hostname 192.168.80.3

Read only: no lease, no motor power, the robot does not move (move it with the tablet
between captures). The robot must be on; standing is better (the layers are those of the
standing robot, as in the mission).

Every capture goes in SceneCaptures/<date_time>/ (one folder per run of the script):
  NN_<label>.npz   layers terrain, terrain_valid, obstacle_distance, intensity, no_step
                   (each with <name>_origin, <name>_cell, <name>_origin_z, VISION frame,
                   (ny, nx), row = y; terrain is in the layer frame: add terrain_origin_z),
                   ground_z (Spot's ground plane), robot_xyz, robot_yaw, label, time
  photos/NN_<label>_<camera>.jpg + NN_<label>.json   the five body cameras, with the camera
                   pose in VISION and the intrinsics (see spot_explorer/robot/photos.py)

Height above the ground of a cell = terrain + terrain_origin_z - ground_z: the quantity that should tell
low grass (4-15 cm) from a bush (above ~0.5 m).
"""
import argparse
import os
import sys
import time
from datetime import datetime
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # the spot_explorer package

from bosdyn.client.frame_helpers import BODY_FRAME_NAME, VISION_FRAME_NAME, get_a_tform_b
from spot_explorer.perception import spot_grid
from spot_explorer.robot import connection
from spot_explorer.robot import photos

LAYERS = ('terrain', 'terrain_valid', 'obstacle_distance', 'intensity', 'no_step')


def capture(local_grid, robot_state_client, folder, index, label):
    name = f"{index:02d}_{label}" if label else f"{index:02d}"
    name = photos._safe(name)
    t0 = time.time()
    layers = local_grid.extra_layers(LAYERS)
    state = robot_state_client.get_robot_state()
    snap = state.kinematic_state.transforms_snapshot
    body = get_a_tform_b(snap, VISION_FRAME_NAME, BODY_FRAME_NAME)
    q = body.rotation
    yaw = float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)))
    try:
        ground_z = local_grid.compute_ground_height_in_vision_frame(robot_state_client)
    except Exception:
        ground_z = np.nan
    data = {'label': label, 'time': t0, 'ground_z': ground_z, 'robot_yaw': yaw,
            'robot_xyz': np.array([body.position.x, body.position.y, body.position.z])}
    for k, lay in layers.items():
        data[k] = lay['values']
        data[f'{k}_origin'] = lay['origin']
        data[f'{k}_cell'] = lay['cell']
        data[f'{k}_origin_z'] = lay['origin_z']
    np.savez_compressed(os.path.join(folder, name + '.npz'), **data)
    shots = photos.capture(name, folder=folder, extra={'label': label})
    missing = [k for k in LAYERS if k not in layers]
    print(f"  -> {name}: {len(layers)} layers{' (missing: ' + ', '.join(missing) + ')' if missing else ''}, "
          f"{len(shots)} photos, ground z {ground_z:.2f} m, robot at "
          f"({body.position.x:.2f}, {body.position.y:.2f}) yaw {np.degrees(yaw):.0f} deg.")
    if 'terrain' in layers and np.isfinite(ground_z):
        h = layers['terrain']['values'] + layers['terrain']['origin_z'] - ground_z
        if 'obstacle_distance' in layers and layers['obstacle_distance']['values'].shape == h.shape:
            obst = layers['obstacle_distance']['values'] <= 0.0
            if obst.any():
                p = np.percentile(h[obst], [50, 95])
                print(f"     obstacle cells: {int(obst.sum())}, height above ground median {p[0]:.2f} m, "
                      f"95% below {p[1]:.2f} m.")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument('--hostname', default='192.168.80.3')
    args = ap.parse_args()
    options = SimpleNamespace(name="captureScene", hostname=args.hostname, verbose=False,
                              recording_user_name="", recording_session_name="")
    robot, _lease, robot_state_client, _meta = connection.setLogInfo(options)
    local_grid = spot_grid.LocalGrid(robot)
    base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    folder = os.path.join(base, 'SceneCaptures', datetime.now().strftime('%d-%m-%Y_%H-%M-%S'))
    os.makedirs(folder, exist_ok=True)
    photos.init(robot, folder)
    print(f"Connected. Captures in {folder}")
    print("Write what is in front of the robot (e.g. 'tall grass 1 m ahead') and press Enter; "
          "empty Enter = capture without label; 'q' = quit.")
    index = 1
    while True:
        try:
            label = input(f"[{index:02d}] label: ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if label.lower() in ('q', 'quit', 'exit'):
            break
        try:
            capture(local_grid, robot_state_client, folder, index, label)
            index += 1
        except Exception as e:
            print(f"  capture failed: {e}")
    print(f"{index - 1} captures in {folder}")


if __name__ == '__main__':
    main()
