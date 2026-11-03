"""
Photos from Spot's body cameras at the moments worth looking at (confirmed block, GraphNav
stopped), and for the standing captures of tools/capture_scene.py.

Why: on 28-10 grass was seen as a wall, and the only ground truth was what we remembered.
With a photo taken by the robot itself at the moment it stops, and the camera pose in the
VISION frame next to it, the obstacle cells of the scan can be projected onto the image
afterwards: grass, bush or solid object, without guessing.

    photos.init(robot, folder)      once, at start-up (the mission does it if SAVE_BLOCK_PHOTOS)
    photos.capture('name')          -> <folder>/photos/name_<camera>.jpg + name.json

capture() never raises and does nothing if init() was not called: a photo must never stop
the mission. The JSON has, for every camera: acquisition time, VISION_tform_camera
(position + quaternion w,x,y,z), pinhole intrinsics (fx, fy, cx, cy), image size.
"""
import json
import os
import re
import time

from bosdyn.client.frame_helpers import VISION_FRAME_NAME, get_a_tform_b
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.api import image_pb2

# The five body cameras (grey, ~640x480). Same names on every Spot.
BODY_CAMERAS = ('frontleft_fisheye_image', 'frontright_fisheye_image',
                'left_fisheye_image', 'right_fisheye_image', 'back_fisheye_image')
JPEG_QUALITY = 75

_STATE = {'client': None, 'folder': None}


def init(robot, folder):
    """Image client and base folder. Returns True if the cameras can be used."""
    try:
        _STATE['client'] = robot.ensure_client(ImageClient.default_service_name)
        _STATE['folder'] = folder
        return True
    except Exception as e:
        print(f"[PHOTO] Cameras not available, no photos: {e}")
        _STATE['client'] = None
        return False


def _safe(name):
    return re.sub(r'[^A-Za-z0-9_.-]+', '_', name).strip('_')


def _camera_meta(resp):
    shot = resp.shot
    meta = {'rows': resp.source.rows, 'cols': resp.source.cols,
            'acquisition_time': shot.acquisition_time.seconds + shot.acquisition_time.nanos * 1e-9,
            'frame': shot.frame_name_image_sensor}
    try:
        tf = get_a_tform_b(shot.transforms_snapshot, VISION_FRAME_NAME, shot.frame_name_image_sensor)
        meta['vision_tform_camera'] = {
            'position': [tf.position.x, tf.position.y, tf.position.z],
            'rotation_wxyz': [tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z]}
    except Exception:
        meta['vision_tform_camera'] = None
    src = resp.source
    if src.HasField('pinhole'):
        i = src.pinhole.intrinsics
        meta['pinhole'] = {'fx': i.focal_length.x, 'fy': i.focal_length.y,
                           'cx': i.principal_point.x, 'cy': i.principal_point.y}
    elif src.HasField('pinhole_brown_conrady'):
        i = src.pinhole_brown_conrady.intrinsics.pinhole_intrinsics
        meta['pinhole'] = {'fx': i.focal_length.x, 'fy': i.focal_length.y,
                           'cx': i.principal_point.x, 'cy': i.principal_point.y}
    else:
        meta['pinhole'] = None
    return meta


def capture(name, cameras=BODY_CAMERAS, folder=None, extra=None):
    """
    One JPEG per camera in <folder>/photos/ plus <name>.json with poses and intrinsics.
    folder: default the one given to init(). extra: dict added to the JSON (e.g. a label).
    Returns the list of files written ([] if nothing). Never raises.
    """
    client = _STATE['client']
    folder = folder or _STATE['folder']
    if client is None or folder is None:
        return []
    t0 = time.perf_counter()
    name = _safe(name)
    try:
        out_dir = os.path.join(folder, 'photos')
        os.makedirs(out_dir, exist_ok=True)
        reqs = [build_image_request(c, quality_percent=JPEG_QUALITY,
                                    image_format=image_pb2.Image.FORMAT_JPEG) for c in cameras]
        responses = client.get_image(reqs, timeout=3.0)
    except Exception as e:
        print(f"[PHOTO] {name}: photos not taken ({e}).")
        return []
    written, meta = [], {'name': name, 'time': time.time(), 'cameras': {}}
    if extra:
        meta.update(extra)
    for resp in responses:
        cam = resp.source.name
        try:
            if resp.shot.image.format != image_pb2.Image.FORMAT_JPEG:
                continue        # asked JPEG: anything else is skipped rather than decoded here
            path = os.path.join(out_dir, f"{name}_{cam.replace('_image', '')}.jpg")
            with open(path, 'wb') as f:
                f.write(resp.shot.image.data)
            written.append(path)
            meta['cameras'][cam] = _camera_meta(resp)
            meta['cameras'][cam]['file'] = os.path.basename(path)
        except Exception as e:
            print(f"[PHOTO] {name}: camera {cam} not saved ({e}).")
    try:
        with open(os.path.join(out_dir, f"{name}.json"), 'w') as f:
            json.dump(meta, f, indent=1)
    except Exception as e:
        print(f"[PHOTO] {name}: poses not saved ({e}).")
    print(f"[PHOTO] {name}: {len(written)} photos in {time.perf_counter() - t0:.2f} s (photos/).")
    return written
