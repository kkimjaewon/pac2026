"""Validate the configured scene against SDF before spawning a guarded robot."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml


def pose_matrix(text):
    values = [float(x) for x in (text or '0 0 0 0 0 0').split()]
    if len(values) != 6 or not all(math.isfinite(x) for x in values):
        raise ValueError('finite SDF xyz/rpy pose required')
    x, y, z, r, p, yaw = values
    sr, cr, sp, cp, sy, cy = math.sin(r), math.cos(r), math.sin(p), math.cos(p), math.sin(yaw), math.cos(yaw)
    t = np.eye(4)
    t[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                  [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    t[:3, 3] = [x, y, z]
    return t


def quaternion_matrix(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def validate_scene_world(world_file, scene_file):
    """Fail closed for missing, extra, or mismatched collision models."""
    scene = yaml.safe_load(Path(scene_file).read_text())
    if scene['version'] != 1 or scene['frame'] != 'map':
        raise ValueError('scene must use version1/map')
    expected = {box['source_model']: box for box in scene['boxes']}
    if len(expected) != len(scene['boxes']):
        raise ValueError('unique source_model required')
    world = ET.parse(world_file).getroot().find('world')
    if world is None:
        raise ValueError('SDF world missing')
    if world.findall('include') or world.findall('actor') or world.findall('joint'):
        raise ValueError('included or moving world entities are not covered by this scene')
    names = [model.get('name') for model in world.findall('model')]
    if len(set(names)) != len(names):
        raise ValueError('duplicate world model names')
    virtual = {name for name, box in expected.items() if box.get('geometry_source') == 'visual'}
    actual = {model.get('name'): model for model in world.findall('model')
              if model.findall('.//collision') or model.get('name') in virtual}
    if set(actual) != set(expected):
        raise ValueError(f'world/scene obstacle names differ: actual={sorted(actual)}, expected={sorted(expected)}')
    for name, model in actual.items():
        spec = expected[name]
        if model.findtext('static', 'false').strip().lower() not in ('true', '1'):
            # Team plate is a physics body. Only its declared translation envelope
            # is modeled; arbitrary moving scenery remains forbidden.
            if not (spec.get('allow_dynamic_model') is True and name == 'steel_plate'
                    and 0 < spec.get('motion_xy_radius_m', 0) <= .05):
                raise ValueError(f'{name}: guarded obstacle must be static')
        source = spec.get('geometry_source', 'collision')
        if source not in ('collision', 'visual'):
            raise ValueError(f'{name}: unsupported geometry source')
        collisions = [(link, collision) for link in model.findall('link')
                      for collision in link.findall(source)]
        if len(collisions) != 1 or model.findall('joint') or model.findall('model'):
            raise ValueError(f'{name}: one fixed collision body required')
        link, collision = collisions[0]
        for element in (model, link, collision):
            pose = element.find('pose')
            if pose is not None and (pose.get('relative_to') or pose.get('degrees') or pose.get('rotation_format')):
                raise ValueError(f'{name}: explicit xyz/rpy poses only')
        t = pose_matrix(model.findtext('pose')) @ pose_matrix(link.findtext('pose')) @ pose_matrix(collision.findtext('pose'))
        if not np.allclose(t[:3, :3], quaternion_matrix(spec['quaternion_xyzw']), atol=1e-6, rtol=0):
            raise ValueError(f'{name}: world/scene rotation differs')
        if spec.get('geometry_type') == 'plane':
            plane = collision.find('geometry/plane')
            if plane is None or not np.allclose([float(v) for v in plane.findtext('normal').split()], [0, 0, 1]):
                raise ValueError(f'{name}: horizontal ground plane required')
            size = [float(v) for v in plane.findtext('size').split()]
            if not np.allclose(size, spec['size'][:2], atol=1e-6, rtol=0) or not np.allclose(t[:3, 3], [0, 0, 0]):
                raise ValueError(f'{name}: ground plane extent/pose differs')
            continue
        geometry = collision.find('geometry/box/size')
        if geometry is None:
            raise ValueError(f'{name}: box collision required')
        size = np.array([float(v) for v in geometry.text.split()])
        radius = spec.get('motion_xy_radius_m', 0)
        expected_size = np.array(spec['size']) - np.array([2*radius, 2*radius, 0])
        if len(size) != 3 or not np.allclose(size, expected_size, atol=1e-6, rtol=0):
            raise ValueError(f'{name}: world/scene size differs')
        delta = t[:3, 3] - np.array(spec['center'])
        if abs(delta[2]) > 1e-6 or math.hypot(*delta[:2]) > radius + 1e-6:
            raise ValueError(f'{name}: world lies outside configured motion envelope')
    return scene


def camera_from_world(world_file):
    """Use the SDF sensor pose/topic/frame as the single camera configuration."""
    world = ET.parse(world_file).getroot().find('world')
    entries = [(m, l, s) for m in world.findall('model') for l in m.findall('link')
               for s in l.findall('sensor') if s.get('type') == 'rgbd_camera']
    if len(entries) != 1:
        raise ValueError('exactly one RGB-D camera required')
    model, link, sensor = entries[0]
    transform = np.eye(4)
    for element in (model, link, sensor):
        pose = element.find('pose')
        if pose is not None and (pose.get('relative_to') or pose.get('degrees') or pose.get('rotation_format')):
            raise ValueError('camera requires explicit xyz/rpy SDF poses')
        transform = transform @ pose_matrix(element.findtext('pose'))
    r = transform[:3,:3]
    # Matrix -> quaternion without a new runtime dependency.
    trace = np.trace(r)
    if trace > 0:
        scale = math.sqrt(trace + 1.) * 2
        q = [(r[2,1]-r[1,2])/scale, (r[0,2]-r[2,0])/scale,
             (r[1,0]-r[0,1])/scale, scale/4]
    else:
        i = int(np.argmax(np.diag(r))); j=(i+1)%3; k=(i+2)%3
        scale=math.sqrt(1.+r[i,i]-r[j,j]-r[k,k])*2
        q=[0.,0.,0.,0.]; q[i]=scale/4; q[j]=(r[j,i]+r[i,j])/scale
        q[k]=(r[k,i]+r[i,k])/scale; q[3]=(r[k,j]-r[j,k])/scale
    topic = '/' + sensor.findtext('topic', '').strip('/')
    frame = sensor.findtext('gz_frame_id', '').strip()
    if topic == '/' or not frame:
        raise ValueError('camera topic and gz_frame_id required')
    return {'translation': transform[:3,3].tolist(), 'quaternion_xyzw':q,
            'topic': topic, 'frame':frame}


def world_with_pedestal(world_file, pedestal_sdf, out_dir):
    """Team world plus the validated UR5e pedestal; camera/plate/table untouched.

    Written once per content hash under out_dir and never edited in place, so the
    team world stays the single source of truth for what A sees.
    """
    import copy
    import hashlib
    tree = ET.parse(world_file)
    pedestal = ET.parse(pedestal_sdf).getroot().find("world/model[@name='robot_pedestal']")
    if pedestal is None or tree.getroot().find("world/model[@name='robot_pedestal']") is not None:
        raise ValueError('UR5e team world needs exactly one robot_pedestal')
    tree.getroot().find('world').append(copy.deepcopy(pedestal))
    text = ET.tostring(tree.getroot(), encoding='unicode')
    path = Path(out_dir)/f"{Path(world_file).stem}_ur5e_{hashlib.sha256(text.encode()).hexdigest()[:12]}.sdf"
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists():
        path.write_text(text)
    return path
