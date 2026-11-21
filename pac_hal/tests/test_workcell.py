"""Guarded launch must reject stale or incompletely described world geometry."""
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
from pac_hal.workcell import validate_scene_world
CONFIG = Path(__file__).resolve().parents[1] / 'config'

def test_proposed_world_matches():
    validate_scene_world(CONFIG/'fr3_workcell_demo.sdf', CONFIG/'fr3_workcell_scene.yaml')

@pytest.mark.parametrize('change', ['table', 'extra', 'plate_outside', 'relative_pose', 'moving', 'include', 'duplicate'])
def test_mismatch_rejected(tmp_path, change):
    tree = ET.parse(CONFIG/'fr3_workcell_demo.sdf')
    world = tree.getroot().find('world')
    if change == 'table':
        world.find("model[@name='table']/link/collision/geometry/box/size").text = '1 .8 .7'
    elif change == 'extra':
        model = ET.SubElement(world, 'model', name='unknown_obstacle')
        ET.SubElement(ET.SubElement(model, 'link', name='body'), 'collision', name='shape')
    elif change == 'plate_outside':
        world.find("model[@name='steel_plate']/pose").text = '.45 0 .705 0 0 0'
    elif change == 'moving':
        world.find("model[@name='table']/static").text = 'false'
    elif change == 'include':
        ET.SubElement(world, 'include')
    elif change == 'duplicate':
        world.append(ET.fromstring(ET.tostring(world.find("model[@name='table']"))))
    else:
        world.find("model[@name='table']/pose").set('relative_to', 'steel_plate')
    file = tmp_path/'bad.sdf'
    tree.write(file)
    with pytest.raises(ValueError):
        validate_scene_world(file, CONFIG/'fr3_workcell_scene.yaml')

from pac_hal.workcell import camera_from_world, quaternion_matrix, pose_matrix
import numpy as np

def test_camera_uses_sdf_pose_topic_frame():
    c=camera_from_world(CONFIG/'fr3_workcell_demo.sdf')
    assert c['topic']=='/camera' and c['frame']=='camera_link'
    assert np.allclose(c['translation'],[.35,-.6,1.5])
    t=pose_matrix('.35 -.6 1.5 0 .9212469046592403 1.5707963267948966')
    assert np.allclose(quaternion_matrix(c['quaternion_xyzw']),t[:3,:3])

def team_fixture(tmp_path):
    tree=ET.parse(CONFIG/'fr3_workcell_demo.sdf');w=tree.getroot().find('world')
    for m in list(w.findall('model')):
        if m.get('name').startswith('table_leg_'):w.remove(m)
    w.find("model[@name='table']/pose").text='.65 0 .35 0 0 0'
    for geom in w.findall("model[@name='table']/link/*/geometry/box/size"):geom.text='.7 .8 .7'
    w.find("model[@name='steel_plate']/static").text='false'
    w.find("model[@name='steel_plate']/pose").text='.5 0 .705 0 0 0'
    camera=w.find("model[@name='rgbd_camera']");camera.find('pose').text='.5 0 1.5 0 1.5708 0'
    link=camera.find('link');link.remove(link.find('collision'))
    sensor=link.find('sensor');sensor.find('topic').text='depth_camera';sensor.find('gz_frame_id').text='depth_camera_link'
    path=tmp_path/'team.sdf';tree.write(path);return path

def test_team_dynamic_plate_and_camera_virtual_guard(tmp_path):
    path=team_fixture(tmp_path)
    validate_scene_world(path,CONFIG/'fr3_team_scene.yaml')
    c=camera_from_world(path)
    assert c['topic']=='/depth_camera' and c['frame']=='depth_camera_link'
    assert np.allclose(c['translation'],[.5,0,1.5])

@pytest.mark.parametrize('change',['table_moves','plate_outside','camera_moves','camera_pose'])
def test_team_scene_change_fails_closed(tmp_path,change):
    path=team_fixture(tmp_path);tree=ET.parse(path);w=tree.getroot().find('world')
    if change=='table_moves':w.find("model[@name='table']/static").text='false'
    elif change=='plate_outside':w.find("model[@name='steel_plate']/pose").text='.6 0 .705 0 0 0'
    elif change=='camera_moves':w.find("model[@name='rgbd_camera']/static").text='false'
    else:w.find("model[@name='rgbd_camera']/pose").text='.5 0 1.6 0 1.5708 0'
    tree.write(path)
    with pytest.raises(ValueError):validate_scene_world(path,CONFIG/'fr3_team_scene.yaml')

TEAM_WORLD = Path(__file__).resolve().parents[2]/'pac_worlds/worlds/franka_world.sdf'

def test_team_cell_scenes_match_worlds_for_both_robots(tmp_path):
    from pac_hal.workcell import world_with_pedestal
    validate_scene_world(TEAM_WORLD, CONFIG/'fr3_team_scene.yaml')
    ur5e_world = world_with_pedestal(TEAM_WORLD, CONFIG/'ur5e_workcell_demo.sdf', tmp_path)
    validate_scene_world(ur5e_world, CONFIG/'ur5e_team_scene.yaml')
    # Robot swap must not change what A sees or where the work is.
    assert camera_from_world(ur5e_world) == camera_from_world(TEAM_WORLD)
    for name in ('steel_plate', 'table', 'rgbd_camera'):
        a = ET.parse(TEAM_WORLD).getroot().find(f"world/model[@name='{name}']")
        b = ET.parse(ur5e_world).getroot().find(f"world/model[@name='{name}']")
        assert ET.tostring(a) == ET.tostring(b)
    # Same obstacles as the FR3 team scene; only the pedestal is added.
    import yaml
    fr3 = yaml.safe_load((CONFIG/'fr3_team_scene.yaml').read_text())['boxes']
    ur = yaml.safe_load((CONFIG/'ur5e_team_scene.yaml').read_text())['boxes']
    assert [b for b in ur if b['name'] != 'robot_pedestal'] == fr3
    # Deterministic, written once.
    assert world_with_pedestal(TEAM_WORLD, CONFIG/'ur5e_workcell_demo.sdf', tmp_path) == ur5e_world

def test_pedestal_world_refuses_double_pedestal(tmp_path):
    from pac_hal.workcell import world_with_pedestal
    once = world_with_pedestal(TEAM_WORLD, CONFIG/'ur5e_workcell_demo.sdf', tmp_path)
    with pytest.raises(ValueError):
        world_with_pedestal(once, CONFIG/'ur5e_workcell_demo.sdf', tmp_path)

def test_ha006b_team_profile_and_scene():
    # Third arm enters through the generic profile path; same team obstacles as FR3.
    from pac_hal.contract import load_config, validate_config
    import yaml
    c = validate_config(load_config(CONFIG/'ha006b_team.yaml'))
    assert c['robot']['adapter'] == 'generic_gazebo' and c['robot']['joint_count'] == 6
    assert c['frames']['flange'] == 'tool0' and c['frames']['reference_to_base']['translation']['x'] == -0.4
    validate_scene_world(TEAM_WORLD, CONFIG/'ha006b_team_scene.yaml')
    fr3 = yaml.safe_load((CONFIG/'fr3_team_scene.yaml').read_text())['boxes']
    assert yaml.safe_load((CONFIG/'ha006b_team_scene.yaml').read_text())['boxes'] == fr3
    # Only URDF parent-child pairs plus base_link/motor_s (j1-only relation, swept clear) are skipped.
    pairs = {tuple(sorted(p)) for p in yaml.safe_load((CONFIG/'ha006b_team_scene.yaml').read_text())['disabled_self_pairs']}
    assert pairs == {tuple(sorted(p)) for p in [
        ('base_link', 'lower_frame_link'), ('lower_frame_link', 'upper_frame_link'), ('upper_frame_link', 'arm_link'),
        ('arm_link', 'wrist_body_link'), ('wrist_body_link', 'wrist_holder_link'), ('wrist_holder_link', 'flange_link'),
        ('lower_frame_link', 'motor_s'), ('arm_link', 'motor_v_r2'), ('base_link', 'motor_s')]}
