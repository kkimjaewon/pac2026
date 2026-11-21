#!/usr/bin/env python3
"""Move the simulated plate within the agreed map XY envelope; no robot command."""
import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from ros_gz_interfaces.srv import SetEntityPose


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dx', type=float, default=0., help='absolute offset from planned point, metres')
    parser.add_argument('--dy', type=float, default=0.)
    parser.add_argument('--config', default=str(Path(get_package_share_directory('pac_bringup'))/'config/scenario_config.yaml'))
    args, ros_args = parser.parse_known_args()
    config = yaml.safe_load(Path(args.config).read_text())
    if config['world_file'] != 'franka_world.sdf' or not 0 < config['accept_radius_m'] <= .05:
        parser.error('canonical FR3 world and an accept_radius <= 5cm are required')
    if not all(math.isfinite(v) for v in (args.dx,args.dy)) or math.hypot(args.dx,args.dy) >= config['accept_radius_m']:
        parser.error('XY offset must be finite and strictly inside accept_radius')
    world = ET.parse(Path(get_package_share_directory('pac_worlds'))/'worlds'/config['world_file']).getroot().find('world')
    model = world.find("model[@name='steel_plate']")
    pose = [float(v) for v in model.findtext('pose').split()]
    # Keep the physical plate centre at its SDF height, not at the AI surface height.
    req = SetEntityPose.Request();req.entity.name='steel_plate';req.entity.type=req.entity.MODEL
    req.pose.position.x=pose[0]+args.dx;req.pose.position.y=pose[1]+args.dy;req.pose.position.z=pose[2]
    req.pose.orientation.w=1.
    if any(abs(v)>1e-9 for v in pose[3:]):
        parser.error('plate_mover currently requires a horizontal plate')
    rclpy.init(args=ros_args);node=rclpy.create_node('plate_mover')
    client=node.create_client(SetEntityPose, '/world/'+world.get('name')+'/set_pose')
    try:
        if not client.wait_for_service(timeout_sec=5):raise RuntimeError('start pac_bringup sim/all first')
        future=client.call_async(req);rclpy.spin_until_future_complete(node,future,timeout_sec=5)
        if not future.done() or future.exception() or not future.result().success:
            raise RuntimeError('Gazebo did not accept the plate pose')
        print(f'plate centre=({req.pose.position.x:.3f}, {req.pose.position.y:.3f}, {req.pose.position.z:.3f})')
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
