#!/bin/bash
LOG=~/plate_truth_bag1.txt; : > $LOG
step() {
  ros2 service call /world/franka_world/set_pose ros_gz_interfaces/srv/SetEntityPose "{entity: {name: 'steel_plate', type: 2}, pose: {position: {x: $1, y: 0.0, z: 0.705}, orientation: {w: 1.0}}}" > /dev/null
  echo "$(date +%s.%N) $(date +%H:%M:%S) plate_x=$1" | tee -a $LOG
  ros2 topic pub -r 2 -t 20 /plate_truth geometry_msgs/msg/PointStamped "{header: {frame_id: map}, point: {x: $1, y: 0.0, z: 0.71}}" > /dev/null
}
step 0.500
step 0.505
step 0.510
step 0.530
step 0.500
echo "끝 ✅  정답 기록: $LOG"
