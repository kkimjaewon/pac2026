#!/bin/bash
say() { echo; echo "════════════════════════════════════"; echo "  ▶ $1"; echo "════════════════════════════════════"; }
move() { ros2 service call /world/franka_world/set_pose ros_gz_interfaces/srv/SetEntityPose "{entity: {name: 'steel_plate', type: 2}, pose: {position: {x: $1, y: 0.0, z: 0.705}, orientation: {w: 1.0}}}" > /dev/null; }
fault() { ros2 param set /ai_inference_node fault_mode $1 > /dev/null; }
ros2 param set /ai_inference_node fault_delay_ms 250 > /dev/null
say "장면 1: 정상 — 로봇이 철판 위에서 대기";            sleep 5
say "장면 2: 철판을 4.5cm 이동 → AI가 찾고 로봇이 따라감"; move 0.545; sleep 10
say "장면 3: AI 지연 (250ms) → B가 HOLD, 로봇 제자리";   fault delay; sleep 10
say "        복구";                                    fault none; sleep 6
say "장면 4: AI 정지 → B가 HOLD, 로봇 제자리";          fault stop; sleep 10
say "        복구";                                    fault none; sleep 6
say "장면 5: AI 이상값 (9.9m) → B가 차단, 로봇 제자리";  fault bad_value; sleep 10
say "        복구";                                    fault none; sleep 6
say "장면 6: 철판을 원래대로 → 로봇이 다시 따라감";       move 0.50; sleep 12
say "끝 — AI가 흔들려도 로봇은 흔들리지 않는다"
