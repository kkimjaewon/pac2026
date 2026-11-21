# C계층 공통 인터페이스와 하위 로봇 설정 (v0.3)

기준: 사용자가 제공한 「인터페이스 0~5」 v0.3 문서, 2026-10-05.
기존 Git README보다 최신 A→B ROS Topic, 경과 시간 age_ns 계약을 적용한다.
외부 문서의 메시지 필드/상수/토픽은 변경하지 않고 코드를 맞췄다.
패키지 이름은 현재 Git의 intelion_interfaces/pac_hal을 유지한다.
문서의 hal_system은 `pac_hal` 역할이며 실행 파일명은 hal_system.launch.py다.

## 상위 계층이 알아야 할 내용

A는 ControlCommand를 /ai/control_command로 발행한다. A 로직은 이번에 구현하지 않았다.
B는 ROS 통신 스레드에서 HalCommand를 발행하고 HalState/HalError를 수신한다.
B의 1ms RT 루프에서 publish/콜백/파일 I/O를 수행하지 않는다.
B와 C는 같은 PC-RT에 있어야 한다. 다른 PC의 monotonic 시각으로 expires를 비교하지 않는다.
ROS_DOMAIN_ID=42, RMW_IMPLEMENTATION=rmw_fastrtps_cpp는 실행 환경에서 맞춘다.
B는 MODE_POSITION 하나만 사용하며 HOLD/STOP을 새 mode 번호로 임의 추가하지 않는다.

HalCommand는 session_id, command_seq, created_mono_ns, expires_mono_ns,
header.frame_id='map', target_pose, source_result_id를 반드시 채운다.
expires_mono_ns = created_mono_ns + 5_000_000.
자세는 정규화 quaternion(x,y,z,w). TCP 위치는 map 기준 절대 좌표(m).
값이 같아도 command_seq를 올린다. 경로 편차를 C에서 다시 누적하지 않는다.
header.stamp는 시뮬레이션 시간이며 만료 계산에 사용하지 않는다.

B는 HAL_READY/HAL_ACTIVE, pose_valid 및 stale 여부를 확인하고 움직임을 허용한다.
CMD_ACCEPTED(검사 통과), CMD_SENT(전송), TCP 도달은 서로 다른 의미다.
기본 계약에는 개별 명령 적용 ACK가 없다. 일반 상태 수신을 적용 완료로 보고하지 않는다.
interface_only 모드는 HAL_INITIALIZING, stale=true, pose_valid/twist_valid=false다.
기본 gazebo 모드는 실제 측정 FK와 활성 수신단의 신선도가 확보될 때 READY/ACTIVE를 보고한다.
따라서 B는 이 구현으로 로봇 운동을 시작할 수 없다.

## 현재 구현 범위

- 메시지 4종 생성: C++/Python 공용 타입.
- robot별 YAML과 시작 시 엄격한 설정 검증. 오타/누락/위험한 수치/계약 변경은 실패한다.
- 명령의 세션·순번·좌표·mode·시각·5ms TTL·NaN·quaternion·작업영역 검사.
- 오류는 발생/해제 전이 때만 발행, 현재 오류 목록은 HalState.active_errors에 제공.
- 진단 heartbeat가 있어도 새 측정이 없으면 state_seq/received_mono_ns/header.stamp를 갱신하지 않는다.
- CSV 필수 컬럼을 기록하되 발생하지 않은 단계는 빈 칸으로 둔다.
- Gazebo backend는 공식 URDF KDL FK/IK, 관절 제한, ros2_control position 수신단과 감속 정지를 구현한다.
- HalJointCommand/Status는 C 내부 채널이며 B/C 공통 메시지 필드를 바꾸지 않는다.
- interface_only는 모든 운동 명령을 거부한다. 형식 검사 성공도 E_UNSUPPORTED로 보고한다.
- Python gateway의 CSV/ROS 경로는 실시간 스레드가 아니다. C 1kHz/지터 달성을 주장하지 않는다.

## 설정 구조와 책임

C가 쓰는 두 profile은 `pac_hal/config/franka_fr3.yaml`, `pac_hal/config/ur5e.yaml`이다.
상위 통합 launch가 robot/config_file/session_id/run_id/log_directory를 선택한다.
로봇 내부 구조를 B 코드에 넣지 않고 C profile 안에 모은다.

| 섹션 | 주요 항목 | 의미·검증 |
|---|---|---|
| robot | id/model/adapter/execution_backend | FR3 7축 / UR5e 6축. gazebo 실행 / interface_only 검사 모드 |
| frames | reference/base/flange/tcp | map, 로봇 base, 플랜지, 공구 끝 링크 |
| frames | reference_to_base | ^map T_base, 설치 위치·자세 |
| frames | flange_to_tcp | ^flange T_tcp, 공구 길이·장착 방향 |
| joints | name/device_index/direction/zero_offset_rad | canonical 이름 ↔ 장치 축 순서, ±1 방향, 영점 |
| joints | lower/upper, max_velocity/max_acceleration | rad, rad/s, rad/s². 실제 적용은 향후 어댑터가 URDF와 교차 검증 |
| workspace | min_m/max_m | map xyz 범위. 도달 가능·충돌 안전 보장은 아님 |
| workspace | max_tcp_speed/max_tcp_acceleration | B와 합의할 제약. IK 후 관절 제약을 대신하지 않음 |
| timing | command_period_ns / command_ttl_ns | 1ms / 5ms, v0.3 고정. AI TTL 200ms와 별개 |
| timing | state_period_ns | 로봇 상태 실제 수신 주기 예상. 실측 후 조정 |
| timing | state_stale_ns | max(5ms, 3×state_period_ns) |
| timing | state_lost_ns | max(20ms, 10×state_period_ns) |
| timing | receiver_watchdog_ns | 수신단 목표 중단 검출 기준. 현재 설정만, 실제 watchdog 없음 |
| timing | ik_budget_ns | 500μs 초안, 미측정. 반복 수 제한과 함께 사용 예정 |
| ik | solver/iterations/tolerances/seed | KDL·최대30회·1mm/0.01rad 초안·직전 해 seed |
| transport | controller_manager/controller_name/topics | 로봇별 명령·측정 경로. 이 런치는 제어기를 활성화하지 않음 |
| transport | receiver_stop_strategy | 감속 후 유지 계획. 실제 구현 전에는 정지 기능으로 간주 금지 |
| topics/qos | 문서의 고정 토픽과 QoS | 설정에서 임의 변경하면 시작 거부 |
| runtime | active_session_id/state_heartbeat_hz | 명시적 비영 세션, 기본 진단 10Hz |
| logging | run_id/directory/enabled | ~/pac_logs/<run_id>/hal.csv. 실제 경로는 런치 인자로 지정 가능 |

관절 위치 범위는 설치된 공식 description에서 가져왔다.
속도 0.5rad/s, 가속도 1rad/s²는 시연 운영 제한이다. 제조사 정격 가속도라고 주장하지 않는다.
모든 설치/TCP 변환 초기값은 identity다. 실제 셀 위치·공구 offset으로 바꾸어야 한다.
FR3의 hal_position_controller는 계획 이름이며 현재 실행 중인 gravity compensation과 별개다.
YAML을 로드하는 것만으로 시뮬레이터의 controller 설정/TF가 바뀌지는 않는다.

변환 규약: ^A T_B는 B 좌표를 A로 옮기는 변환.
`^base T_flange_target = inverse(^map T_base) × ^map T_tcp_target × inverse(^flange T_tcp)`.
장치 값: `q_device[device_index] = direction * q_canonical + zero_offset_rad`.
상태 복원 시 역변환. Gazebo backend의 KDL FK/IK와 장치 매핑에서 사용한다. 실제 설치값은 팀이 확정해야 한다.

## 빌드·실행

팀 워크스페이스 src에 intelion_interfaces와 pac_hal이 있어야 한다.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select intelion_interfaces pac_hal
source install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch pac_hal hal_system.launch.py robot:=franka_fr3 backend:=interface_only \
  session_id:=42 run_id:=20261005_fr3_interface
```

UR 교체: 기존 C를 종료한 뒤 robot:=ur5e, 새 session_id, 새 run_id로 실행한다.
run_id는 A·B·C가 같은 값을 쓰되 독립 실험은 새 ID를 사용한다.
기존 hal.csv는 덮어쓰거나 이어붙이지 않고 오류로 종료해 실험 혼합을 막는다.

사용자 설정:

```bash
ros2 launch pac_hal hal_system.launch.py robot:=franka_fr3 \
  config_file:=/absolute/path/custom_fr3.yaml \
  session_id:=42 run_id:=20261005_custom log_directory:=/tmp/pac_logs
```

session_id=0 기본값은 B 운동 세션을 허용하지 않는다.
현재 세션 등록 수단은 시작 시 YAML/런치다. 런치 session_id가 YAML 값을 덮어쓴다.
B 재시작/새 RUN 때 C도 새로운 같은 session_id로 재시작한다. 첫 명령으로 자동 세션 교체하지 않는다.
이 운영 규약은 문서의 세션 등록 경로가 미정인 점을 보완하는 초기 제안이며 팀 확인이 필요하다.
런치에서 robot은 기본 profile 선택에 쓰이고 custom config_file을 지정하면 YAML robot.model이 기준이다.

## 로그 정의

`session_id,command_seq,t1_mono_ns,...,t6_mono_ns,result,error_code`.

- t1: C 명령 콜백 수신.
- t2: 공통 명령 검증 종료.
- t3: 변환·IK·관절 검사 완료.
- t4: 로봇 전송 직전, 두 번째 만료 검사 후.
- t5(향후): 관련 로봇 상태 수신. 일반 상태만으로 명령 ACK를 대체하지 않는다.
- t6: C 내부 position 수신단의 목표 채택 ACK 수신. TCP 도달을 뜻하지 않는다.

gazebo는 t3/t4와 별도 applied 행의 t6를 기록한다. t5는 상관된 측정 ACK가 없어 비어 있다.
interface_only의 t3~t6는 빈 칸이다. 수신단 100Hz 상태는 모든 명령의 ACK를 담지 않는다. header.stamp/Gazebo 시간을 monotonic 열에 넣지 않는다.
이 t1~t6 의미는 사용자 문서에서 이름만 주어진 부분에 대한 C 제안이다.
세션 불일치/중복/역순은 별도 코드가 없는 v0.3에서 E_FORMAT, 상세 detail로 구분한다.
새 오류코드를 독단적으로 추가하지 않는다.

## 후속 구현 및 팀 합의 항목

1. 세션 설정/재개 경로와 source_result_id=0 모호함 확인.
2. 실제 설치/TCP 변환 확정, 실물 전송 어댑터 및 충돌 검사 구현.
3. Gazebo 수신단 만료/세션/watchdog 정지는 구현했다. 실물 정지 전략은 별도 검증.
4. 공통 HalState에 적용 ACK를 추가할지는 팀 합의. 현재 ACK는 C 내부 전용이다.
5. 카메라 토픽: 현재 팀 월드는 /camera/*이며 v0.3의 /depth_camera/*와 다름.
   공통 기반 담당이 브리지 remap을 반영해야 한다. C 패키지는 다른 담당 파일을 수정하지 않는다.
6. 현재 월드 카메라 15Hz/32FC1와 문서의 15Hz/16UC1·30Hz/32FC1 후보는 실제 입력으로 최종 확정.

인터페이스를 바꾸어야 할 때는 문서·README도 함께 수정하고 A/B/C가 같은 빌드를 사용한다.
이번 작업에서 팀원에게 메시지를 보내거나 원격 Git에 push하지 않았다.

## B 통신 스레드의 C++ 명령 작성 예

아래는 메시지 작성 예이며 RT 루프 내부 publish 예제가 아니다.
B가 계산한 최신 target_pose와 목표 생성 시각을 통신 스레드에서 함께 복사해 전송한다.

```cpp
#include <intelion_interfaces/msg/hal_command.hpp>
#include <rclcpp/rclcpp.hpp>

// publisher: rclcpp::QoS(1).best_effort().durability_volatile()
// active_session_id는 C launch session_id와 동일한 비영 세션.
intelion_interfaces::msg::HalCommand command;
command.header.frame_id = "map";
command.session_id = active_session_id;
command.command_seq = ++command_seq;
command.mode = intelion_interfaces::msg::HalCommand::MODE_POSITION;
command.target_pose = latest_smoothed_tcp_pose;
command.source_result_id = last_used_ai_seq;
command.created_mono_ns = latest_target_created_mono_ns;
command.expires_mono_ns = command.created_mono_ns + 5000000LL;
command_publisher->publish(command);  // B의 비실시간 통신 스레드에서 실행
```

latest_target_created_mono_ns는 B가 목표를 만든 시점의 CLOCK_MONOTONIC ns다.
벤치마크 t0 의미를 하나로 고정하고, 송신 시 시각을 갱신해 오래된 목표를 새것으로 만들지 않는다.
header.stamp를 채울 경우 시뮬레이션 시간을 쓰되 만료 계산과 분리한다.

## 검증 결과 (2026-10-05)

- ROS 2 Jazzy에서 intelion_interfaces/pac_hal colcon 빌드 통과.
- 설정·만료·세션·중복·역순·NaN·quaternion·범위 검사 24개 테스트 통과.
- ament_flake8 통과.
- 독립 ROS_DOMAIN_ID=143에서 FR3/UR5e 각각 런치 및 실제 토픽 송수신 시험 통과.
- 생성된 메시지 필드가 제공 문서와 정확히 일치함을 introspection으로 확인.
- 만료 오류 발생·해제, interface_only 정상 형식 명령 거부, 무효 상태 유지 확인.
- 수행하지 않은 t3~t6 로그 칸이 비어 있음 확인. 운동·IK·정지 검증은 해당 없음.

## Gazebo와 합성 B 실행 (2026-10-05 추가)

```bash
ros2 launch pac_hal hal_simulation.launch.py robot:=franka_fr3 session_id:=42
# 새 터미널, 같은 ROS_DOMAIN_ID 및 overlay
ros2 run pac_hal hal_demo_publisher --session-id 42 --output /tmp/hal-demo.json
```

기존 interface-only 예시에는 `backend:=interface_only`를 명시한다.
기본 설정은 상태 예상 10ms, stale 30ms, lost 100ms, IK 예산 500us다.
이는 데모 설정이며 실제 수신 주기/성능의 증명이 아니다. IK 예산 초과는 E_IK_FAILED로 거부한다.
합성 B는 6초 2mm 왕복 후 송신을 끝내고 측정 TCP 이동과 watchdog 정지를 검사한다.
실물에는 사용하지 않는다. 같은 command 토픽에 실제 B와 합성 B를 동시에 연결하지 않는다.
