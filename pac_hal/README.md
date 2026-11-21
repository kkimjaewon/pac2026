# pac_hal

C(김재원) 담당. v0.3 B/C 계약을 유지하는 Gazebo 전용 HAL과 합성 B 테스트 노드.

2026-10-07: **FR3와 UR5e의 기본 Gazebo 실행은 같은 C++ `hal_node`를 사용한다.**
`robot_config.cpp`가 프로파일을 검사하고 `HalConverter`는 공식 모델의 6/7축 IK/FK를,
`GazeboPositionAdapter`는 두 시뮬레이터의 동일 ros2_control 전송을 담당한다.
상위 B–C 메시지와 A/B 소스는 변경하지 않는다. 실물용 제조사 전송은 별도 어댑터가 필요하다.
UR TCP는 공식 `tool0` 축에 맞춰 고정 회전을 적용한다. 도구가 없어도 flange 축과 TCP 축은 같지 않을 수 있다.
시나리오 4 실행·검증·한계: [공통 HAL 검증](../../docs/scenario4-common-hal-2026-10-07.md).
요청한 파일별 진행표와 자세한 설계 이유: [FR3 HAL 파일 설명](../../docs/fr3-hal-file-guide.md).
A→B 장애 시험용 `fake_ai_node.py`도 추가했다. 이 노드는 C 명령 publisher가 아니며 B가 필요하다.

`HalCommand → 세션/순번/5ms TTL → map/base/TCP 변환 → IK → 관절 제한 →
C++ watchdog position controller → Gazebo`. `/joint_states`의 실제 관절 측정으로
FK/TCP 상태를 계산한다. 지원 기능·작업 요구·실제 모델·충돌 장면의 사전 검사를 통과하고 측정과 수신단 상태가 신선할 때만 READY/ACTIVE를 보고한다.

## 팀 셀에서 로봇 교체 (시나리오 4, A·B 포함)

팀 월드(카메라·철판·책상)는 그대로 두고 로봇만 바꾼다. 두 로봇 모두 **session_id 42**,
B는 같은 `demo/configs/fr3_demo.yaml`을 쓴다(B 코드·설정 변경 없음). 교체 때 B·A·Gazebo/C를
모두 끄고 다시 켠다. Gazebo를 다시 켜면 카메라 시각이 0부터 시작하므로 A도 다시 켠다.

```bash
ros2 launch pac_bringup sim.launch.py robot:=franka_fr3 gui:=false session_id:=42
# 종료 후
ros2 launch pac_bringup sim.launch.py robot:=ur5e gui:=false session_id:=42
# 종료 후 (HD현대로보틱스 HA006B, 6축 산업용)
ros2 launch pac_bringup sim.launch.py robot:=ha006b gui:=false session_id:=42
```

HA006B는 C++ 변경 없이 `generic_gazebo` 경로와 설정 2개(`ha006b_team.yaml`, `ha006b_team_scene.yaml`)로
들어온다. 바닥 x=-0.4에 설치하고 대기 TCP는 FR3와 같은 (0.10,0,0.83) 공구 아래다. 공식 SRDF가 없어
자기충돌 제외는 URDF 부모·자식 링크 + base_link/motor_s 1쌍(j1만으로 움직이는 관계, j1 전 범위 최소 1.42mm)이다.
`robot:=ha006b`에서만 공식 모델 `hdr_description`이 필요하다(FR3·UR5e 실행에는 필요 없음):

```bash
# 팀 저장소 밖 별도 workspace (팀 저장소에 섞여 커밋되지 않게)
mkdir -p ~/repos/hdr_ws/src && cd ~/repos/hdr_ws/src
git clone https://github.com/hyundai-robotics/hdr_description.git
git -C hdr_description checkout 8512ee3a110e9e75967d3e1fda8477d0ba6fcd05   # 검증한 버전
cd ~/repos/hdr_ws && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source ~/repos/hdr_ws/install/setup.bash   # 팀 workspace setup 뒤에 추가로 source
```

이 저장소는 시뮬레이션 모델(URDF·메쉬)까지만 쓴다. 실제 Hi6 제어기 연결은 하지 않았다.

장면은 `demo/scripts/scene_driver.py`(노드 하나가 철판 이동·A 고장 주입을 직접 호출)로 돌린다.
아래 공통 교체 셀(session 43 예시)은 C 단독 시험용이며 팀 셀과 다른 실행이다.

## 같은 작업점에서 로봇 교체 (C 단독 공통 셀)

프로젝트 루트에서 서로 다른 실행으로 사용한다. 기존 B/Gazebo/C를 종료한 뒤 로봇을 바꾸고,
B에도 C와 같은 새 session_id를 적용한다. FAULT 해제는 같은 로봇의 세션/순번을 유지한다.

```bash
bash scripts/hal_ros_exec.sh ros2 launch pac_hal common_workcell.launch.py robot:=franka_fr3 session_id:=42
bash scripts/hal_ros_exec.sh ros2 launch pac_hal common_workcell.launch.py robot:=ur5e session_id:=43
```

두 명령을 동시에 실행하지 않는다. 공통 작업점 `(0.35,0,0.81)`, 아래보기 TCP 자세,
FR3 바닥 설치/UR5e 30cm 받침대 설치를 사용한다. 시작 관절값은 시뮬레이션 초기화 전용이다.
C와 시뮬레이터가 같은 프로파일의 설치 변환을 사용하고 시작 때 장면/초기 자세를 검사한다.
IK 계산량은 기본으로 프로필 `max_iterations`(현재 30회)로 제한한다(`ik_budget_clock:=iterations`). 명령 wall TTL 5ms 만료 검사는 항상 유지하고, 늦어진 IK의 wall/CPU 시간은 hal.csv에 기록한다. 시간 예산 비교는 `ik_budget_clock:=wall` 또는 `thread_cpu`(500us)로 실행한다.

## 일반 실행

ROS 2 Jazzy 환경과 이 workspace의 install/setup.bash를 source한 터미널에서:

```bash
ros2 launch pac_hal hal_simulation.launch.py robot:=franka_fr3 session_id:=42
```

작업대·물체·카메라는 팀 `pac_worlds/worlds/franka_world.sdf`를 사용한다.
Franka는 공식 xacro의 `gazebo_effort:=false`로 생성해 position 제어를 사용한다.
기본 gravity 예제와 별도로 이 launch를 사용한다. GUI는 `gui:=false`로 생략할 수 있다.
다른 터미널에서 합성 B를 실행한다:

```bash
ros2 run pac_hal hal_demo_publisher --session-id 42 --output /tmp/hal-demo.json
```

현재 신선한 TCP에서 z축 2mm를 6초 동안 왕복하고 송신을 끝낸다.
실측 이동량과 수신단 watchdog 정지를 JSON으로 검사한다. 다른 B publisher가 있으면 거부한다.
시간 제한 초과 명령은 거부되며 Python 송신은 1kHz RT 성능 측정용이 아니다.

UR 교체는 이전 Gazebo/HAL을 종료한 뒤 새 session으로 실행한다:

```bash
ros2 launch pac_hal hal_simulation.launch.py robot:=ur5e session_id:=43
ros2 run pac_hal hal_demo_publisher --session-id 43
```

기존 Gazebo에 C만 연결하려면 `hal_system.launch.py robot:=... session_id:=... run_id:=... collision_scene_file:=<실제장면.yaml>`을 사용한다. 실행용 충돌 장면은 필수다.
C의 설정 노드가 로봇 설명을 검사하고 기존 관절 제어기를 HAL position controller로 전환한다.
새 빌드 후에는 Gazebo도 재시작해야 새 C++ plugin을 읽는다.
인터페이스 검사만 하려면 `backend:=interface_only`를 지정한다. 이 모드는 운동을 거부한다.

오류 경로도 검사하려면 합성 B 송신이 끝난 뒤 실행한다:

```bash
ros2 run pac_hal hal_fault_probe --session-id 42 --output /tmp/hal-faults.json
```

만료·중복·작업영역 밖·도달 불가 목표의 거부 코드와 CMD_REJECTED를 검사한다.
도달 불가 목표는 IK 실패 또는 IK 시간 제한 초과로 E_IK_FAILED를 반환할 수 있다.

## 실제 B 연동

팀 B 구현(`rt_control`)을 가져와 FR3 C 연결을 검증했다.
실행 profile, 수정한 세션·복구 정책, 시험 결과는
[실제 B와 FR3 연결](../../docs/real-b-fr3-integration.md)에 있다.
실제 B를 사용할 때 `b_mock` 또는 `hal_demo_publisher`를 함께 실행하지 않는다.

## B 연결 시 필요한 값

### 받은 b_mock으로 FR3 시험

팀원이 제공한 B 모의 패키지는 `src/b_mock`에 포함되어 있다.
기존 ROS 2 Jazzy/Franka 시뮬레이션 의존성이 설치된 workspace에서 다음을 실행한다.
아래 `<workspace>`는 각자 사용하는 ROS workspace의 절대 경로로 바꾼다.

```bash
cd <workspace>
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to pac_hal b_mock
source install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch pac_hal hal_simulation.launch.py robot:=franka_fr3 session_id:=42
```

다른 터미널에서 같은 workspace를 source하고 B를 실행한다:

```bash
source /opt/ros/jazzy/setup.bash
source <workspace>/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run b_mock fake_b_node --ros-args -p session_id:=42 -p motion:=sine
```

현재 측정 TCP에서 y축 2mm를 6초 주기로 왕복한다. `hal_demo_publisher`와 동시에 실행하지 않는다.
각 터미널의 Ctrl+C로 B와 시뮬레이션을 종료한다.
상세 설정은 [b_mock README](../b_mock/README.md), 검증 결과는
[B/C 연결 결과](../../docs/b-mock-integration.md)를 참고한다.

### 실제 B 제어기 계약

- `/hal/final_command`: HalCommand, BestEffort/Volatile/depth1.
- `/hal/state`: HalState, 같은 QoS. 신선한 pose_valid 및 READY/ACTIVE 확인.
- `/hal/error`: HalError, Reliable/Volatile/depth32, 오류 발생·해제 이벤트.
- 같은 PC의 CLOCK_MONOTONIC, 같은 nonzero session_id, 증가하는 command_seq.
- created_mono_ns는 목표 생성 시각, expires는 created+5ms. 송신할 때 TTL을 새로 연장하지 않는다.
- 절대 map TCP pose, m/rad, quaternion xyzw. 위치·관절 제약을 고려한 매끄러운 경로.

B 완성 후 합성 B를 종료하고 실제 B가 동일한 토픽에 발행하면 된다.
설정 `config/franka_fr3.yaml`, `ur5e.yaml`은 시작 시 고정된다.
기본 base/TCP 변환은 원점 설치·도구 없음 예시다. 실제 설치/도구 변환과 충돌 영역을 확정해야 한다.

## 확인과 한계

공식 FR3/UR 모델로 FK/IK, 좌표 변환, Jacobian, 도달 불가/시간 제한, 설정 검사 테스트를 실행한다.
HalJointCommand/Status는 **C 내부** 채널이다. 적용 ACK는 수신단이 목표를 채택했다는 뜻이며
TCP 도달/공작 완료를 의미하지 않는다. 실측 TCP 이동을 따로 검사한다.
5ms 만료 시 수신단이 감속 후 hold하고, 무통신 watchdog도 독립 적용한다.
속도/가속도 제한은 데모용이다. 충돌 장면을 지정한 launch는 모델 기반 경로 검사를 사용하며,
실물 설치·도구·케이블 및 움직이는 물체의 런타임 추적은 별도 검증이 필요하다.

CSV는 sent/rejected와 일부 applied ACK 행을 session/seq로 연결한다.
t1~t4는 C 수신/검증/IK/전송 전, t6는 C 내부 수신단 ACK 수신이다.
t5는 상관된 측정 ACK가 없어 비어 있다. 100Hz ACK가 모든 명령을 기록하지는 않는다.
1kHz±100us, 실물 Franka/UR 전송, 용접 공정은 별도 구현·검증 항목이다.

상세 계약: [hal-interface-v03.md](../../docs/hal-interface-v03.md).

## BC-07·08·09 팀 월드 통합

최신 `pac_worlds`와 `pac_hal`을 빌드·source한 환경에서:

```bash
ros2 launch pac_hal team_workcell.launch.py gui:=false session_id:=42 run_id:=새로운_실행ID
```

팀 `franka_world.sdf` 하나에 FR3·C 위치 컨트롤러·카메라 브리지·TF를 실행한다.
시뮬레이션 초기 TCP는 (0.10, 0, 0.83)m, 아래 방향으로 접힌 자세다.
카메라 설정은 월드에서 읽으며 A의 `/depth_camera/image_raw`,
`/depth_camera/depth/image_raw`, `/depth_camera/camera_info`에 연결한다.
팀 `sim.launch.py`를 동시에 켜지 않는다. A/B 노드는 별도로 실행한다.
월드와 충돌 장면의 책상·철판·카메라 값이 다르면 시작을 거부한다.

기본 로그는 `~/pac_logs/<run_id>/hal.csv`. 기존 열 뒤에 AI 결과 번호, B 생성·C 수신,
사전 검증·IK 종료·전송 시작/종료·수신단 목표 채택·ACK 수신 시각을 추가했다.
`validated_mono_ns`는 IK 전 검사 종료, `t3_mono_ns`는 IK 이후 관절·충돌 검사 종료다.
목표 채택 시각은 TCP 도착 시각과 구분하며, 100Hz 상태에서 관측하지 못한 적용 시각은 공란이다.
상세 실행·검증·팀 답변: [BC-07·08·09 보고서](../../docs/bc-07-09-team-integration-2026-10-07.md).

## 검증된 장치 기능에 맞춘 실행 계약

확장 목표는 어떤 장치든 검증된 조건에서만 안전하게 실행하는 메커니즘이다.
프로필 `execution`에 지원 모드/상태 피드백/독립 watchdog/정지 방식과 전송 주기를, `task`에 작업 모드/최소 축 수/허용 주기/대표 TCP 목표를 명시한다.
`/hal/preflight` 결과와 측정 상태를 확인하고, 미지원 구성은 READY 전에 거부한다.
`backend_period_ns`가 입력 주기보다 느리면 최신 명령 하나만 보관하며 TTL을 연장하지 않는다. 느린 주기를 허용한 작업 요구 파일이 필요하다.
현재 검증 범위는 6/7축 position Gazebo이며 실물·힘/속도/공정 모드·E_STOP는 별도 구현과 검증이 필요하다.
설계 비교·실패 보완·실제 시험 기록: [HAL 확장성 계약](../../docs/hal-extension-contract-2026-10-07.md).

## A·B·C 지연 로그 병합

hal.csv의 sent/applied는 동일 명령의 단계 행이다. run_id+session_id+command_seq로 한 행으로 합치고 rejected·superseded는 따로 집계한다. source_result_id는 B가 반영한 A 메시지 seq이며, A 재시작은 B ai_rx.csv의 writer_boot_id와 실제 반영 행으로 구분한다.
로그 종료/flush 후 `tools/prepare_hal_log.py`를 실행하거나 `pac_hal.log_contract.normalize`를 병합 도구에서 참고할 수 있다.
단계·시계·누락 처리 및 검증 결과: [AC-01 로그 병합 계약](../../docs/ac-01-hal-log-contract-2026-10-08.md).

10/08 A 답변 대조: 최신 B의 `used_ai_boot_id`가 있는 동일 run 로그는 `prepare_hal_log.py`에 `--rt-csv <rt.csv> --ai-csv <A의 실제 CSV 경로> --session-id 42`를 함께 주어 실제 사용 boot+seq로 연결할 수 있다. A CSV의 기존 `boot_id` 헤더도 지원한다. stop 행(`published=0`)은 제외하고 미연결은 `ai_join`으로 표시한다. 구형 B 로그는 AC-01 문서의 정확한 ai_rx/apply 복원을 사용한다. 다른 PC의 monotonic 시각은 차감하지 않는다.

10/08 순번 보호 보완: 현재 세션의 모든 양수 명령 시도는 형식·범위·시간 검사 전에 번호를 소비한다. 거부된 명령을 같은 번호로 수정·재전송하지 말고 새 command_seq를 사용한다. 다른 세션·과거 번호는 현재 HalState 진행 watermark를 변경하지 않는다. interface_only 검증기도 같은 순번 규칙을 적용하며 READY/전송을 주장하지 않는다.

`trace_stats.yaml`은 고정 큐 dropped 외에 sent/applied/rejected/superseded와 개별 ACK를 확인하지 못한 경우를 구분한다. `older_sent_without_individual_ack`는 100Hz 수신 상태가 최신 번호를 보고하면서 지나간 송신 기록 수이며, 각 목표의 채택 여부를 확인했다는 뜻이 아니다. ACK가 없는 송신은 목표 채택 지연 표본에 포함하지 않는다. pending eviction·종료 시 미확인·미연결 ACK·잘못된 ACK 시각도 별도 집계한다.

수신단 `code10/DEVICE` 원인 확인용 `/hal_position_controller/diagnostics`(std_msgs/String, best effort)을 추가했다. 수신 패킷 검사와 같은 함수에서 session/count/seq0/clock/TTL/expired/nonfinite/range 거부 사유 및 패킷 seq·시각·축 번호를 게시한다. 기존 HalJointStatus/HalError 계약과 정지 동작은 유지한다. JSON 작성은 비실시간 수신 callback에서만 한다. 이 토픽은 수신 패킷 거부 진단이며 RT update의 관절 경계 clamp 원인까지 구분하는 진단으로 확대하지 않는다.

로그를 켜면 같은 `<log_directory>/<run_id>/`에 `hal_errors.csv`(C 오류 전환)와 `receiver_diagnostics.csv`(수신 패킷 거부 JSON payload)도 저장한다. 기록은 기존 고정 크기 큐→별도 writer를 사용하며 제어 callback에서 파일 I/O를 하지 않는다. `detail_truncated`/`payload_truncated=1`인 행은 원문 전체가 아니므로 JSON 파싱·정확한 원인 근거에서 제외한다. `trace_stats.yaml`의 diagnostic/error 기록 수·잘림·큐 누락을 함께 확인한다. 기존 `hal.csv` 열은 유지했다. HalError의 code10 seq는 당시 C command context이고, 수신 패킷의 정확한 session/seq는 diagnostic payload를 확인한다.

실제 관절 측정은 이름/배열의 완전성·중복·유한값·모델/프로필 위치 제한·알려진 속도 제한을 검사한다. 무효 측정은 DEVICE/FAULT와 pose_valid=false를 발생시키며 상태 순번·신선도 시각을 갱신하지 않는다. ACK나 역순/중복 측정으로 해제되지 않고 신선한 정상 측정으로 복구한다.

실행 모델은 초기화한 최초 robot_description에 고정한다. 동일 XML 반복은 허용하지만 이후 다른 XML은 TF/INCOMPATIBLE/FAULT 및 pose_invalid로 유지한다. 원래 XML을 보내도 자동 해제하지 않으며 새 실행에서 모델·충돌·작업 preflight를 다시 수행한다. 초기화가 부분적으로 성공한 경우에도 다른 XML과 모델 구성요소를 섞지 않는다. 이는 소프트웨어 설명 일치 보호이며 실물 교정 인증은 아니다. /hal/preflight에 run_id와 model_change_policy도 게시한다.

동일 실행의 검증을 반복할 때는 `--report-name trial_02.json`처럼 새 파일명을 지정한다. 기존 결과 덮어쓰기는 거부하며 report의 run_id는 실제 C preflight의 run_id와 같아야 한다.

모델 준비와 실행은 같은 URDF snapshot에 연결된다. `prepare_hal_controller`가 controller 변경 전에 검증 XML을 저장하고, `hal_system.launch.py`가 `expected_model_file`로 전달한다. 파일이 없거나 최초/이후 robot_description이 다르면 실행을 거부한다. 직접 hal_node를 수동으로 켜는 대신 표준 launch를 사용한다. 모델 초기화 실패는 preflight INCOMPATIBLE/오류와 FAULT로 알린다.

`/hal/preflight.robot_contract`는 기준/base/flange/TCP 프로필 이름, 설치/TCP 변환, 선언한 기능과 TCP 속도·가속도 한도를 게시한다. TCP 이름은 TF broadcaster의 존재 증거가 아니다. `workspace.max_tcp_speed_m_s`/`max_tcp_acceleration_m_s2`는 map XYZ 벡터 norm의 명령 목표 제한에 적용하며, 알려진 실제 FK TCP 속도도 검사한다. 기본 값은 0.1m/s·0.2m/s²이다. B는 더 낮거나 같은 한도를 사용해야 한다. 이 이산 목표 검사·초과 감지는 연속 물리 속도/정지 거리 보증이나 안전 인증을 뜻하지 않는다.

공개 probe는 DDS subscriber 연결만 기다리는 데서 그치지 않고 신선한 측정 자세의 정적 명령이 C에서 전송되고 receiver ACK로 목표 채택된 것을 확인한 뒤 경로를 시작한다. 고정 대기 시간만으로 준비를 판정하지 않는다. startup 신규 오류도 시험 실패로 기록한다.
