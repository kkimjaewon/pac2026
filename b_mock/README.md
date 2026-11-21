# b_mock — 받은 pac_b_mock와 FR3 C++ HAL 연결

사용자가 제공한 `pac_b_mock.tar.gz`의 `b_mock`를 기존 PAC2026 workspace에 추가했다.
포함된 ControlCommand/HalCommand/HalState/HalError는 기존 계약과 필드·상수가 같다.
상수의 `=` 주변 공백만 다르므로 기존 `intelion_interfaces`를 그대로 사용한다.
C 내부 HalJointCommand/Status를 포함하는 기존 인터페이스 패키지도 유지한다.

이 노드는 `/hal/final_command`에 HalCommand를 발행하고 `/hal/state`, `/hal/error`를 읽는다.
실제 B 실시간 제어기와 별개의 **C 개발용 B 대역**이다. `pac_rt_control`은 변경하지 않았다.

## 실행 — FR3

현재 프로젝트의 helper를 쓰면 ROS Jazzy, overlay, ROS_DOMAIN_ID=42가 적용된다.

터미널 1, Gazebo/HAL이 꺼져 있을 때만:

```bash
cd /home/jaewon/Documents/ChatGPT/intellion
bash scripts/launch_hal_sim.sh session_id:=42
```

터미널 2, 받은 B 모의 패키지를 이용한 왕복:

```bash
cd /home/jaewon/Documents/ChatGPT/intellion
bash scripts/run_b_mock.sh --ros-args -p session_id:=42 -p motion:=sine
```

기본 `hold`는 현재 위치를 유지한다. `sine`은 현재 측정 TCP에서 y축 2mm를 6초 주기로 왕복한다.
발행은 Ctrl+C까지 계속된다. `-p duration_s:=6.0`을 더하면 6초 뒤 발행만 중단하고
C 상태 모니터링은 계속한다. B 발행이 멈추면 C 수신단이 만료/watchdog에 따라 감속 hold한다.

기존 `hal_demo_publisher`나 다른 B publisher를 함께 실행하지 않는다.
다른 B publisher가 감지되면 이 mock의 발행 스레드를 멈춘다.

직접 ROS 명령:

```bash
source /opt/ros/jazzy/setup.bash
source /home/jaewon/Documents/ChatGPT/intellion/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run b_mock fake_b_node --ros-args -p session_id:=42 -p motion:=sine
```

## FR3 연결에 맞춰 수정한 부분

| 항목 | 받은 원본 | 연결판 |
|---|---|---|
| 메시지 정의 | 같은 v0.3 4개 메시지 | 기존 공유 패키지 사용, 필드 변경 없음 |
| session | 시작 monotonic 값으로 임의 생성 | C launch와 같은 session_id 지정, 기본 42 |
| 처음 목표 | [0.50,0,0.20], 고정 orientation | 신선한 측정 TCP와 orientation을 출발점으로 사용 |
| sine/line | ±5cm/절대 line, 즉시 출발·반전 | 기본 2mm, 부드러운 출발·반전 |
| 재시작 seq | 1부터 시작 | 현재 C가 보고한 last_command_seq 뒤에서 시작 |
| header.stamp | B 기본 ROS clock | 실제 측정의 Gazebo stamp 복사 |
| expired | expires < created여서 E_FORMAT | created와 expires를 함께 과거로 옮겨 5ms TTL 형식 유지, E_EXPIRED 시험 |
| new_session | 임의 새 세션을 C가 받아들이기를 기대 | 미등록 세션 명령 1개를 보내 E_FORMAT 확인 후 등록된 세션으로 복귀 |
| 상태 없음/stale | 계속 명령 생성 | 신선한 READY/ACTIVE 측정과 session 일치 전 송신 대기 |

원본 고정 목표는 `reference_mode:=absolute`로 남겼다. 이 경우 큰 목표 점프는 C의 관절 속도/가속도
제약으로 거부될 수 있다. 목표로 이동했거나 정상 경로라는 뜻이 아니다. FR3 동작 확인에는
기본 `reference_mode:=measured`를 사용한다. 원본의 `install_b_mock.sh` 대신 기존 workspace에서
b_mock 패키지를 빌드하므로 새 workspace나 `.bashrc` 수정은 필요 없다.

## 파라미터

| 이름 | 기본 | 실행 중 변경 | 의미 |
|---|---|---|---|
| session_id | 42 | 아니오 | C와 같은 등록 세션 |
| rate_hz | 1000.0 | 아니오 | Python 스레드의 예정 발행률, 실제 RT 성능 보장 아님 |
| reference_mode | measured | 아니오 | 측정 기준 / 원본 absolute 좌표 |
| amplitude_m | 0.002 | 아니오 | measured 경로 왕복 거리, 최대 0.005m |
| motion_period_s | 6.0 | 아니오 | measured 왕복 주기, 최소 4초 |
| duration_s | 0.0 | 아니오 | 0은 계속 발행, 양수는 시작 경로 이후 제한 시간 |
| motion | hold | 예 | hold / sine / line |
| fault_mode | none | 예 | 아래 장애 입력 |

motion 변경은 현재 신선한 측정에서 다시 시작한다. duration 제한도 새 경로에서 다시 계산한다.
line은 현재 위치→y축 amplitude→현재 위치의 quintic 경로로 끝에서 속도·가속도가 0이다.
C 설정이 바뀌어 stale 기준이 달라지면 mock의 측정 신선도 30ms 기준도 함께 맞춰야 한다.
B 진짜 새 RUN은 C를 같은 새 session_id로 명시적으로 시작한다. C는 첫 명령으로 세션을 등록하지 않는다.

## 장애 입력

다른 터미널에서도 동일한 ROS 환경을 source해야 한다.

```bash
ros2 param set /fake_b_node fault_mode expired
ros2 param set /fake_b_node fault_mode none
ros2 param set /fake_b_node fault_mode replay
ros2 param set /fake_b_node fault_mode none
ros2 param set /fake_b_node fault_mode invalid_number
ros2 param set /fake_b_node fault_mode none
ros2 param set /fake_b_node fault_mode unreachable
ros2 param set /fake_b_node fault_mode none
ros2 param set /fake_b_node fault_mode new_session
ros2 param set /fake_b_node fault_mode stop
```

| 모드 | 기대 C 반응 |
|---|---|
| expired | E_EXPIRED=3 / CMD_REJECTED |
| replay | 이미 보낸 seq를 새 TTL로 재발행, E_FORMAT=1 |
| invalid_number | NaN 위치, E_FORMAT=1 |
| unreachable | [2,0,.2], 현재 workspace 밖이므로 E_OUT_OF_RANGE=4 |
| new_session | 미등록 session, E_FORMAT=1. C active_session은 기존 값 유지 |
| stop | 발행 중단, C 수신단 감속 hold |

각 지속 장애는 `fault_mode none`으로 복구한다. new_session은 자동 none 복귀한다.
원본 `nan` 이름도 지원하지만 ROS YAML이 숫자로 해석할 수 있어 `invalid_number`를 권장한다.
오류는 발생/해제 전환 이벤트이므로 계속 잘못된 명령을 보내도 매번 같은 이벤트가 오지 않는다.

## 빌드·검증

```bash
cd /home/jaewon/Documents/ChatGPT/intellion/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select b_mock
source install/setup.bash
```

예정 1000Hz와 로그의 실제 발행률/max_gap은 구분한다. Python 스케줄링과 DDS 지연이 있으며
발행 로그는 C의 전체 1kHz RT 성능/명령 도달을 입증하지 않는다.
C의 CMD_SENT, 수신단 적용 ACK, 실제 TCP 이동을 별도로 검사한다.
