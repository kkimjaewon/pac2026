# pac_rt_control

B(홍연주) | 실시간 제어 계층, rt_control_node (C++17), rt_config.yaml

> 실제 C(FR3 HAL) 연결은 `config/fr3_integration.yaml` + `session_id:=42`로 실행하고 `fake_c`는 켜지 않는다.
> 실행법·수정 이유·검증 결과는 `jaewon` 브랜치의 `PAC2026/docs/real-b-fr3-integration.md`.

---

## 개요

ROS 2 패키지 이름은 **`rt_control`** 입니다 (폴더 이름은 `pac_rt_control`).
A가 보내는 `/ai/control_command`(ControlCommand, 약 15Hz)를 받아 gate·blend·limiter·safety를 거친 뒤,
1ms 실시간 루프에서 만든 최종 명령을 C에게 `/hal/final_command`(HalCommand, 1kHz)로 보냅니다.
C의 `/hal/state`, `/hal/error`를 받아 시작 위치와 안전 모드(RUN / HOLD / DECEL_STOP / FAULT)에 씁니다.

| 방향 | 토픽·서비스 | 타입 |
|---|---|---|
| A → B | `/ai/control_command` | `intelion_interfaces/msg/ControlCommand` |
| B → C | `/hal/final_command` | `intelion_interfaces/msg/HalCommand` |
| C → B | `/hal/state`, `/hal/error` | `intelion_interfaces/msg/HalState`, `HalError` |
| 서비스 | `/rt/reset_fault` | `std_srvs/srv/Trigger` (FAULT 해제) |

## 빌드

`intelion_interfaces`가 같은 워크스페이스(`PAC2026/src/intelion_interfaces`)에 있어야 합니다.
`/opt/ros/jazzy`만 불러온 셸에서 아래처럼 빌드·테스트됩니다.

```bash
cd PAC2026
source /opt/ros/jazzy/setup.bash
colcon build --packages-select intelion_interfaces rt_control b_mock
colcon test --packages-select rt_control && colcon test-result --verbose
source install/setup.bash
```

## 실행 방법

```bash
# B만 (pose_valid HalState가 올 때까지 INIT에서 대기. 예전처럼 2초 뒤 start_position에서 시작하려면
#       yaml에서 start_fallback_on_timeout: true. 그때 require_c_state 기본 true라 DECEL_STOP)
ros2 launch rt_control rt_control.launch.py

# 실제 C(FR3 HAL, 세션 42 등록)와 연결: 등록 세션의 신선한 READY/ACTIVE 상태를 기다렸다가 시작
ros2 launch rt_control rt_control.launch.py \
  config:="$(ros2 pkg prefix --share rt_control)/config/fr3_integration.yaml" session_id:=42

# 가짜 C + B (켜는 순서: 가짜 C -> 가짜 A -> B 는 launch가 지연을 두고 맞춰 줌)
ros2 launch rt_control rt_control.launch.py fake_c:=true run_tag:=test

# 가짜 A까지 (ai_mock 패키지가 source 되어 있어야 함. 없으면 경고만 하고 A는 안 켬)
ros2 launch rt_control rt_control.launch.py fake_a:=true fake_c:=true

# 계획 위치·설정 파일 바꾸기
ros2 launch rt_control rt_control.launch.py planned_position:="[0.5, 0.0, 0.70]"
ros2 launch rt_control rt_control.launch.py config:=/다른/경로.yaml

# 따로 실행
ros2 run rt_control fake_c_node.py
ros2 run rt_control rt_control_node --ros-args -p fifo:=false -p cpu:=-1 -p mlock:=false
ros2 run rt_control rt_loop_demo --seconds 30 --mlock --fifo --prio 80 --cpu 6   # 지터 측정만

# 가짜 C 오류 주입 / FAULT 해제
ros2 param set /fake_c_node fault_mode ik_fail      # ik_fail | state_stop | disconnect | none
ros2 service call /rt/reset_fault std_srvs/srv/Trigger
```

주의
- `config/rt_config.yaml`은 시연 PC 기준으로 `fifo: true`, `cpu: 6`, `mlock: true` 입니다.
  다른 PC에서는 CPU 개수가 다르거나 rtprio 권한(`ulimit -r`)이 없을 수 있으니, 시작 로그의 경고를 확인하고
  필요하면 위처럼 `-p fifo:=false -p cpu:=-1 -p mlock:=false` 로 끄고 돌리세요(지터는 커짐).
- 실행 기록은 `~/pac_logs/<run_id>/` 에 `rt.csv`, `ai_rx.csv`, `meta.yaml` 로 남습니다 (`trace_enable`, `log_root`).
  `run_id`를 비우면 시작 시각(+`_run_tag`)으로 새 폴더를 만들고, `run_id:=exp01`처럼 주면 `~/pac_logs/exp01/`을 그대로 씁니다
  (폴더가 이미 있어도 되지만, 안에 `rt.csv`가 있으면 덮어쓰지 않고 오류로 멈춤).
- 그래프: `python3 tools/plot_rt.py ~/pac_logs/<run_id>` (python3-matplotlib, python3-pandas 필요. 설치 안 되는 개발용 도구).
- 가짜 A(`ai_mock`)는 이 저장소에 아직 없습니다. A 쪽 패키지를 따로 빌드·source 해야 `fake_a:=true`가 동작합니다.

## 주요 파라미터

전체 목록과 기본값은 `config/rt_config.yaml`(값마다 뜻·단위·근거 주석)과 `src/rt_control_node.cpp` 맨 위 주석에 있습니다.

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `j_max` | `1.0` | 출력 저크 한계 [m/s³]. 가속도가 한 주기(1ms)에 `j_max`×1ms 이하로만 바뀜 (S자 출발·정지). 목표 근처 감속도 저크·가속도 한계 안에서 미리 시작해 지나치지 않고, 마지막 3주기는 정확히 목표·속도 0·가속도 0에 닿음. DECEL_STOP 감속도 같은 한계. `0` = 끔 (예전 계산 그대로, 가속도가 0 → `a_max`로 한 번에 바뀜) |
| `blend_deadband` | `0.0005` | 목표 불감대 [m]. 새 AI 명령을 반영하면 blend 목표가 움직일 거리(벡터 크기)가 이보다 작으면 목표를 바꾸지 않음(반영 안 함으로 기록). 도착 뒤 같은 곳을 가리키는 명령이 목표를 계속 미세하게 흔들지 않게. `0` = 끔 |
| `arrive_tol` | `0.002` | 도착 판정 허용 오차 [m] (C와 합의). \|blend 목표 − 측정 TCP(HalState)\| |
| `arrive_hold_s` | `0.3` | 위 오차가 `arrive_tol` 이하로 이만큼 이상 이어지면 "도착" [s] (C와 합의). rt.csv `arrived`, 1초 요약 `도착 예/아니오` |
| `resume_from_last_cmd` | `true` | DECEL_STOP·FAULT → RUN 재개 때, 마지막 발행 HalCommand 위치와 측정 TCP 차이가 `resume_match_tol` 이하면 마지막 발행 위치에서 limiter 재시작. 크면 측정 위치에서 시작하고 경고. `false`면 예전처럼 항상 측정 위치 (C 답 대기) |
| `resume_match_tol` | `0.002` | 위 재개 기준의 허용 오차 [m] |
| `target_offset` | `[0.0, 0.0, 0.0]` (fr3_integration.yaml은 `[0.0, 0.0, 0.10]`) | 철판 표면 위 작업 높이 [m], map 기준. 임시값, 팀 합의(C의 IK 확인) 후 확정. A가 보낸 위치(철판 표면)에 더해 TCP 목표로 씀. gate(계획 위치·허용 반경)는 더하기 전 원래 값으로 검사하고, 시작 위치·AI 없을 때 목표·HOLD 중 목표·start_position에는 안 더함. NaN·무한대 성분은 0으로 보고 경고. launch 인자 `target_offset:="[0.0, 0.0, 0.10]"`로 덮어쓸 수 있음 |

도착 후 고정: limiter는 목표에 속도 0으로 도착하면 목표가 바뀔 때까지 출력 위치를 비트 단위로 그대로 냅니다
(계산을 반복하지 않음). 감속은 `a_max`·`j_max` 안에서 하고, 위치를 갑자기 맞추거나 속도를 강제로 0으로 만들지 않습니다.

## rt.csv 열

계약 v0.3 필수 열(이름·순서 그대로) 뒤에 B 추가 열이 붙습니다. 추가 열은 항상 끝에만 덧붙입니다.

| 열 | 설명 |
|---|---|
| `cycle` ~ `reject_reason` | 계약 v0.3 필수 열 (cycle, wake_mono_ns, compute_ns, overrun, mode, used_ai_seq, weight, goal_x/y/z, y_x/y/z, command_seq, ai_recv_mono_ns, ai_age_ns, ai_apply_mono_ns, reject_reason) |
| `jitter_ns` | 깨어난 시각 − 예정 시각 [ns] |
| `mode_reason` | safety 모드에 들어온 이유 (`EXPIRED\|RANGE`처럼 `\|`로 이음, 없으면 빈 칸) |
| `speed` | limiter 속도 크기 [m/s] |
| `has_used` | blend가 한 번이라도 반영했는지 (0/1). seq 0 반영과 "반영 없음" 구분용 |
| `arrived` | 도착 판정 (0/1). \|goal − 측정 TCP\| ≤ `arrive_tol`이 `arrive_hold_s` 이상 |
| `tcp_error_m` | \|goal(blend 목표) − 측정 TCP\| [m]. 믿을 수 있는 측정이 없거나 INIT이면 `nan` |
| `used_ai_boot_id` | blend가 반영한 A 명령의 `writer_boot_id` (`has_used`가 0이면 0). `used_ai_seq`와 함께 A 로그와 합칠 때 씀 (AB-07, merge_logs.py) |

## 파일 구성

```
pac_rt_control/
├── CMakeLists.txt, package.xml      # 패키지 rt_control (ament_cmake, C++17)
├── config/rt_config.yaml            # rt_control_node 파라미터 (값마다 [확정]/[제안]/[미정] 표시)
├── launch/rt_control.launch.py      # B (+ 선택: 가짜 C, 가짜 A) 실행
├── include/rt_control/, src/
│   ├── rt_control_loop.*            # 1ms 루프 (clock_nanosleep ABSTIME, FIFO·CPU 고정·mlock)
│   ├── ai_command_receiver.*        # A 명령 구독 -> seqlock 스냅샷
│   ├── gate.*                       # TTL·신뢰도·레이아웃·accept_radius 검사
│   ├── blend_policy.*               # 통과한 명령 반영 방식 (single)
│   ├── motion_limiter.*             # v_max·a_max·j_max 제한, 도착 후 고정
│   ├── arrival.*                    # 도착 판정(arrive_tol·arrive_hold_s), 재개 위치 선택
│   ├── safety_mode.*                # RUN / HOLD / DECEL_STOP / FAULT 상태 기계
│   ├── ros_bridge.*                 # B -> C HalCommand 발행 스레드, HalState·HalError 구독
│   ├── cycle_tracer.*               # 매 주기 rt.csv 기록 (spsc_ring)
│   ├── env_check.*                  # RT 커널·전원 프로필·rtprio 권한 점검
│   ├── seqlock.hpp, spsc_ring.hpp, *_snapshot.hpp
│   ├── rt_control_node.cpp          # B 노드 본체
│   └── rt_loop_demo.cpp             # 지터 측정용 단독 실행 파일
├── scripts/fake_c_node.py           # 가짜 C (1차 지연 로봇 흉내, 오류 주입)
├── tests/                           # gtest 9개 (colcon test)
├── tools/plot_rt.py                 # rt.csv 그래프·요약
└── docs/
    ├── b_layer_reference.md         # B 계층 설계·계약 참고
    └── measurements.md              # 측정 기록 전체
```

## 측정 결과 요약

자세한 조건과 표는 [docs/measurements.md](docs/measurements.md). 목표: 주기 1ms, 지터 ±100us.
측정 PC: LG gram 16Z90SP, RT 커널 6.8.1-1061-realtime.

| 측정 | 최대 지터 | 100us 초과 | overrun |
|---|---|---|---|
| `rt_loop_demo` 옵션 없음 (RT 커널, 30초) | 3835.9 us | 98.6% | 3 |
| `rt_loop_demo` FIFO80 + mlock + CPU 2 | 150.5 us | 0.03% | 0 |
| `rt_loop_demo` FIFO80 + CPU 6 + 성능모드 | 36.4 us | 0 | 0 |
| 일반 커널 (6.8.0-generic), mlock + CPU 2 (참고용, 3초) | 351 us | 0.97% | - |
| 가짜 A → B → 가짜 C, cycle_tracer 켬 (약 50초, 49,783주기) | 35.4 us (99.9% 13.3 us) | 0 | 0 |
| 발표용 기록 (44,764주기) | 46.6 us (99.9% 17.7 us) | 0 | 0 |

- RT 커널 + SCHED_FIFO + 전용 CPU + 성능 모드 조합에서 목표를 만족. 일반 커널이나 일반 스케줄링으로는 못 맞춤.
- `/hal/final_command` 997~1000Hz, 1ms당 최대 이동 0.1mm(= v_max), HOLD 동안 출력 변화 0.
- safety 시험: A stop/bad_value → HOLD → 복귀, C ik_fail/state_stop → DECEL_STOP → 복귀, C disconnect → FAULT → `/rt/reset_fault`로 복귀 모두 확인.
- 남은 문제: 시작 직후 발행 스레드가 약 333ms 멈추는 일이 간헐적으로 있음 (Fast DDS 공유메모리 전송 관련 추정,
  `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`면 재현 안 됨). 1ms 루프에는 영향 없음.
