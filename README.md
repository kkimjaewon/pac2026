# PAC 2026 — AI 기반 실시간 로봇 제어 (HD현대로보틱스 미션)

> 깊이 카메라 AI 추론 결과를 **1ms 실시간 제어 루프**에 안전하게 반영하고, **제조사에 종속되지 않는 HAL**로 로봇(또는 Gazebo 시뮬레이터)을 구동하는 3계층 시스템.

---

## 1. 프로젝트 개요

| 계층 | 담당 | 한 줄 책임 |
| --- | --- | --- |
| **A. AI 추론** | 김나현 | 깊이 카메라 데이터 → SensorFrame → 시나리오별 판단 → ControlCommand를 공유 메모리에 기록 |
| **B. 실시간 제어** | 홍연주 | 1ms 고정 주기로 ControlCommand 검증·가중 반영 → 속도·가속도 제한 → HalCommand 출력 |
| **C. 통신·HAL** | 김재원 | HalCommand를 로봇별 신호로 변환·전달, 상태·오류를 B에 반환, 지연 측정 |

### 시연 시나리오
- **시나리오 1 — 위치 보정**: 대상물의 3차원 위치를 검출해 로봇 목표 위치를 보정
- **시나리오 3 — 공정 상태 판정**: 기준 경로 대비 편차를 판정해 경로 보정량으로 반영
- **시나리오 4 — HAL 확장성**: 서로 다른 로봇을 같은 상위 명령으로 제어 (B 입장에서는 시나리오 1과 동일)

### 데이터 흐름
```
깊이 카메라(Gazebo) → ros_gz_bridge → [A] ai_inference_node
        │ POSIX 공유 메모리 /pac2026_control_cmd (Seqlock)
        ▼
[B] rt_control_node (1ms, SCHED_FIFO, 격리 코어)
        │ Topic /hal/final_command (HalCommand)
        ▼
[C] hal_command_gateway → 로봇 어댑터 → 로봇/Gazebo
        │ Topic /hal/state, /hal/error
        └──────────────► [B]
```

---

## 2. 확정 사항 (Fixed)

| 항목 | 값 |
| --- | --- |
| OS | Ubuntu 24.04 LTS — **TBD 확인** |
| 미들웨어 | ROS 2 Jazzy — **TBD 확인** |
| 시뮬레이터 | Gazebo Harmonic — **TBD 확인** |
| 실시간 커널 | PREEMPT_RT Linux |
| 언어 | B: C++17 (rclcpp) / A: Python 또는 C++ — **TBD** |
| 빌드 | `colcon build --symlink-install` |
| 센서 | 깊이 카메라 (RGB + Depth) |

---

## 3. 공통 규약 (Conventions)

### 3.1 좌표
- 공통 작업 좌표계 하나 사용 (이름 **TBD**: `map` / `workpiece`)
- TF: `map → base_link → depth_camera_link`
- 로봇 기종별 좌표 차이는 **C(HAL)** 가 흡수한다.

### 3.2 단위 (SI)
| 물리량 | 단위 |
| --- | --- |
| 위치 | m |
| 자세 | quaternion |
| 속도 / 가속도 | m/s / m/s² |

### 3.3 시간
- 전 계층 **CLOCK_MONOTONIC, 나노초 int64**
- Gazebo 시간은 별도 필드로만 기록

---

## 4. 인터페이스 계약

| 연결 | 방식 | 이름 | 자료형 | QoS |
| --- | --- | --- | --- | --- |
| A → B | POSIX 공유 메모리 | `/pac2026_control_cmd` | ControlCommand (80 bytes) | 해당 없음 |
| B → C | Topic | `/hal/final_command` | `intelion_interfaces/msg/HalCommand` | Best effort, Volatile, Keep last 1 |
| C → B | Topic | `/hal/state` | `intelion_interfaces/msg/HalState` | Best effort, Volatile, Keep last 1 |
| C → B | Topic | `/hal/error` | `intelion_interfaces/msg/HalError` | Reliable, Volatile, Keep last 32 |
| B → 검증 | Topic | `/rt/cycle_stats` | TBD | Best effort, Keep last 10 |
| B → 검증 | 파일 | `logs/rt_cycle_<run>.csv` | CSV | 해당 없음 |

### 4.1 ControlCommand 공유 메모리 레이아웃 (8바이트 정렬, 리틀 엔디안) — *A·B 합의 필요*

| offset | 필드 | 타입 | 비고 |
| --- | --- | --- | --- |
| 0 | seq | uint64 | Seqlock 카운터 (홀수 = 쓰는 중) |
| 8 | command_type | uint8 | 0 position, 1 force_adjust, 2 path_adjust |
| 9 | status | uint8 | 0 OK, 1 검출 실패, 2 센서 무효, 3 판단 불가 |
| 10 | (패딩) | 6 bytes | 0으로 채움 |
| 16 | target[4] | float64 × 4 | |
| 48 | confidence | float64 | 0~1 |
| 56 | timestamp | int64 | 명령 생성 시각 (ns) |
| 64 | valid_until | int64 | 만료 시각 (ns) |
| 72 | capture_time | int64 | 카메라 프레임 수신 시각 (ns) |
| **합계** | **80 bytes** | | Python `struct`: `<QBB6x4ddqqq` |

**Seqlock 프로토콜**: 쓰기(A) — seq를 홀수로 → 본문 쓰기 → seq를 짝수로. 읽기(B) — seq 읽기 → 본문 복사 → seq 다시 읽기, 둘이 다르거나 홀수면 재시도(최대 3회).

---

## 5. 폴더 구조

```
PAC2026/
├── README.md               # 이 문서
├── CONTRIBUTING.md         # 기여 규칙 (브랜치·PR·커밋)
├── config/
│   └── scenario_config.yaml   # A·B 공유 시나리오 설정
├── docs/                   # 설계 문서, 회의 결정 사항
└── src/
    ├── pac_worlds/           # 공통     Gazebo 월드
    ├── pac_description/      # 공통     용접 로봇 + 깊이 카메라 모델, 정적 TF
    ├── pac_gz_bridge/        # 공통     Gazebo ↔ ROS 2 브리지 설정
    ├── pac_bringup/          # 공통     전체 통합 launch
    ├── intelion_interfaces/  # C 주도   HalCommand / HalState / HalError 메시지
    ├── pac_ai_inference/     # A(김나현) ai_inference.launch.py
    ├── pac_rt_control/       # B(홍연주) rt_control_node, rt_config.yaml
    └── pac_hal/              # C(김재원) hal_system.launch.py, 로봇별 어댑터
```

> `package.xml` / `CMakeLists.txt`(또는 `setup.py`)는 각 담당이 구현을 시작할 때 추가한다.

---

## 6. 실행 순서 (구현 후)

1. (대회 전 1회) RT 커널·커널 인자 확인, `cyclictest`로 기준 지터 측정
2. Gazebo + 로봇 스폰 → `ros2 launch pac_hal hal_system.launch.py` → C가 READY 출력
3. `ros2 launch pac_rt_control rt_control.launch.py` → B가 RUN 진입
4. `ros2 launch pac_ai_inference ai_inference.launch.py` → A 명령 반영 시작
5. 종료 시 B가 HOLD로 감속 정지 후 CSV 기록 저장

A·B·C 중 무엇을 먼저 켜도 로봇이 튀지 않도록 설계한다.

```bash
git clone <이 저장소 URL> PAC2026 && cd PAC2026
colcon build --symlink-install && source install/setup.bash
```

---

## 7. 남은 결정 사항

- [ ] 공통 작업 좌표계 이름 (`map` / `workpiece`)
- [ ] 공유 메모리 생성 주체 (A 생성 · B attach 제안)
- [ ] AI 명령 TTL (200ms vs 40~60ms)
- [ ] 신뢰도 하한 c_min (0.5 vs 0.7)
- [ ] ControlCommand에 `status`, `capture_time` 추가 여부
- [ ] 시연 대상 (실물 로봇 vs 시뮬레이터, 로봇 몇 종)
- [ ] OS / ROS 2 / Gazebo 버전 최종 확인

---

## 8. 범위 밖

로봇 관절 서보 제어(로봇 내부 제어기) · 카메라 하드웨어 캘리브레이션 값 결정 · 모든 로봇 어댑터 사전 구현.
