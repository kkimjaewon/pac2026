# PAC2026 실시간 제어 계층(B) 개발 레퍼런스

> **[안내]** A→B·B→C는 v0.3부터 ROS 토픽(`intelion_interfaces`). 1.2~1.4장의 공유 메모리 설명은 B 내부 Seqlock 버퍼 설계로만 참고.

> 이 문서는 Claude Code가 B를 구현할 때 참고하는 기준 문서다.
> 출처: 드라이브 「아이디어 요약서」(3차 수정본), 「PAC2_아이디어 계획」(1계층 4단계 ControlCommand, 2계층-3계층 협의, 통신계층 아이디어).
> 로봇은 HD현대 HA006B에서 **Franka FR3**로 변경됨 (Gazebo, `~/franka_ws`).

## 0. 표기 규칙

- **[확정]** 팀 문서에 이미 정해진 값. 임의로 바꾸지 말 것.
- **[제안]** 문서에 방향만 있고 숫자·형식이 없어서 B가 초안으로 정한 값. 팀 합의 후 바뀔 수 있으므로 **상수/설정 파일 한 곳에만** 두고 코드 곳곳에 박지 말 것.
- **[미정]** 아직 결정되지 않음. 설정값으로 열어 두고 기본값만 넣을 것.

대회: 2026-10-09(금) 11:00 ~ 10-10(토) 18:00. 평가 핵심은 AI 정확도가 아니라 **추론 결과가 제어에 반영되는 과정의 실시간성·안정성·확장성**.

---

## 1. 공통 인터페이스 계약

### 1.1 모든 계층 공통 규칙 [확정]

| 항목 | 규칙 |
|---|---|
| 시계 | `CLOCK_MONOTONIC`, **나노초 int64**. float 초 단위 금지. 모든 계층이 이 시계 하나만 사용 |
| Gazebo 시간 | sim time은 별도 필드로만 기록. 지연 계산에는 쓰지 않음 |
| 좌표계 | 작업셀 고정 공통 좌표계 `map`. 원점·축 방향은 시연 전에 정의 |
| 단위 | 위치 m, 자세는 정규화 쿼터니언 `(x, y, z, w)` 순서 |
| 명령 의미 | 매 주기 갱신되는 **절대** 목표(TCP). 중복 전달돼도 누적되지 않음 |
| 실패 표현 | 예외 금지. 항상 데이터를 보내고 안의 플래그로 성공/실패 표시 |
| 침묵 금지 | 정상이어도, 실패여도 명령을 보냄. 응답 없음과 고장을 구분하기 위함 |
| 블로킹 금지 | 계층 사이는 최신값 버퍼. 누구도 상대를 기다리지 않음 |
| 책임 | AI·HAL은 대체 제어값을 임의로 만들지 않음. 반영 여부·정도와 오류 대응은 **B 한 곳**에서 결정 |

### 1.2 A → B: AI 명령 공유 메모리

- 이름: `/pac2026_control_cmd` [확정]
- 동기화: Seqlock [확정]. A(Python, `multiprocessing.shared_memory` + `struct.pack_into`)가 쓰고, B(C++, `shm_open` + `mmap`)가 읽음
- 바이트 정렬: **리틀 엔디안, 아래 오프셋 고정** [제안]. 문서에 "엔디안/패딩 미정"으로 남아 있어서, 모든 필드를 자연 정렬 위치에 두고 예약 바이트로 패딩을 명시함. Python `'<'` 포맷과 C++ 구조체가 같은 72바이트가 됨

| 오프셋 | 필드 | 타입 | 설명 | 상태 |
|---|---|---|---|---|
| 0 | `seq` | uint32 | Seqlock 번호 (홀수 = 쓰는 중) | 확정 |
| 4 | `command_type` | uint8 | 0=position, 1=force_adjust, 2=path_adjust | 확정 |
| 5 | `status` | uint8 | 0=OK, 1=FAIL(검출 실패·프레임 누락·신뢰도<0.5) | 제안 |
| 6 | `reserved` | uint16 | 0으로 채움 | 제안 |
| 8 | `cmd_id` | uint64 | 명령 번호. 구간별 지연 추적용 | 제안 |
| 16 | `target[4]` | float64×4 | 의미는 아래 표. 안 쓰는 칸은 0 | 확정 |
| 48 | `confidence` | float64 | 0~1. FAIL이면 0 | 확정 |
| 56 | `timestamp_ns` | int64 | 제어기 PC가 **카메라 프레임을 받은 시각** | 확정 |
| 64 | `valid_until_ns` | int64 | `timestamp_ns + TTL` | 확정 |

Python 포맷 문자열: `'<IBBHQ4ddqq'` (크기 72)

```cpp
// include/pac2026/ai_command.hpp 에 들어갈 형태
struct AiCommandShm {
  uint32_t seq;
  uint8_t  command_type;
  uint8_t  status;
  uint16_t reserved;
  uint64_t cmd_id;
  double   target[4];
  double   confidence;
  int64_t  timestamp_ns;
  int64_t  valid_until_ns;
};
static_assert(sizeof(AiCommandShm) == 72);
static_assert(offsetof(AiCommandShm, target) == 16);
static_assert(offsetof(AiCommandShm, valid_until_ns) == 64);
```

`target` 슬롯 의미:

| command_type | target[0] | target[1] | target[2] | target[3] |
|---|---|---|---|---|
| 0 position (시나리오 1) | x [m, map] | y | z | 0 |
| 2 path_adjust (시나리오 3) | 계획 경로 대비 횡방향 편차 [m] | 0 | 0 | 0 |
| 1 force_adjust | 사용 안 함 (시나리오 2 미선택) | | | |

A 쪽 약속 [확정]: TTL 200ms, 센서 프레임 100ms 이상 미수신·검출 실패·신뢰도 0.5 미만이면 `status=FAIL`, `confidence=0`으로 **그래도 보냄**. A 프로세스가 죽으면 아무것도 안 오므로, B는 마지막 명령의 유효기한 만료로 중단을 감지함.

### 1.3 B → C: 최종 목표 버퍼

- 방식: A-B와 같은 최신값 버퍼 + Seqlock [확정: "B와 C 사이에도 같은 방식"]
- 이름: `/pac2026_hal_cmd` [제안]. 초기 문서의 ROS 토픽 `/hal/final_command`로 바꿀지는 C 담당자와 결정 [미정]. 어느 쪽이든 **B의 1ms 루프 안에서 ROS publish 금지**. 토픽이 필요하면 B 바깥(별도 스레드나 C 쪽)에서 공유 메모리를 읽어 내보냄
- 내용 [확정]: map 기준 TCP 절대 위치·자세, 명령 번호, 생성 시각, 유효기한. 초기 시연은 자세 고정·위치만 변경

| 오프셋 | 필드 | 타입 | 설명 | 상태 |
|---|---|---|---|---|
| 0 | `seq` | uint32 | Seqlock | 제안 |
| 4 | `mode` | uint8 | 0=RUN, 1=HOLD, 2=DECEL_STOP, 3=ESTOP | 제안 |
| 5 | `command_kind` | uint8 | 0=position, 2=path_adjust (원인 명령 종류, 기록용) | 제안 |
| 6 | `reserved` | uint16 | 0 | 제안 |
| 8 | `cmd_id` | uint64 | B가 매 주기 1씩 증가시키는 번호 | 제안 |
| 16 | `src_ai_cmd_id` | uint64 | 이 목표에 반영된 마지막 AI 명령 번호 (지연 추적) | 제안 |
| 24 | `pos[3]` | float64×3 | TCP 목표 위치 [m, map] | 확정 |
| 48 | `quat[4]` | float64×4 | TCP 목표 자세 (x,y,z,w) | 확정 |
| 80 | `created_ns` | int64 | B가 만든 시각 | 확정 |
| 88 | `valid_until_ns` | int64 | `created_ns + B→C TTL` | 확정(값은 미정) |

크기 96. Python 포맷: `'<IBBHQQ3d4dqq'`.
B→C TTL 기본값: 10ms [미정]. "C의 계산·전송 예산을 고려해 결정, AI TTL과 구분"이 확정된 원칙.

### 1.4 C → B: 상태·오류 버퍼

- 이름: `/pac2026_hal_state` [제안], Seqlock
- 내용 [확정 방향]: 최신 상태 + 갱신 시각 + 상태 코드 + 오류 정보. 새 상태가 안 오면 **수신 시각을 갱신하지 않음**(오래됐다는 걸 알 수 있게)

| 오프셋 | 필드 | 타입 | 설명 |
|---|---|---|---|
| 0 | `seq` | uint32 | Seqlock |
| 4 | `robot_status` | uint8 | 0=정상, 1=정지됨, 2=오류 |
| 5 | `error_code` | uint8 | 아래 표 |
| 6 | `reserved` | uint16 | 0 |
| 8 | `state_id` | uint64 | 상태 번호 |
| 16 | `last_applied_cmd_id` | uint64 | 마지막으로 송신 처리한 B 명령 번호 |
| 24 | `error_cmd_id` | uint64 | 오류와 관련된 명령 번호 |
| 32 | `tcp_pos[3]` | float64×3 | 측정 관절값→FK로 계산한 TCP 위치 [m, map] |
| 56 | `tcp_quat[4]` | float64×4 | TCP 자세 |
| 88 | `received_ns` | int64 | C가 로봇 상태를 받은 시각 |

크기 96. Python 포맷: `'<IBBHQQQ3d4dq'`.

오류 코드 [제안, 문서의 오류 종류를 번호로만 매김]:

| 코드 | 이름 | 뜻 | B의 대응 [제안] |
|---|---|---|---|
| 0 | OK | 정상 | RUN |
| 1 | CMD_EXPIRED | C가 받은 명령이 만료 | 경고 기록, RUN 유지 (반복되면 HOLD) |
| 2 | IK_FAILED | 역기구학 실패·계산시간 초과 | DECEL_STOP, 재개 승인 대기 |
| 3 | JOINT_LIMIT | 관절 범위·속도 초과로 차단 | DECEL_STOP, 재개 승인 대기 |
| 4 | STATE_TIMEOUT | 로봇 상태 갱신 중단 | DECEL_STOP, 상태 복원 + 재개 승인 대기 |
| 5 | COMM_LOST | 통신 단절 | DECEL_STOP, 재연결 후 새 명령부터 |
| 6 | ROBOT_FAULT | 로봇 측 오류 | ESTOP |
| 7 | INVALID_COMMAND | 형식 오류·모르는 좌표계 | 기록, 해당 명령 폐기 |
| 8 | STALE_OR_DUP | 역순·중복 명령 폐기 | 기록만 |

B 쪽 상태 신선도 검사 [제안]: `now - received_ns > 50ms`이면 B도 독자적으로 STATE_TIMEOUT으로 판단.

---

## 2. 전체 시나리오 장면

### 시나리오 1 — 비전 AI 기반 목표 위치 보정 (먼저 구현)

작업대 위 대상물이 처음엔 계획 위치에 있어 로봇이 계획대로 움직인다. 시연 도중 대상물을 수 cm 옮기면 A가 새 위치를 보고하고, 로봇은 멈추거나 튀지 않고 새 위치로 부드럽게 따라간다.

- A: RGB-D 영상 → YOLOv8n 검출 → bbox 중심 깊이 중앙값으로 3D 위치 → map 변환 → `position` 명령
- B: 1ms마다 읽고 3단계 처리 → 목표를 부드럽게 이동
- C: Franka 역기구학 → 관절 명령 → 상태 복원해서 반환
- 장애 주입: 추론 지연 50~300ms(200ms TTL 초과 포함), 추론 중단(AI 강제 종료 후 재시작), 이상값 혼입
- 기대 결과: 제어 주기 유지, 늦은 값은 약하게, 만료 값은 무시, 중단 중엔 마지막 목표 유지, 이상값은 걸러짐

### 시나리오 3 — 공정상태 추론 기반 경로 보정

로봇이 용접 기준선을 따라가는데 대상물이 틀어져 있어 점점 벗어난다.

- A: YOLOv8n-seg로 용접선 분할 → RANSAC 중심선 → 계획 경로 대비 횡방향 편차 → `path_adjust` 명령 (정상 ±2mm / 이탈 / 판단 불가)
- B: 편차를 α만큼 반영해 계획 경로 위 절대 목표를 수정. **시나리오 1과 같은 3단계 처리**
- 장애 주입: 용접선이 끊긴 구간(판단 불가 → 마지막 보정량 유지), 큰 모델로 지연
- 허용 범위: 기준선 대비 ±10mm

### 시나리오 4 — 이종 로봇 HAL (주로 C의 몫)

같은 상위 명령으로 서로 다른 로봇을 제어. 원래 "HDR50 + HA006B → HA006B를 UR5e로 교체"였으나 로봇이 Franka로 바뀌어 **조합 재결정 필요** [미정].
B는 코드·실행 파일 변경 없이 그대로 돌아가야 함(상위 코드 변경 0줄이 평가 포인트). B가 할 일은 C의 오류(IK 실패, 상태 수신 중단, 과거 명령 재전달)에 맞게 정지·보류·재개하는 것.

### 카메라 [미정]

문서는 손목 장착 RGB-D(640×480, 30Hz). 현재 Gazebo 환경은 작업대 위 고정 카메라(`team_franka_sim`). A 담당자와 결정 필요. B에는 영향 없음.

---

## 3. A ↔ B 상호작용

### 3.1 매 주기 B가 하는 일 (시나리오 1)

```
now = clock_gettime(CLOCK_MONOTONIC)
cmd = seqlock_read(/pac2026_control_cmd)   // 최대 3회, 실패하면 이전 주기 값
```

**1단계 유효성 검사** — 하나라도 걸리면 이번 값은 반영 안 함 (`valid = 0`)

```
finite   = 모든 target·confidence가 유한한 숫자
ok       = (status == OK)
fresh_ok = (now < valid_until_ns)
in_range = |target - planned_position| <= 0.05 m   (각 축 또는 거리, [제안] 거리 기준)
valid    = finite × ok × fresh_ok × in_range          (0 또는 1)
```

**2단계 가중치**

```
age       = now - timestamp_ns
freshness = clamp(1 - age / TTL, 0, 1)                // 방금 = 1, TTL에서 0
w         = clamp(confidence, 0, 1) × freshness × valid
```

**3단계 부드러운 반영**

```
a0     = 1 - exp(-dt / tau)                           // dt = 1ms, tau는 시뮬레이션으로 확정
alpha  = a0 × w
p_ai   = finite ? target : goal                       // NaN 정리 (아래 주의 참고)
goal   = goal + alpha × (p_ai - goal)                 // 필터된 목표
```

그다음 속도·가속도 제한(남은 거리 기반 미리 감속):

```
e       = goal - out_pos                               // 벡터
d       = |e|
v_des   = (e / max(d, eps)) × min(v_max, sqrt(2 × a_max × d))
v       = v_prev + clamp_norm(v_des - v_prev, a_max × dt)
out_pos = out_pos + v × dt
```

**무분기 규칙**: 위 계산은 if 없이 곱셈·min·max·clamp로. 선택이 필요하면 삼항 연산자(조건 이동 명령으로 컴파일)를 쓰되, 실행 경로가 바뀌는 if/else 분기는 쓰지 않음.

**주의 (NaN)**: `NaN × 0 = NaN`이다. valid=0을 곱해도 NaN이 사라지지 않으므로 **곱하기 전에 반드시 NaN을 현재 목표로 바꿔치기**할 것. 이 경우를 자동 테스트에 넣을 것.

### 3.2 상황별 기대 동작

| 상황 | A가 보내는 것 | B의 판단 | 결과 |
|---|---|---|---|
| 정상 | OK, 신뢰도 높음, 새 시각 | valid=1, w 큼 | 빠르게 따라감 |
| 추론 지연 (TTL 이내) | OK, 오래된 시각 | freshness 작음 → w 작음 | 약하게 반영 |
| 추론 지연 (TTL 초과) | 새 값 없음 | fresh_ok=0 → w=0 | 마지막 목표 유지 |
| A 프로세스 중단 | 아무것도 없음 | 만료로 중단 감지 → w=0 | 마지막 목표 유지, 재시작하면 이어감 |
| 검출 실패 | FAIL, 신뢰도 0 | ok=0 → w=0 | 마지막 목표 유지 |
| 이상값 (±5cm 초과) | OK | in_range=0 → w=0 | 걸러짐 |
| NaN | OK | finite=0 → w=0 | 걸러짐 |
| Seqlock 3회 실패 | (쓰는 중) | 이전 주기 값 사용 | 정상 진행 |

### 3.3 시나리오 3 차이점

`path_adjust`일 때는 `target[0]`(횡방향 편차)를 같은 3단계로 필터링해 `offset`을 만들고,
`goal = planned_path(s) + n(s) × offset` (s: 경로 진행 위치, n: 경로의 횡방향 단위벡터).
허용 범위는 ±10mm. 판단 불가(FAIL)면 마지막 `offset` 유지.

### 3.4 파라미터 [미정, 설정 파일로]

| 이름 | 기본값 | 비고 |
|---|---|---|
| `ttl_ns` | 200,000,000 | 확정값이지만 설정으로도 노출 |
| `tau_s` | 0.15 | 흔들림 vs 반응속도 보고 확정 |
| `v_max` | 0.10 m/s | Franka 안전 범위 안에서 조정 |
| `a_max` | 0.50 m/s² | |
| `range_pos_m` | 0.05 | 시나리오 1 |
| `range_path_m` | 0.010 | 시나리오 3 |
| `planned_position` | (0.50, 0.00, 0.20) | 계약 문서 v0.3 값. 월드(weld_cell)에서 철판 위치가 확정되면 바꿀 것 |
| `fixed_quat` | (1, 0, 0, 0) | 공구가 아래를 보는 자세, Franka 기준 확인 필요 |

---

## 4. C ↔ B 상호작용

### 4.1 매 주기

1. B가 최종 목표를 `/pac2026_hal_cmd`에 덮어씀 (C를 기다리지 않음)
2. B가 `/pac2026_hal_state`를 Seqlock으로 읽음 (최대 3회)
3. 상태 신선도와 `error_code`로 B의 모드 결정

### 4.2 B의 모드 (Safety Supervisor)

```
RUN ──(오류 2,3,4,5)──▶ DECEL_STOP ──(정지 확인 + 원인 해소 + 재개 승인)──▶ RUN
 │                                                                         ▲
 └──(오류 6)──▶ ESTOP ─────────────(수동 해제)────────────────────────────┘
RUN ──(시작 요청 없음)── 시작 시에는 HOLD에서 출발, 상태 수신 확인 후 RUN
```

- **HOLD**: 현재 출력 위치를 목표로 고정
- **DECEL_STOP**: 속도를 a_max로 줄여 0으로 만들고 그 자리 유지. 정지 판단은 C의 상태 기준이지 "명령을 보냈다"가 아님
- **재개**: 새 상태가 계속 들어오는지 확인 후, 현재 위치에서 새 목표로 시작. 만료된 옛 명령을 다시 쓰지 않음
- 시작·정지·재개 같은 **상태 전환은 최신값 덮어쓰기와 별개로 확인 절차**를 둠 [확정 원칙]. 구현은 [제안]: `mode` 필드 + C가 `robot_status`로 확인

### 4.3 측정 지점 (명령 번호 기준) [확정 방향]

```
t0 B 명령 생성 → t1 C 수신 → t2 변환 완료 → t3 송신 → t4 로봇 상태 수신 → t5 상태 버퍼 저장 → t6 B가 상태 읽음
```

B는 t0, t6과 `cmd_id`, `src_ai_cmd_id`를 기록. 나머지는 C가 기록하고 나중에 `cmd_id`로 합침.

---

## 5. 전체 파일·데이터 흐름도

```
 [로봇 PC: Gazebo + Franka FR3]                      [제어기 PC: PREEMPT_RT]
                                                     
  RGB-D 카메라 ──(ROS 2 토픽, 이더넷)──▶  A: AI 추론 (Python, 일반 우선순위, 비격리 코어)
                                           센서어댑터 → 추론 → 타겟매퍼 → 쓰기
                                                       │
                                     /dev/shm/pac2026_control_cmd  (Seqlock, 72B)
                                                       │
                                                       ▼
                                         B: 실시간 제어 (C++, SCHED_FIFO, 격리 코어, 1ms)
                                           읽기 → 검사 → 가중치 → 필터 → 속도·가속도 제한
                                           → 모드 결정 → 쓰기          └─ 링버퍼 ─▶ 로그 스레드 ─▶ logs/*.csv
                                                       │         ▲
                                  /dev/shm/pac2026_hal_cmd       /dev/shm/pac2026_hal_state
                                       (Seqlock, 96B)               (Seqlock, 96B)
                                                       ▼         │
                                         C: 통신·HAL (ROS 2 노드, 비격리 코어)
                                           검사 → 좌표변환 → IK → 제한검사 → 명령 생성
  Franka (ros2_control) ◀──(ROS 2 토픽, 이더넷)──┘   상태 수신 → FK → 상태 버퍼
       └── 관절 상태 ─────────────────────────────▶
```

개발 중 (A, C 없음):

```
 tools/fake_a.py ──▶ /dev/shm/pac2026_control_cmd ──▶ B ──▶ /dev/shm/pac2026_hal_cmd ──▶ tools/fake_c.py
                                                      ▲                                       │
                                                      └──── /dev/shm/pac2026_hal_state ◀──────┘
 logs/*.csv ──▶ tools/analyze_log.py ──▶ 그래프 (주기 히스토그램, 지터, alpha, 위치·속도·가속도·저크)
```

---

## 6. B가 만들어야 할 파일과 내용

```
pac2026_b/
├── CMakeLists.txt
├── config/
│   └── b_params.yaml
├── include/pac2026/
│   ├── time_util.hpp
│   ├── ai_command.hpp
│   ├── hal_interface.hpp
│   ├── seqlock.hpp
│   ├── shm_region.hpp
│   ├── rt_setup.hpp
│   ├── blend_filter.hpp
│   ├── motion_limiter.hpp
│   ├── safety_supervisor.hpp
│   └── rt_logger.hpp
├── src/
│   ├── rt_setup.cpp
│   ├── blend_filter.cpp
│   ├── motion_limiter.cpp
│   ├── safety_supervisor.cpp
│   ├── rt_logger.cpp
│   └── b_controller_main.cpp
├── tests/
│   ├── test_seqlock.cpp
│   ├── test_blend_filter.cpp
│   ├── test_motion_limiter.cpp
│   └── test_safety_supervisor.cpp
└── tools/
    ├── shm_layout.py
    ├── fake_a.py
    ├── fake_c.py
    └── analyze_log.py
```

| 파일 | 들어갈 내용 |
|---|---|
| `time_util.hpp` | `now_ns()` (CLOCK_MONOTONIC → int64 ns), timespec 더하기·정규화, ns↔s 변환 |
| `ai_command.hpp` | 1.2의 `AiCommandShm`, `CommandType`·`Status` enum, `static_assert` 크기·오프셋 검사, 이름 상수 `/pac2026_control_cmd` |
| `hal_interface.hpp` | 1.3의 `HalCommandShm`, 1.4의 `HalStateShm`, `Mode`·`ErrorCode` enum, 이름 상수, `static_assert` |
| `seqlock.hpp` | 템플릿 `SeqlockWriter<T>`, `SeqlockReader<T>`. `std::atomic_ref`(또는 `__atomic`)로 seq를 acquire/release. 쓰기: seq+1(홀수) → 데이터 → seq+1(짝수). 읽기: `try_read(out, max_retry=3)` → 성공 여부 반환, 실패 시 out 미변경. 동적 할당·대기 없음 |
| `shm_region.hpp` | `shm_open` + `ftruncate` + `mmap` RAII. 생성/열기 모드, 소멸 시 `munmap`. 루프 시작 전에 모두 열어 둠 |
| `rt_setup.hpp/.cpp` | 옵션: `mlockall(MCL_CURRENT|MCL_FUTURE)`, `SCHED_FIFO` 우선순위, CPU affinity, 스택 미리 건드리기(prefault). 각 단계 성공/실패를 출력하고 **실패해도 계속 실행**(일반 커널·권한 없음 대비) |
| `blend_filter.hpp/.cpp` | 3장 1~3단계를 **순수 함수**로. 입력: 명령, now, 이전 목표, 파라미터. 출력: 새 목표, alpha, valid, w, 사유 비트. I/O·로그·시계 호출 없음 → 테스트에서 시간을 마음대로 넣을 수 있게 |
| `motion_limiter.hpp/.cpp` | 3.1의 속도·가속도 제한(남은 거리 기반 감속). 순수 함수. DECEL_STOP용 "속도를 0으로" 모드 포함 |
| `safety_supervisor.hpp/.cpp` | 4.2의 모드 전이. 입력: 상태 버퍼, now, 재개 요청. 출력: 모드. 상태 신선도 검사 포함 |
| `rt_logger.hpp/.cpp` | 고정 크기 SPSC 링버퍼(미리 할당). 루프는 한 줄 push만(가득 차면 버리고 카운트 증가, 대기 금지). 별도 스레드가 CSV로 저장. 열: 주기번호, 깨어난 시각, 지터, 실행시간, ai_cmd_id, valid, w, alpha, goal xyz, out xyz, v, mode, error_code |
| `b_controller_main.cpp` | 설정 읽기 → shm 열기 → RT 설정 → 로거 시작 → `clock_nanosleep(TIMER_ABSTIME)` 1ms 루프: 읽기 → blend_filter → motion_limiter → supervisor → 쓰기 → 로그 push. 종료 시그널 처리, 끝나면 주기 평균·최대 지터 요약 출력. 루프 안에서 메모리 할당·파일 I/O·printf·ROS 호출 금지 |
| `b_params.yaml` | 3.4 파라미터, B→C TTL, 상태 타임아웃, CPU 번호, 우선순위, 실행 시간(초), 시나리오 선택 |
| `test_*.cpp` | 3.2 표의 모든 상황 + NaN + 경계값(TTL 직전/직후, ±5cm 경계). 결과 궤적에서 속도·가속도 한계 초과가 없는지, 목표를 지나치지 않는지 확인 |
| `shm_layout.py` | 1.2~1.4의 Python `struct` 포맷과 크기 검사. fake 프로그램들이 공유 |
| `fake_a.py` | 공유 메모리에 position 명령 쓰기. 모드: `normal`, `delay`(50~300ms), `stop`, `outlier`, `fail`, `nan`, `move`(일정 시점에 대상물 이동). 실제 A와 같은 Seqlock 쓰기 순서 |
| `fake_c.py` | hal_cmd 읽어서 기록, 간단한 1차 지연 로봇 흉내로 hal_state 쓰기. 오류 주입: `ik_fail`, `state_stop`, `comm_lost` |
| `analyze_log.py` | CSV → 주기 히스토그램, 최대·평균 지터, 실행시간 분포, alpha·w 변화, 목표/출력 위치, 속도·가속도·저크, "AI 그대로 반영" 대비 비교 그래프 |

### 구현 순서

1. `time_util` + `rt_setup` + `b_controller_main`(빈 1ms 루프) → 지터 측정
2. `blend_filter` + `motion_limiter` + 테스트 (실시간 없이 시뮬레이션으로)
3. `seqlock` + `shm_region` + `ai_command` + `shm_layout.py` + `fake_a.py` → 루프에 연결
4. `hal_interface` + `fake_c.py` + `safety_supervisor` → 루프에 연결
5. `rt_logger` + `analyze_log.py` → 그래프
6. PREEMPT_RT 제어기 PC에서 isolcpus, cyclictest, AI 부하 유무 비교

각 단계가 끝나면 빌드·실행·테스트 확인 후 git commit.

### 팀과 확정해야 할 것 (회의 안건)

1. A와: 1.2의 `status`·`cmd_id` 필드 추가와 바이트 레이아웃 (72B), Python Seqlock 쓰기 순서
2. C와: 1.3·1.4 레이아웃, B→C TTL, 오류 코드 번호, 오류별 B 대응, 공유 메모리 vs ROS 토픽
3. 전원: 시나리오 4 로봇 조합(Franka + ?), 카메라 위치(손목 vs 고정), `map` 원점과 `planned_position`, Franka 고정 자세 쿼터니언
