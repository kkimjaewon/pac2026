# BC-07·08·09 C 답변 및 실행 검증

> 10/07 14:48 추가: main 4dcd4e8에는 B 높이 오프셋·run_id가 반영됐다. 최신 실제 A→B→C 연결·공통 폴더 검증은 [PR 검토안](team-workcell-pr-review-2026-10-07.md)을 따른다. 아래 답변은 이전 main 기준 기록이다. 사용자 확인에 따라 BC-06은 BC-07과 같은 요청이다.

작성: C 김재원 | 10/07 12:43. 로컬 구현·검증 완료, push·PR·병합 전 재원 검토 대기.
팀 main `c90e9c7`의 월드와 수정하지 않은 A/B를 별도 overlay에서 사용했다.
아래 답변은 팀 문서의 **기존 BC 요청 블록 안**에 이어 붙인다. 팀 합의는 아직이다.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ **[BC-07] 팀 월드 + C를 한 번에 켜는 방법 (Gazebo 하나로)** ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

- **요청**: B → C | 10/07 11:27 | 우선순위 🔴
- **필요 시점**: 10/07 18:00까지
- **상태**: C 답변 작성 완료 · B 확인 대기

▶ **요청 내용**

팀 월드·로봇·C 컨트롤러·카메라 브리지·TF를 한 Gazebo로 실행하고, 창 없이 실행되는지 확인.

▶ **관련**

`jaewon` 로컬 변경: `src/pac_hal/launch/team_workcell.launch.py`, `hal_simulation.launch.py`,
`config/fr3_team_scene.yaml`. 사용 월드: 팀 main의 `pac_worlds/worlds/franka_world.sdf`.
프로젝트 루트 검증 기록: `logs/bc-07-09-team/`, `logs/bc-07-09-team-repeat/`.

▶ **답변** (C 김재원, 10/07 12:43)

C 통합 진입점 `team_workcell.launch.py`를 추가했어요. 팀 월드를 그대로 읽고,
C 위치 컨트롤러와 카메라 브리지·TF까지 한 번에 실행합니다. 카메라의 pose·topic·frame은
월드에서 읽도록 해서 별도 숫자 설정이 서로 어긋나지 않게 했어요.

```bash
ros2 launch pac_hal team_workcell.launch.py gui:=false session_id:=42 run_id:=새로운_실행ID
```

`pac_worlds`와 `pac_hal`을 빌드하고 해당 install을 source한 환경에서 실행하면 됩니다.
이때 `pac_bringup/sim.launch.py`를 추가 실행하지 않아요. A/B 노드는 각자 실행합니다.
Gazebo 서버 하나, `-s -r` 실행, 카메라 640×480 수신과 map→depth_camera_link TF를 실제 확인했습니다.
짧은 측정 구간에서 RTF는 첫 실행 0.999, 재실행 0.996이었어요.
RTF는 시뮬레이션 진행 속도이며 1ms/±100μs 실시간 제어 성능을 뜻하지 않습니다.

▶ **결정** (10/07 12:43) → 요약 탭 옮김 ⬜

**제안·합의 대기:** 시연의 월드 실행 진입점을 `team_workcell.launch.py` 하나로 통일.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ **[BC-08] 로봇 시작 자세를 카메라 시야 밖으로** ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

- **요청**: B → C | 10/07 __:__ | 우선순위 🟡
- **필요 시점**: 10/08 12:00까지
- **상태**: C 답변 작성 완료 · B 확인 대기

▶ **요청 내용**

시작 자세의 팔 끝이 철판 검출에 섞이지 않도록 대기 자세 변경. B는 신선한 C 측정 위치에서 출발.

▶ **관련**

`team_workcell.launch.py`의 PARKING, `config/fr3_team_scene.yaml`.
카메라 이미지: 프로젝트 루트 `logs/bc-07-09-team/parking-camera.png`.
IK/경로 기록: `checks/bc-07-09/poses.json`, `checks/bc-07-09/approach.json`.

▶ **답변** (C 김재원, 10/07 12:43)

팔을 몸 쪽으로 접고 손끝을 아래로 향하게 시작하도록 바꿨어요.
대기 TCP는 map 기준 **(0.10, 0, 0.83)m**, quaternion xyzw **(1, 0, 0, 0)**입니다.
실제 Gazebo 측정은 (0.0999994, -0.0000010, 0.8299903)m이었어요.
시작 자세와 책상·철판 이동 범위·카메라·자기 충돌에 대한 C 검사도 통과했습니다.

팔 일부는 영상 아래에 보이지만 **철판 검출 영역과 분리**됩니다. 최신 A의 verifier를
수정하지 않고 연결했을 때 철판 출력은 (0.500710, 0.000713, 0.709999)m,
월드 표면 기준 위치 오차는 약 **1.006mm**였습니다. 이전 6cm 오차가 이번 시작 상태에서는 재현되지 않았어요.

이는 **시뮬레이터 초기 관절 설정**입니다. 실물 로봇을 켤 때 이 자세로 자동 이동시키는 코드는 아닙니다.
B는 C의 신선한 READY/ACTIVE 측정 pose에서 계속 출발하면 됩니다.
대기점→철판 위 (0.5, 0, 0.81) 직선 접근도 40개 구간의 오프라인 IK·충돌 검사를 통과했어요.
이 검사는 긴 IK 예산을 사용했으며 실제 B 주행·실시간 성공 보장은 별도로 확인해야 합니다.

**B에게 부탁:** 현재 main B에는 철판 표면 z에 +0.10m를 더하는 처리가 아직 없어요.
오프셋과 연속 접근을 반영한 뒤 실제 A→B→C 이동 중 가림·재검출을 함께 확인하면 좋겠습니다.
이번 B→C 구동 시험은 A 입력을 다른 토픽으로 돌려 시작 자세 유지만 확인했습니다.

▶ **결정** (10/07 12:43) → 요약 탭 옮김 ⬜

**제안·합의 대기:** 시뮬레이션은 접힌 대기 자세로 시작하고, B가 측정 위치에서 높이 오프셋을 적용해 접근.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ **[BC-09] hal.csv 기록 형식 (End-to-End 지연용)** ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

- **요청**: B → C | 10/07 __:__ | 우선순위 🟡
- **필요 시점**: 10/08 12:00까지
- **상태**: C 답변 작성 완료 · B 확인 대기

▶ **요청 내용**

session_id·command_seq로 B 로그와 연결할 C 구간 시각, 결과·오류 코드 및 공통 저장 위치 제공.

▶ **관련**

`include/pac_hal/hal_types.hpp`, `src/hal_command_gateway.cpp`, `src/hal_latency_trace.cpp`.
실제 기록: 프로젝트 루트 `logs/bc-07-09-team/C/team_bc070809/hal.csv`.
대조 결과: `logs/bc-07-09-team/csv-audit.json`.

▶ **답변** (C 김재원, 10/07 12:43)

기존 열을 유지하면서 이름이 명확한 구간 시각을 추가했어요. 기본 위치는
**`~/pac_logs/<run_id>/hal.csv`**이고, B의 실행 ID를 C launch의 `run_id`에 지정하면 같은 폴더를 사용합니다.
B는 현재 시작 시각+run_tag로 ID를 자동 생성하므로, B가 출력한 폴더 이름을 C에 넣어 주세요.
B를 session_id=42로 먼저 켜서 C를 기다리게 하고, 그 ID로 C를 켜면 됩니다.
같은 ID의 기존 hal.csv가 있으면 덮어쓰지 않고 실행을 거부합니다.

| 열 | 의미 |
|---|---|
| session_id, command_seq | C 명령의 세션·순번. B는 실행 meta의 세션과 rt.csv command_seq를 함께 사용 |
| source_result_id | C가 받은 명령의 AI 결과 번호. B used_ai_seq와 대조 |
| b_created_mono_ns | B 명령 생성 시각, CLOCK_MONOTONIC |
| received_mono_ns | C callback 수신 시각, 기존 t1 |
| validated_mono_ns | 형식·세션·TTL·작업 범위·모델·상태 등 IK 전 사전 검사 종료 |
| ik_end_mono_ns | IK 호출 종료. IK를 실행하지 않은 거부는 공란 |
| t3_mono_ns | IK 이후 관절 제한·경로 충돌 검사까지 종료한 시각 |
| transport_start_mono_ns, transport_end_mono_ns | 로봇 어댑터 전송 호출 시작·종료 |
| receiver_applied_mono_ns | 수신 컨트롤러가 목표를 채택한 시각, **실제 TCP 도착 시각은 아님** |
| ack_received_mono_ns | C가 해당 ACK를 받은 시각, 기존 t6 |
| result, error_code | sent/rejected/applied 및 숫자 오류 코드. 성공 0 |

첫 실행에서 **5,965건**을 실제 B rt.csv와 연결해 생성 시각이 일치하고, 기록된
C 구간 시각의 순서도 맞는 것을 확인했습니다. 로그 큐 dropped=0입니다.
파일 기록은 별도 worker에서 처리해 제어 callback에서 파일 I/O를 하지 않습니다.

**분석할 때 구분할 점:** sent와 applied는 같은 명령의 서로 다른 단계 행입니다.
수신 상태는 100Hz이므로 모든 1kHz 명령에 applied 행이 생기지는 않아요.
이번 첫 실행에서는 applied 569건을 관측했습니다. ACK가 없는 행은 적용 시각 미관측으로 처리하고,
TCP 도착이나 전송 실패로 간주하지 않습니다. t5는 명령별 실제 TCP 측정과 연결할 계약이 없어 공란입니다.
AI 결과→목표 채택 지연은 관측된 applied 행을 이용할 수 있지만, 모든 명령의 최초 적용 시각이나
실제 TCP 도착까지 측정하려면 별도 상태 관측·상관 규칙이 필요합니다.

▶ **결정** (10/07 12:43) → 요약 탭 옮김 ⬜

**제안·합의 대기:** B와 C 로그는 세션·command_seq로 연결하고, 목표 채택 지연과 실제 TCP 도달 지연을 구분.

## 재원 로컬 실행 및 검증 범위

프로젝트 루트 `/home/jaewon/Documents/ChatGPT/intellion`에서:

```bash
bash scripts/launch_team_hal.sh gui:=false session_id:=42 run_id:=새로운_실행ID
```

이 로컬 helper는 root ros2_ws의 C와 `checks/bc-07-09/install`의 팀 main 검증 overlay를 사용합니다.
팀의 일반 ROS workspace에서는 최신 pac_worlds와 변경한 pac_hal을 빌드·source한 뒤 위 ros2 launch 명령을 사용합니다.
다른 설치 overlay를 사용하려면 `TEAM_WORLD_OVERLAY=/절대경로/install`을 지정합니다.
검증용 ROS_DOMAIN_ID는 146으로 격리했고, 배포용 helper 기본은 팀의 42입니다.

- Release 빌드 성공. `colcon test-result`: 78 tests, 0 errors, 0 failures, 0 skipped.
- headless 통합 실행 두 번, 한 Gazebo와 C 컨트롤러·카메라 브리지·TF 확인.
- 실제 A의 정지된 철판 검출 및 실제 B→C의 대기 pose 유지 확인. 전체 A→B→C 목표 이동은 이번 시험에서 제외.
- 재실행에서 EXPIRED 오류 발생·해제 이벤트 2회 관측. 5ms TTL은 유지했으며 실시간 성능 미달 문제를 해결했다고 표시하지 않음.
- 첫 A 종료에서 pybind11 take_message 예외가 있었으나 재실행에서는 종료 전 A/B/C 모두 살아 있음을 확인했고 같은 종료 예외는 재현되지 않음.
- 팀 철판은 동적 물체. C 충돌 장면은 현재 높이·자세와 XY 반경 5cm 이동 범위를 가정함. 실제 런타임의 낙하·회전·범위 밖 이동 추적은 별도 구현 필요.
- 카메라는 월드에 collision이 없어 visual 크기를 C의 가상 충돌 영역으로 추가함. 실물 공구·케이블 전체 모델은 아직 없음.
- 두 번째 실행 후 테스트가 만든 Gazebo·A/B/C 프로세스 정리 확인. push·PR·병합하지 않음.
