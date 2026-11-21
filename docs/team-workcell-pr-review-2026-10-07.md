# 팀 월드·C HAL PR 검토안

검증 기준 main: `4dcd4e8` (B target_offset/run_id 포함). 작성 2026-10-07.
원격 push·PR 생성·브랜치 병합은 사용자 확인 대기. 현재 검증본은 프로젝트 루트 `checks/c-pr-review`에 있다.

## 포함할 변경

- `pac_hal`: 팀 월드 하나로 FR3·C 위치 제어·카메라 bridge·TF를 켜는 `team_workcell.launch.py`.
- 시뮬레이터 초기 TCP (0.10, 0, 0.83), 아래 방향. 팔과 철판 검출 영역을 분리한다. 실물 자동 이동 기능은 아니다.
- 기존 FR3/UR5e 공통 HAL, 모델·설정·전송 분리, 관절/충돌 검사·상태·watchdog·복구 보완과 관련 테스트. 통합 launch가 사용하는 C 변경의 선행 의존성이다.
- `hal.csv` 비동기 기록과 구간별 monotonic 시각, source_result_id·session_id·command_seq 연결. ACK는 목표 채택이며 TCP 도착이 아니다.
- `intelion_interfaces`: main의 공개 메시지 4개 정의는 유지하고 C 내부 `HalJointCommand`·`HalJointStatus` 2개와 생성 목록·설명을 추가한다.
- 기존 C 개발용 `b_mock`과 C 실행·설계·검증 문서. 실제 B는 main 버전을 사용한다.
- `.gitignore`: main 항목과 로컬 logs 제외를 함께 유지한다.

A·실제 B·팀 월드의 최종 파일은 검증한 main 버전과 같게 유지한다. 원본 HWP/PDF·개인정보·로그/영상·빌드 결과·옛 A/B 제안 패치는 업로드 범위에서 제외한다.
과거 성능 보고서에 적힌 A/B 제안 시험은 당시 조건의 역사적 기록이며 이번 무수정 A/B 실행과 구분한다.

## 병합 정리 방식

Git 병합 사전 계산에서 예전 B 사본·공통 메시지·b_mock·.gitignore 충돌이 확인됐다.
`pac_rt_control`, A, bringup, world는 main 우선; C 패키지는 현재 검증본; 내부 메시지는 추가한다.
공개 메시지의 필드·상수 정의는 main과 C의 차이가 주석/공백뿐인지 대조했으며 main 정의로 새로 빌드했다.
승인 후 jaewon의 검토 대상 C 변경을 커밋하고 main을 병합·정리한 최종 파일을 이 검증본과 대조한 뒤 push한다.
팀 main으로의 최종 PR 병합은 B 담당이 수행할 예정이다.

## 실제 검증

- `intelion_interfaces`를 별도 새로 빌드한 뒤 최신 main A/B와 C 재빌드 성공. 예전 설치 메시지에만 의존한 검증을 피했다.
- B·C 검사 결과: **186 tests, 0 errors, 0 failures, 0 skipped**. C 개발용 b_mock motion 검사도 **11개 통과**.
- 실제 RGB-D A(verifier)→main B→C→Gazebo FR3 구동. A/B 소스 변경 없음.
- B의 main fr3_integration.yaml에서 `planned_position`만 launch 인자로 **[0.5, 0, 0.71]** 지정. 로그 루트만 격리된 시험 폴더로 돌렸다.
- B 목표 **(0.500710, 0.000713, 0.809999)** 확인: A 표면 z≈0.71에 offset +0.10 반영.
- B 기본 속도 1mm/s 유지. 약 7초 관측에서 TCP **6.734mm** 이동. 전체 약40cm 접근·목표 도착은 이번 시험에서 확인하지 않았다.
- 같은 폴더·run_id의 rt.csv/hal.csv **6,566건**: command_seq·AI 결과 번호·B 생성 시각 연결 성공.
- sent 6,566행, applied 626행, 모든 기록 error_code=0, trace queue dropped=0. 100Hz 상태는 모든 명령의 applied 시각을 관측하지 못한다.
- 한 Gazebo, GUI 없음, camera RGB/depth/info·TF 확인. 이번 startup 측정 RTF **0.493**. 앞선 0.996~0.999와 조건 차이가 있어 속도를 보장하지 않는다. 1kHz±100μs는 미검증/미달 상태를 유지한다.
- 첫 실행은 A 초기화 완료 전에 6초 대기가 끝나 실패. 정상 A 출력 준비를 기다린 재실행에서 위 결과 확인. 운전 중 A/B/C 모두 살아 있었고 종료 후 테스트 프로세스 정리 확인.

로컬 증거: `logs/c-pr-review-test-results.log`, `logs/c-pr-review-runtime-v2/report.json`, `csv-audit.json`, `shared/pr_review_1007_v2/`.

## 팀 실행 명령

빌드한 팀 workspace를 source한 뒤 서로 다른 터미널에서 같은 새 run_id를 사용한다.

```bash
ros2 launch pac_hal team_workcell.launch.py gui:=false session_id:=42 run_id:=demo_1007_01
ros2 launch rt_control rt_control.launch.py config:="$(ros2 pkg prefix rt_control)/share/rt_control/config/fr3_integration.yaml" session_id:=42 run_id:=demo_1007_01 planned_position:="[0.5, 0.0, 0.71]"
ros2 launch ai_inference ai_inference.launch.py detector_mode:=verifier
```

기존 team sim.launch.py와 C의 다른 simulation launch를 함께 켜지 않는다. B 기본 v_max=0.001m/s라 대기점부터 목표까지 수 분이 필요하다. 속도 변경 시 경로·관절 제한·실시간 검증을 다시 해야 한다.
B/C 기본 저장 위치는 `~/pac_logs/demo_1007_01/`. 현재 A는 log_dir 아래 자체 시각 이름의 ai CSV를 만들어 A run_id 공통화는 별도 담당 확인이 필요하다.

## BC-06 확인 범위

사용자가 BC-06과 BC-07이 같은 요청임을 확인했다. 별도 미확인 기능을 추가하지 않고 통합 launch 요청으로 함께 처리한다.
