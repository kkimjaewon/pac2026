# 실제 B 제어기와 FR3 C HAL 연결

B 원본은 `feature/rt-control` 브랜치의 `e5f2b91`이다. `pac_rt_control` 패키지를 가져오고
B/C 연결에 필요한 부분을 수정했다. ROS 패키지 이름은 `rt_control`이다.
공유 메시지 4개의 필드·상수는 동일하다. 기존 C의 메시지 패키지와 C 내부 메시지 2개를 유지했다.
`b_mock`과 실제 B는 별개이며 동시에 실행하지 않는다. GitHub 업로드는 별도 승인 후 진행한다.

## 수정 이유

- `session_id` 기본 42: C가 등록한 실행 번호와 일치해야 한다. B가 임의 번호를 만들면 E_FORMAT으로 거부된다.
- 시작 시 등록 세션의 신선한 READY/ACTIVE 상태를 기다린다. 기본으로 시간 초과 후 임의 위치에서 시작하지 않는다.
- 측정 TCP 위치와 자세에서 시작한다. AI가 없을 때 blend의 초기 목표도 측정 위치로 맞춰 불필요한 이동을 없앤다.
- FAULT 해제는 등록 세션과 명령 순번을 유지한다. 실제 새 실행은 B와 C를 같은 새 세션으로 재시작한다.
- B 재시작 시 C가 보고한 마지막 순번 뒤에서 이어서 발행한다. 단일 B publisher를 전제로 한다.
- limiter는 실제 monotonic 주기 간격을 사용한다. 고정 1ms로 계산하면 깨어나는 간격의 변화 때문에 C가 관절 가속도 초과로 거부할 수 있었다.
- `fr3_integration.yaml`은 Gazebo clock, 측정 자세, 보수적인 1mm/s 이동 설정을 사용한다.
  팀의 기존 `rt_config.yaml`은 그대로 남겼다. 개발 환경에서는 FIFO/CPU 고정/mlock을 끈다.

## 실행

ROS Jazzy와 로봇 의존성이 설치된 workspace 루트에서:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to pac_hal rt_control
source install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch pac_hal hal_simulation.launch.py robot:=franka_fr3 session_id:=42
```

다른 터미널에서 같은 ROS/overlay와 domain 환경을 설정한 뒤 실제 B를 실행한다:

```bash
ros2 launch rt_control rt_control.launch.py \
  config:="$(ros2 pkg prefix --share rt_control)/config/fr3_integration.yaml" \
  session_id:=42
```

세 번째 터미널에서 개발용 A를 실행한다:

```bash
ros2 run pac_hal fake_ai_node.py --ros-args -p mode:=normal
```

현재 fake AI는 map 좌표 `[0.3069,0,0.5903]` 근처의 작은 z축 목표를 만든다.
이 값은 예시 FR3 초기 배치용이며 물체 검출 결과가 아니다. 설치 위치/작업 좌표를 바꾸면
A 목표와 B의 `planned_position`을 함께 맞춰야 한다. 실제 A가 준비되면 fake AI를 종료하고 연결한다.
B에서 `fake_c:=true`를 켜지 않는다. 실제 C와 모의 C를 동시에 쓰면 안 된다.

로컬 프로젝트 helper는 `scripts/run_real_b.sh`이며, C가 이미 켜져 있으면 B만 실행할 수 있다.
B가 켜진 상태에서 다음 검사 스크립트를 동시에 실행하지 않는다.

## 검증

가짜 A 입력 → 실제 C++ B → 실제 C++ HAL → Gazebo FR3로 검사했다.
B/C가 같은 PC에서 CLOCK_MONOTONIC을 공유하는 조건이다.

- 원본 B 단위 검사: 7개 시험 실행 파일, 총 86개 검사 통과.
- 세션 43인 B는 C 세션 42에 명령을 보내지 않고 INIT에서 대기.
- AI 입력 없이 시작: 측정 위치 유지, z축 변화 0m.
- 실제 B 출력 세션 42, 명령 유효시간 5ms 확인.
- AI 목표 반영 후 실측 z축 이동 **2.000054mm** 확인.
- A 송신 중단 후 `RUN → HOLD` 확인. HOLD는 새 AI 반영을 막으며 즉시 정지 명령과는 다르다.
- AI layout_version 불일치 후 FAULT, 정상 버전 복원과 `/rt/reset_fault` 후 `FAULT → RUN` 확인.
- FAULT 해제 후 등록 세션 42 유지 확인.
- B 종료 후 C 수신단 watchdog 활성화와 모든 명령 관절 속도 0 확인.
- B 재시작 시 이전 순번 뒤의 명령이 C에서 정상 송신 처리됨을 확인.

반복 검사는 `PAC2026/tools/check_real_b_fr3.py`다. C/Gazebo만 켜고 workspace overlay를 source한 뒤 실행한다.
검사 자체가 B와 가짜 A를 실행하며 종료 시 B를 정리한다. C/Gazebo는 계속 켜 둔다.

```bash
python3 PAC2026/tools/check_real_b_fr3.py
```

결과는 스크립트 상위 폴더의 `logs/real-b-fr3-check.json`, B 로그는 `logs/real-b-fr3.log`에 남는다.
현재 로컬 결과는 프로젝트 루트 `logs/`에도 저장했다. 공유용 요약은 `docs/real-b-fr3-validation.json`에 있다.

최종 검사에서 C 오류 활성화 이벤트는 0개였고, 로그의 최대 주기 지터는 1052.4μs였다. 일반 커널/GUI 환경의
주기 지터는 수백 μs로 관측되었으며 1kHz±100μs 성능 달성을 뜻하지 않는다.
충돌 방지·실물 로봇·실제 AI 물체 검출·공정 완료 판정은 이 검사 범위 밖이다.
