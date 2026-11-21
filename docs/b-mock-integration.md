# 받은 B 모의 노드와 FR3 HAL 연결 결과

`pac_b_mock.tar.gz`의 `b_mock` 패키지를 `src/b_mock`에 추가했다.
첨부 파일의 설치 스크립트는 실행하지 않고 기존 ROS 2 Jazzy workspace에서 빌드했다.
ControlCommand, HalCommand, HalState, HalError의 필드·상수는 기존 정의와 같아
공유 `intelion_interfaces`를 교체하지 않았다. UR5e와 실제 B 제어기 구현은 변경하지 않았다.

## 연결 규칙

- C launch와 B의 `session_id`를 42로 맞춘다. C는 미등록 세션을 자동 수락하지 않는다.
- B는 신선한 C 측정 상태를 기다리고 측정 TCP·orientation에서 출발한다.
- 기본 이동은 y축 2mm, 6초 왕복이다. 큰 절대 목표 점프로 시작하지 않는다.
- 명령 시간은 같은 PC의 monotonic clock, TTL은 5ms다. header에는 C 측정 Gazebo stamp를 복사한다.
- 만료 장애는 TTL 형식을 유지한 채 시간을 과거로 옮긴다. 미등록 세션 장애는 한 번 시험하고 원래 세션으로 복귀한다.
- B 송신 중단 시 C 수신단의 만료/watchdog가 감속 hold를 수행한다.

세부 파라미터·장애 주입 명령은 [b_mock 사용법](../src/b_mock/README.md)에 있다.

## 실행

GitHub에서 받은 팀원은 [pac_hal README의 workspace 실행법](../src/pac_hal/README.md)을 사용한다.
아래 helper와 `checks/`, `logs/` 경로는 재원의 로컬 프로젝트에 있으며 팀 저장소에는 포함하지 않는다.

프로젝트 루트 `/home/jaewon/Documents/ChatGPT/intellion`에서 각각 다른 터미널로 실행한다.
Gazebo/HAL이 이미 켜져 있다면 첫 번째 명령은 다시 실행하지 않는다.

```bash
bash scripts/launch_hal_sim.sh session_id:=42
```

```bash
bash scripts/run_b_mock.sh --ros-args -p session_id:=42 -p motion:=sine
```

Ctrl+C로 B를 종료한다. 다른 B publisher나 `hal_demo_publisher`를 동시에 실행하지 않는다.

## 검증 결과

- b_mock 빌드 성공, 경로·TTL 단위 검사 11개 통과, flake8 검사 통과.
- Gazebo FR3의 측정 y축 왕복 최대 이동량: **1.999826mm**.
- 송신 중단 후 수신단 watchdog와 명령 속도 0 확인.
- 만료 E_EXPIRED(3), 재전송 E_FORMAT(1), NaN E_FORMAT(1), 작업영역 밖 E_OUT_OF_RANGE(4) 확인.
- 미등록 세션 거부 후 기존 세션 42로 복구 확인, line 명령 송신 확인.

현장 검사는 루트 `checks/b_mock_fr3_check.py`, 결과는 `logs/b-mock-fr3-check.json`에 저장했다.
동일 PC·Gazebo 시뮬레이션에서 확인한 결과이며 실물 로봇 검증은 아니다.
Python의 예정 발행률은 1000Hz지만 검사 중 실제 발행률은 약 833–935Hz였다.
GUI와 검사 부하에서 TTL 만료·IK 시간 제한·제약 거부도 관측되어 1kHz 실시간 성능을 보장하지 않는다.
