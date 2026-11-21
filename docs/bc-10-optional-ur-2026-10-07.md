# BC-10: FR3 실행과 UR 선택 의존성 분리

작성: C 김재원 | 10/07 15:43. 로컬 수정·검증 완료, 새 push·PR은 재원 확인 대기.

▶ **답변** (C 김재원, 10/07 15:43)

제 PC의 UR은 소스 빌드가 아니라 apt 패키지예요. 설치되어 있는 것은
`ros-jazzy-ur-description` 3.5.1과 `ros-jazzy-ur-simulation-gz` 2.5.0입니다.
두 패키지 모두 `/opt/ros/jazzy`에서 조회됩니다.

이번 FR3 오류는 C 쪽 의존성 처리 문제였어요. FR3를 골라도 Gazebo 리소스 경로를 만들 때
`ur_description`을 찾았고, 테스트 설정에서 UR 모델 생성을 무조건 실행했습니다.
선택한 로봇의 리소스만 찾도록 수정하고, UR 패키지가 없으면 UR 검사만 건너뛰도록 바꿨어요.
패키지 선언에서도 UR 필수 의존성을 제거했습니다. FR3 검사는 계속 실행하므로
`BUILD_TESTING=OFF` 우회가 필요하지 않습니다.

UR 패키지를 검색할 수 없도록 격리한 환경에서 빌드와 검사를 실행했습니다.
검사 결과는 70개 중 오류·실패 0, UR 관련 6개 건너뛰기입니다.
같은 환경에서 Gazebo 서버 하나로 팀 월드·카메라·TF·실제 A/B/C를 실행했고,
FR3가 약 6.8mm 이동하는 동안 C 오류가 없었습니다.
UR이 있는 기존 환경에서도 78개 검사가 모두 통과했습니다.
이번 검증은 제 PC에서 했으며, B 그램의 직접 실행 확인은 아직입니다.
전체 목표 도착과 1kHz±100μs 실시간 성능을 통과했다는 뜻은 아닙니다.

따라서 시나리오 1은 이번 수정 반영 후 UR 설치 없이 먼저 시험하면 됩니다.
UR5e 시험을 위해 설치할 때는 아래 apt 모의 실행 결과를 먼저 확인해 주세요.
제 PC도 `controller_manager`와 `gz_ros2_control`은 소스 overlay가 선택되지만,
apt UR 설치가 다른 PC의 소스 빌드와 충돌하지 않는다고 보장할 수는 없어요.

```bash
# 설치 계획만 확인합니다. 실제 설치하지 않습니다.
apt -s install ros-jazzy-ur-description ros-jazzy-ur-simulation-gz

# 설치 계획을 확인한 뒤 UR 시험에 필요한 경우 실행합니다.
sudo apt install ros-jazzy-ur-description ros-jazzy-ur-simulation-gz
```

`ur_simulation_gz` apt 패키지는 `gz_ros2_control`, UR 컨트롤러, MoveIt 설정 등도 의존성으로
가져올 수 있습니다. 이미 소스로 빌드한 제어 스택이 있다면 설치 목록·업데이트 목록과
source 순서를 함께 확인해야 합니다. 제 PC의 확인 결과는 다음과 같습니다.

```text
ur_description    → /opt/ros/jazzy
ur_simulation_gz  → /opt/ros/jazzy
controller_manager → 프로젝트 ros2_ws/install/controller_manager
gz_ros2_control   → 프로젝트 ros2_ws/install/gz_ros2_control
```

UR 공식 소스 설치 안내는 [Universal Robots Gazebo Simulation README](https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation/blob/ros2/README.md)를 참고하세요.
이번 로컬 검증은 위 apt 버전 기준입니다. 소스 설치로 바꿔 검증한 것은 아닙니다.

## 수정 반영 후 B 실행

기존 FR3 소스 workspace와 ROS 환경을 source한 상태에서 해당 workspace 루트에서 실행합니다.
기존 설정의 `BUILD_TESTING=OFF`를 명시적으로 다시 켜 주세요.

```bash
colcon build --packages-select pac_hal --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select pac_hal
colcon test-result --verbose
ros2 launch pac_hal team_workcell.launch.py gui:=false session_id:=42 run_id:=bc10_fr3
```

UR 선택 시에는 `ur_description`, `ur_simulation_gz` 설치가 여전히 필요합니다.
`PAC_HAL_BUILD_UR_TESTS` 기본값은 ON이며, 두 패키지가 ament index에 있으면 UR 검사도 실행합니다.
옵션을 OFF로 하면 UR 검사만 생략합니다. FR3 검사나 전체 BUILD_TESTING을 끄지 않습니다.
UR 패키지가 검색되는데 모델 생성 자체가 실패하면 빌드 오류를 유지합니다.

## 변경 파일과 검증 근거

- `src/pac_hal/launch/hal_simulation.launch.py`: 선택한 로봇의 모델 리소스만 검색.
- `src/pac_hal/CMakeLists.txt`: ament index로 UR 유무 확인, UR native 검사만 조건부 등록.
- `src/pac_hal/tests/test_kinematics.py`: UR 비활성 시 UR Python 검사만 명시적 skip.
- `src/pac_hal/package.xml`: UR 실행·시험 필수 의존성 제거.
- 이 문서: B 답변·설치 방식·검증 범위 기록.

검증 로그는 프로젝트 루트에 보관합니다. 원시 실행 로그는 이번 push 대상에 포함하지 않습니다.

| 검증 | 결과 | 프로젝트 루트 기준 근거 |
|---|---|---|
| UR 설치 환경 빌드·검사 | 78개, 오류 0, 실패 0, skip 0 | `logs/bc10-with-ur-results-final.log` |
| UR 검색 차단 환경 빌드·검사 | 70개, 오류 0, 실패 0, skip 6 | `logs/bc10-no-ur-results-final.log` |
| 실제 A→B→C→FR3 실행 | 서버 하나, 6.758mm 이동, C 오류 없음 | `logs/bc10-fr3-no-ur/report.json` |
| A 철판 검출 | (0.500710, 0.000713, 0.709999)m, 월드 기준 오차 1.006mm | 같은 보고서 |
| B 목표 높이 | z=0.809999m, 표면 +0.10m | `logs/bc10-fr3-no-ur/shared/bc10_fr3_no_ur/rt.csv` |
| B/C CSV 연결 | command_seq 7,154개 연결, C sent/applied error_code 모두 0 | 같은 폴더 `rt.csv`, `hal.csv` |

UR 검색 차단 시험은 시스템 패키지를 삭제한 것이 아니라 ament index와 ROS prefix에서 UR을 제외한
격리 환경입니다. 실행 당시 두 UR 패키지 조회 실패를 확인하고 실제 Gazebo를 구동했습니다.
측정 구간 RTF는 약 0.713이며, 목표 도착 시험은 하지 않았습니다.
검증용 Gazebo·제어 프로세스는 시험 종료 후 정리했습니다.

▶ **결정** (10/07 15:43) → 요약 탭 옮김 ⬜

제안·합의 대기: FR3 시나리오 1의 빌드·실행은 UR 없이 유지하고, UR5e 시험 시에만 UR 모델 패키지를 설치.
