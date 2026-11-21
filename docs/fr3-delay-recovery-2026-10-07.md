# FR3 C 추가 보완: 지연·철판 이동·정지 후 재개 (2026-10-07)

최종 조건: 실제 Gazebo FR3 + C + 팀 B를 별도 복사해 만든 READY 대기 수정 제안본. A는 30Hz 검증 발행기다. C CPU=15, 일반 스케줄링, `ik_budget_clock=thread_cpu` 비교 모드. 팀 B 원본, A, 나현 원본 월드, Git 원격은 변경하지 않았다. 이번 작업은 아직 push하지 않았다.

## 최종 결과

- x/y/z ±5cm, 중앙 복귀, 철판 이동 후 추종 **8/8 도착**. 이동 구간 active 오류 이벤트 0, Gazebo 로봇 접촉 0. 보고 시점 최대 목표 오차 약0.00134mm. 실제 하드웨어 정확도로 해석하지 않는다.
- 철판 실제 Gazebo 모델 pose가 `(0.35,0,0.705)`에서 `(0.39,0.02,0.705)`로 바뀐 것을 native Pose_V 구독으로 확인했다. 서비스 응답만으로 이동했다고 판단하지 않았다.
- A 출력 중단 → HOLD/목표 유지. C 수신단 비활성화 → E_STATE_STALE/E_DISCONNECTED, FAULT. 재활성화·수동 reset → 세션42/순번 유지. B 종료 → watchdog 속도0. 모두 재검증했다.
- C 검사 **60개**, B 수정 제안본의 기존 검사 **86개**, 추가 standalone READY 재개/수동 reset 검사 통과.
- 준비된 서비스로 철판 이동 **20회 반복**, 모든 응답 성공, 수신단 원본 monotonic sample 최대 간격 **13.18ms**, active 오류 0.
- 비동기 기록 큐 enqueued=92,261, dropped=0. 최종 시험 C의 IK+관절 제한+충돌 검사 sent 구간 median=74.3µs, p99=108.5µs, max=939.7µs. 이 수치는 1kHz 주기 지터가 아니다.

원본 증거는 프로젝트 루트 `logs/c-delay-recovery/final-warm-proposed-b/report.json`, `plate-world-pose.jsonl`, `teleport-warm-20.json`, `warm_persistent_final_v2/hal.csv`, `feedback_gaps.csv`, `trace_stats.yaml`에 있다. 테스트 코드는 루트 `checks/c-delay-recovery`, `checks/world-validation`에 있다. 팀 저장소에 테스트 로그/별도 B 빌드 전체를 올리는 범위는 아니다.

## C에서 바꾼 부분

### 1. 제어 콜백에서 기록 작업 분리

`HalLatencyTrace`의 CSV 출력과 ACK map 처리를 별도 스레드로 옮겼다. 단일 executor가 고정 크기 SPSC 큐에 POD 이벤트만 넣는다. 기록이 밀려도 제어가 기다리지 않고, 누락은 `trace_stats.yaml`로 확인한다. 실제 수신 timestamp를 큐에 넣으므로 기록 스레드 지연을 ACK 수신 시간으로 잘못 쓰지 않는다.

DDS/기록 스레드 생성 후 현재 executor의 CPU/FIFO 설정을 적용한다. 기록 스레드가 C executor의 CPU/우선순위를 의도치 않게 물려받지 않도록 했다. FIFO 권한을 자동 변경하지 않았다.

### 2. 계산 예산과 명령 만료를 구분하는 선택 모드

기존 `wall` 500µs 기준은 기본값으로 유지한다. 새 readonly launch/노드 인자 `ik_budget_clock:=thread_cpu`는 실제 CPU 계산 시간을 500µs로 제한한다. **명령의 wall TTL 5ms는 유지**하며 IK 종료, 충돌 검사, 전송 직전, 수신 controller에서 만료 명령을 거부한다.

wall 모드 실험에서는 실제 계산이 26~37µs인데 선점으로 wall 537~733µs가 걸려 IK_FAILED가 발생했다. CPU 모드 실제 시험에서는 wall 500µs를 넘었지만 CPU 계산과 TTL을 만족한 명령을 처리했고, 이 때문에 발생하던 불필요한 정지는 없었다. CSV에 IK wall/CPU 시간과 모드를 함께 남긴다.

이 모드는 실시간 OS를 대체하거나 1kHz±100µs를 보장하지 않는다. 예산의 의미가 달라지므로 팀에 차이를 알리고 선택해야 한다. 일반 launch와 노드의 기본값을 몰래 바꾸지 않았다.

### 3. 철판 이동 서비스 준비를 C 시작 이전에 완료

새 Gz CLI 프로세스를 매번 만들면 철판 이동 10회 모두 원본 상태 생성 공백이 약100~110ms로 재현됐다. 카메라 렌더링을 제거한 6회에서도 최대109.3ms로 재현됐다. ROS–Gazebo 서비스 브리지도 최초 요청에서는 같은 공백이 있었지만, 계속 살아 있는 브리지를 준비한 뒤 요청하면 공백이 사라졌다. 정확한 Gazebo 내부 함수 비용을 확정한 결과는 아니다.

`hal_simulation.launch.py`는 지속적인 SetEntityPose 서비스 브리지를 만든다. 충돌 장면을 사용하는 FR3 launch에서는 `prepare_workcell_service`가 철판의 **원래 SDF pose**로 준비 요청을 하고, 신선한 FR3 joint stream이 0.3초 유지되는 것을 확인한 뒤 C 준비/실행을 시작한다. 준비 실패 시 launch를 종료하고 C를 시작하지 않는다.

시연 중 이동도 이 ROS 서비스로 요청한다. 새 `gz service` 프로세스를 계속 호출하는 방식은 개선된 경로가 아니다. STATE_STALE 임계값을 늘리지 않았으며, 실제 통신 단절은 계속 정지/FAULT 처리한다.

[Gazebo UserCommands 원본](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/user_commands/UserCommands.cc)은 서비스 요청을 큐에 넣고 PreUpdate에서 실행한다. 따라서 검증에서는 별도의 native world pose 구독으로 적용 여부까지 확인했다.

### 4. READY에 실제 측정 속도 반영

기존 C는 수신단의 목표 속도가 0이면 실제 관절이 아직 따라가는 중이어도 READY를 표시할 수 있었다. 이제 실제 측정 관절 속도와 수신단 속도가 모두 정지 기준 안에 있어야 READY다. 실제 속도를 알 수 없으면 ACTIVE로 표시한다.

앞선 B 복구 로그에서는 B limiter 속도0 직후 실제 측정 위치로 reset하며 TCP가 한 주기에 약0.82µm 바뀌었다. 이 변화가 관절3 가속도1.197rad/s²(제한1.0) 거부로 이어졌다. B 출력의 정지와 실제 로봇의 정지를 구분해야 한다.

측정값으로 C 기준을 미세 갱신하는 실험도 했지만 실제 시험에서 해당 조건이 발동하지 않아 실행 코드에서 제외했다. 움직임 제한을 우회하는 보정으로 해결했다고 주장하지 않는다.

## B 담당에게 전달할 수정 제안

`SafetyStateMachine::update()`의 `stopped` 판정에 **신선한 C HAL_READY**를 추가해 주세요. B limiter 속도만0인 상태에서 DECEL_STOP을 풀거나 FAULT reset을 승인하면, 로봇이 추종 중인 측정 위치로 기준이 갑자기 바뀔 수 있습니다.

```cpp
const bool stopped = in.limiter_speed <= config_.stopped_speed &&
  (!config_.require_c_state || (in.c_has_state &&
    in.c_state_age_ns <= config_.c_state_timeout_ns && in.hal_state == HAL_READY));
```

제안본은 별도 복사/빌드에서 검사86개와 실제 B→C→FR3 최종 시험을 통과했다. 팀 B 원본에는 아직 적용하지 않았다. 루트 `docs/b-recovery-ready-proposed.patch`가 적용할 패치다. E_SEND_FAILED=7 처리와 지속 EXPIRED 빈도 집계 계약은 이전 요청과 함께 B 담당이 검토해야 한다.

## 실행 예시와 적용 범위

기존 B/b_mock/Gazebo가 없는지 먼저 확인하고, 프로젝트 루트에서:

```bash
bash scripts/hal_ros_exec.sh ros2 launch pac_hal fr3_guarded_workcell.launch.py \
  session_id:=42 ik_budget_clock:=thread_cpu
```

높은 대기 자세에서 작업점까지는 앞선 보고서의 세 경유점을 사용한다. 일반 A gate의 ±5cm 범위를 벗어나는 초기 접근은 B와 별도 단계로 합의해야 한다. 최종 전체 시험은 이미 접근한 작업점 자세에서 시작했다.

시연 중 서비스 예시(제안 월드, XY 이동5cm 미만):

```bash
bash scripts/hal_ros_exec.sh ros2 service call /world/franka_world/set_pose \
  ros_gz_interfaces/srv/SetEntityPose \
  '{entity: {name: steel_plate, type: 2}, pose: {position: {x: 0.39, y: 0.02, z: 0.705}, orientation: {w: 1.0}}}'
```

실제 A 연결/카메라 TF/표면점 z+0.10 적용은 이번 검증 범위에 포함되지 않는다. 테스트 발행기는 최종 TCP를 보냈다. 나현 최신 원본 월드, 실물 공구/로봇, UR5e, 5cm/s·0.25m/s², 충돌의 연속 안전 보장은 여전히 별도 검증 대상이다. 추가 push/PR/병합은 사용자 검토/승인 후 진행한다.
