# 실제 A→B→C 성능 검증 — 2026-10-07

## 결론

실제 Gazebo RGB-D 영상 A → 최신 팀 B 검증 사본 → 현재 공통 C++ HAL → FR3를 구동했다. 철판 이동 3회에서 표면+10cm TCP 목표를 2mm 이내로 따라가고 0.3초 유지했으며, A 지연/범위 밖 값/출력 중단 차단과 입력 복귀, C 프로세스 단절 시 B 감속 정지 및 수신기 watchdog 정지를 확인했다. **1kHz ±100μs는 미달**이다. CPU 분리 설정도 이번 측정에서 개선을 보장하지 못했다.

A는 실제 영상 검출이지만 YOLO 신경망이 아니다. 원격 A/B 그대로의 성공이 아니라 아래 A 평면 검출·B 높이 오프셋 제안이 포함된 결과다. C의 기존 FR3/UR5e 공통화 변경은 보존했다. Git fetch만 수행했으며 main 병합·push·PR은 하지 않았다.

## 소스와 조건

원격 조직 저장소 main c5a2c8b796e57350336ae5e75e2455405b50fe8b, 로컬 C 기반 eaa7a492cb9468979e4c8d4e523500fd73ac9abf + 이전 로컬 변경. A/B 원격 파일을 별도 overlay에 추출하여 팀 작업 파일을 덮어쓰지 않았다. 원격 `nahyun/ai-inference`도 존재하여 읽었다. main에 실제 A 구현이 포함된 것을 확인했다.

Jazzy/Harmonic, Intel i5-1340P, Linux 7.0.0-31-generic PREEMPT_DYNAMIC, FIFO/메모리잠금 없음, ulimit -r=0. ROS_DOMAIN_ID=145. B 1ms, AI TTL200ms, B→C TTL5ms, C 상태 한계30ms, B 속도5mm/s·가속도10mm/s². C executor CPU15, IK wall budget500μs. 같은 PC monotonic 시계. Gazebo FK 오차는 실물·용접 정확도가 아니다.

철판 표면 (0.35,0,0.71), TCP 목표 (0.35,0,0.81), 카메라 (0.35,-0.6,1.5)/RPY(0,0.9212469046592403,π/2). UR5e 작업셀은 C 설치대+30cm 설정을 사용한다. 최종 실제 A→B→C UR5e 구동도 별도로 확인했다. UR5e 전체 고부하/지연 시험은 이번 quick 검증 범위에 포함되지 않는다.

## 실제 발견한 문제와 수정

1. 빨간 철판은 A의 회색 검출 대상과 맞지 않았다. FR3/UR5e SDF visual을 회색으로 맞췄고 형상은 유지했다.
2. 비스듬한 카메라에서 기존 깊이 중앙값 기준 검출은 표면을 약 (0.489,-0.017,0.708)로 오검출했다. B가 약14cm RANGE 초과를 차단했다. A 검증 사본에 알려진 map 작업대 높이로 깊이 평면을 분할하는 옵션을 추가했다. 목표 x/y를 입력하지 않고 영상에서 계산한다. 회색 P 검출은 손끝 가림으로 중심이 약2cm 치우쳐 V와 불일치했으며, P/V 허용 차이를 검증에서2mm로 설정해 V fallback을 사용했다.
3. 최신 B에는 작업 높이 오프셋이 없었다. B 검증 사본에 `work_height_offset_m`을 추가했다. raw 표면 좌표를 gate 검사한 뒤 통과 목표에만 +z10cm를 적용한다. 기본0·유한값[0,0.3] 검사. A/B 메시지 계약과 C 절대 TCP 의미는 유지했다.
4. 실행 중 A detector_mode 파라미터를 바꿔도 DetectorSwitch 내부 config는 다시 만들어지지 않았다. 첫 실패 로그의 primary 이름 구간은 실제 primary 시험으로 사용하지 않는다. 이번 최종 auto 설정은 노드 시작 때 지정했다. A 담당과 동적 설정 적용 범위를 정해야 한다.

A 제안은 수평·map축 정렬 사각 철판의 외곽이 보이는 조건에 한정한다. 회전·완전 가림·실센서 오차에 대한 일반 검출기로 검증하지 않았다. 동적으로 움직인 철판과 C의 초기 정적 collision snapshot이 같다고 가정하지 않는다. 이번 이동은 같은 z의 작은 범위로 시험했으며, 실제 공정에는 이동 범위를 포함한 장면 업데이트/보수적 swept obstacle 검증이 필요하다.

## 위치 보정과 응답

| 계획 TCP map [m] | 최종 오차 [mm] | 2mm 안 진입 후0.3초 유지 확인: 최초 진입 [s] | pose 요청→첫 A 검출 [ms] |
|---|---:|---:|---:|
| [0.36, 0.0, 0.81] | 1.058 | 2.241 | 173.4 |
| [0.34, 0.01, 0.81] | 1.055 | 4.307 | 126.3 |
| [0.35, 0.0, 0.81] | 1.063 | 3.011 | 129.2 |

오차는 알려진 철판 이동 위치+10cm와 C 측정 FK를 비교했다. 동작속도를5mm/s로 제한했으므로 이동 완료시간은 수초다. 전체 A→C 목표 채택과 이동 완료시간을 구분한다. 세 점 모두2mm/0.3초 기준 통과, 최종 오차 약1.06mm라 **제안서1mm/100ms 기준을 통과했다고 하지 않는다**.

## 제어 주기·추론·연계 지연

wake 지터는 B가 예정 시각보다 늦게 깨어난 시간이다. 실제 wake 간격 편차 통계도 metrics.json에 별도 기록했다. 평균·p99만으로 ±100μs를 판정하지 않는다.

| 설정 | 구간 | 중앙값 [μs] | p99 [μs] | 최대 [μs] | 건너뛴 주기 |
|---|---|---:|---:|---:|---:|
| B CPU 미지정 | no_A_baseline | 56.2 | 90.7 | 1199.0 | 1 |
| B CPU 미지정 | actual_A_plane_auto | 58.1 | 116.4 | 2928.8 | 19 |
| B CPU 미지정 | actual_A_plus_8_CPU_workers | 59.9 | 1071.6 | 4852.4 | 204 |
| B CPU14 / bridge12 / C15 | no_A_baseline | 55.7 | 493.9 | 3662.3 | 119 |
| B CPU14 / bridge12 / C15 | actual_A_plane_auto | 56.9 | 962.3 | 3810.8 | 233 |
| B CPU14 / bridge12 / C15 | actual_A_plus_8_CPU_workers | 62.3 | 2128.0 | 4661.4 | 568 |

A 추론(영상쌍 수신 이후 처리) 중앙값 28.06ms / p99 36.74ms / 최대 43.22ms. A는 약15Hz 카메라 입력, startup/지연/부하 구간은 그보다 적은 출력이 될 수 있다. 8개 CPU 작업자는 SHA256 반복 경쟁 부하이며 고부하 YOLO 모델 실행과 구분한다.

전체 CPU14 비교 실행 중 같은 AI 결과의 첫 B 반영은 중앙값 29.58ms, 첫 수신기 목표 채택 ACK를 C가 받은 시점은 중앙값 36.61ms / p99 71.27ms / 최대 84.57ms. 이 통계에는 startup·입력 지연·부하·복구에서 실제 반영된 결과가 포함돼 있으며, 모든 발행 결과가 반영됐다는 뜻은 아니다. A seq·B command_seq·C trace를 결합했다. A capture는 ROS 영상쌍 수신이며 카메라 노출 시작은 아니다. C t4는 adapter send 직전, t6는 목표 채택 ACK 수신, t5는 비어 있다. 실물 TCP E2E를 측정한 것이 아니다.

CPU14 배치에서 기본 구간 p99가 나빠졌고, 추가 부하에서도 최대4ms 이상이 남았다. 코어를 고정했다는 사실만으로 자원 분리·실시간 목표 달성이라고 하지 않는다. 성능코어CPU6 추가시험(`abc-actual-pcore`)에서는 이후 Gazebo multicast `Network is unreachable`·영상/상태 수신 중단 및 큰 시각 불연속이 기록돼 정상 성능 비교·C 종료 watchdog 판정에서 제외했다. 원인을 CPU 설정이라고 단정하지 않는다.

## 장애·복구 결과

- A output sleep200ms: 나이가200ms를 넘은 결과는 B HOLD. 지연을 제거하면 RUN 복귀.
- A (9.9,9.9,9.9): B RANGE/HOLD, 로봇 목표 급변 없음. 정상 입력으로 복귀.
- A output stop: TTL 만료 후 HOLD, 마지막 목표 유지. none 복귀 후 RUN.
- 8개 CPU 부하: 수신기 명령 거부(E_DEVICE)와 DECEL_STOP 발생. CPU14 실행에서는 active 이벤트1회/clear1회, 미지정 실행에서는3회/각 clear. `receiver rejected ... or hit a boundary`의 상세 원인은 현재 status가 이유를 구분하지 못하므로 TTL 만료라고 단정하지 않는다.
- C HAL 프로세스 종료(CPU14 정상 실행): 마지막 C 상태는 약3초 오래됨, B STATE_TIMEOUT→DECEL_STOP. receiver watchdog_active=true, commanded velocity 최대0. 단절은 목표 도착 후 정지 상태에서 주입했으며, 운동 중 정지거리 시험과 구분한다. 이 때 종료 전 마지막 TCP를 신선한 측정값으로 보지 않는다.
- C가 상태를 계속 발행하면서 하위 피드백을 잃은 별도 pcore 구간은 C E_STATE_STALE(8)/E_DISCONNECTED(9) 및 B FAULT로 기록됨. 그 구간의 watchdog 상태는 stale여서 성공 근거로 쓰지 않는다.

`abc-actual-full-v2`의 이동 중 일시적인 C deadline 오류/DECEL_STOP과 부하 오류도 원본 report에 보존했다. 정상 도착이 있었다는 이유로 전체 스트림 무오류라고 하지 않는다. 전체 시험의 실물 충돌/긴 경로/실시간 인증·C 재시작 복구를 완료한 것은 아니다.

## 검사·재현·팀 요청

최신 B 사본 단위 검사93개, 실제 영상 FR3/UR5e A 회귀 검사5개: 오류·실패0. 기존 공통 C 67개 검사는 앞선 공통 HAL 보고서를 따른다. 이번 visual 변경은 실제 Gazebo 영상 구동으로 확인했다.

1. A: 검증 월드 색·카메라 TF 일치, 비스듬한 카메라/가림을 다루는 검출, 신뢰도·회전/외곽가림 실패 정책, 동적 detector 설정 범위를 검토한다.
2. B: 표면 gate 후10cm offset 패치 검토. 제어 루프·bridge 코어는 실제 커널/CPU로 측정하여 정하고, PREEMPT_RT/FIFO 환경에서 주기·지터를 다시 검증한다. C 오류 카운팅/실제 정지 확인 후 재개 제안은 이전 보고서도 함께 확인한다.
3. C: TTL5ms 검사를 끄지 않는다. 수신기 거부 이유를 구조적으로 구분할 상태 필드, simulator/controller 자원 배치, 실제 측정 신선도·watchdog을 검증한다. TTL을 늘릴 경우 B와 C command_ttl_ns를 동시에 맞추고 장애 정지 지연도 다시 측정해야 한다. 현재5ms 유지.

검증 코드·A/B 제안 패치·재현 설명: 프로젝트 루트 `docs/abc-integration/README.md`, `checks/abc-integration/`. 원본 로그: `logs/abc-actual-full-v2/`, `logs/abc-actual-affinity-v2/`. 각 폴더 report.json/metrics.json과 A CSV·B rt.csv/ai_rx.csv·C hal.csv가 근거다. URI가 아닌 로컬 절대경로로 열 수 있다.

UR5e 첫 실제 A 시험에서 작은 비연결 깊이 조각이 철판 bbox 내부에 있어 평면 분할이 결과를 거부했다. 최대 연결 성분의 픽셀을 정확히 선택하도록 수정했고, 저장한 UR5e 영상 회귀 검사를 추가해 총5개 통과했다. `logs/abc-actual-ur5e`는 실패 이력이며 수정안의 실제 재구동 결과는 추가 기록한다.

## UR5e 실제 A/B/C 최종 추가 검증

`logs/abc-actual-ur5e-corrected`: 카메라/map 작업점·A/B 소스를 유지하고 C robot:=ur5e/session43으로 실행했다. A는 같은 실제 RGB-D 입력/평면 검출, B는 같은10cm offset이며 C만6축 설정·설치대+30cm·공식tool0/충돌설정으로 달라진다. 첫 미검출은 A 제안의 정확한 connected-component 선택으로 해결했다.

철판+1cm 이동: 최종 TCP 오차 1.059mm, 2mm 진입 2.195초 이후0.3초 유지. A stop/HOLD 및 resume/RUN, C 프로세스 종료 후 B DECEL_STOP 및 수신기 watchdog_active=true/명령 속도0 모두 통과. C 단절 주입 시에는 목표 도착 후 정지 상태였으며 운동 중 정지거리 시험은 아니다. 해당 이동·정상·stop/resume 구간 C 오류 이벤트0. 정상 A 구간 B 지터 p99 86.3μs/최대 790.6μs로 ±100μs는 미달. A 수신쌍→C ACK 수신 중앙값 32.12ms. 가짜 A UR5e 검증과 별도이며 실물 검증은 아니다.

## 최종 동일 소스 FR3 재구동

`logs/abc-actual-fr3-final`: UR5e 마지막 실행과 동일한 A/B 소스 해시로 FR3를 다시 실행했다. 철판+1cm 이동 오차 1.058mm, 2mm 진입 2.205초/0.3초 유지. A stop/HOLD, resume/RUN, C 단절 DECEL_STOP/watchdog 신호 모두 통과. 정상 A 구간 지터 p99 91.4μs/최대 3398.1μs. 이동/정상/stop/resume C 오류 이벤트0. 상위 A/B 코드의 로봇별 분기 변경 없이 C 프로필·session만 바꾼 기본 실증이다. 최종 두 실행 모두 quick 범위이며 고부하 비교는 앞선 FR3 전체 실행을 따른다.

공유용 제안 패치·소스 해시: `docs/proposals/abc-a-map-plane-proposed.patch`, `abc-b-height-offset-proposed.patch`, `abc-manifest.json`. 담당자 검토·병합 전 제안이며 적용 여부를 팀과 맞춘다.
