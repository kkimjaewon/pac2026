# 측정 기록

측정 도구: `build/rt_loop_demo` (1ms 주기, clock_nanosleep TIMER_ABSTIME)
목표: 주기 1ms, 지터 ±100us

## 2026-10-05 RT 커널 (6.8.1-1061-realtime), 부하 없음, 30초

PC: LG gram 16Z90SP. CPU 6·7 = 물리 코어 3 (하이퍼스레드 짝).

| 항목 | 옵션 없음 | FIFO80 + mlock + CPU 2 | FIFO80 + CPU 6 + 성능모드 |
|---|---|---|---|
| 명령 옵션 | (없음) | `--mlock --fifo --prio 80 --cpu 2` | `--mlock --fifo --prio 80 --cpu 6` |
| 전원 프로필 | (기록 없음) | (기록 없음) | performance |
| 실행한 주기 수 | 30000 | 30000 | 30000 |
| 주기 평균 (us) | 1000.098 | 1000.000 | 1000.000 |
| 주기 최소 / 최대 (us) | 230.273 / 4771.534 | 857.074 / 1134.992 | 967.943 / 1026.584 |
| 지터 평균 (us) | 147.017 | 8.691 | 3.847 |
| 최대 지터 (us) | 3835.883 | 150.507 | 36.418 |
| 지터 100us 초과 | 29582 (98.6067%) | 10 (0.0333%) | 0 (0.0000%) |
| 건너뛴 주기 (overrun) | 3 | 0 | 0 |

메모
- 옵션 없음: RT 커널이어도 일반 스레드로 돌리면 거의 모든 주기(98.6%)가 100us를 넘고, 최대 3.8ms까지 늦어짐. overrun 3회.
- FIFO80 + mlock + CPU 2: 대부분 안정적이지만 100us 초과가 10회, 최대 지터 150us로 목표를 살짝 넘음.
- FIFO80 + CPU 6 + 성능모드: 30000 주기 동안 100us를 넘은 지터와 overrun이 한 번도 없음. 최대 지터는 주기의 약 3.6%.
- 부하 없이 30초만 잰 값이라 최악의 경우를 대표하지는 않음. 다음에는 stress-ng 부하를 걸고 오래 돌려서 다시 잴 것.

## 비교 기준: 일반 커널 (6.8.0-146-generic), 부하 없음

| 항목 | 옵션 없음 | mlock + CPU 2 |
|---|---|---|
| 측정 시간 | 10초 | 3초 |
| 최대 지터 (us) | 304 | 351 |
| 지터 100us 초과 비율 | 6.3% | 0.97% |

메모
- 일반 커널에서는 옵션을 켜도 최대 지터가 300us 이상이라 목표(±100us)를 못 맞춤.
- RT 커널 + FIFO80 + CPU 6 + 성능모드 조합에서 처음으로 목표를 만족함 (위 표).
- 일반 커널 측정은 시간이 짧아(3~10초) 참고용.

## 2026-10-06 가짜 A → B → 가짜 C 연결 (ros_bridge + fake_c_node.py)

조건: RT 커널, 성능모드, `rt_control_node -p fifo:=true -p cpu:=6 -p mlock:=true -p "planned_position:=[0.5, 0.0, 0.71]"`,
가짜 A(15Hz, base_target [0.5, 0, 0.71]), 가짜 C(로봇 시작 [0.45, 0, 0.75], tau 0.02s, 상태 250Hz). RMW: rmw_fastrtps_cpp.
발행 스레드: FIFO 70(bridge_prio), 루프 CPU 6·7 밖. 루프 말고 다른 B 스레드는 CPU 6·7에서 내보냄.

| 항목 | 결과 |
|---|---|
| `ros2 topic hz /hal/final_command` | 997~1000 Hz |
| frame_id / expires − created | 모두 map / 모두 5,000,000 ns |
| 시작 위치 | "HalState 위치에서 시작 [0.4500, 0.0000, 0.7500]" → 약 0.5~0.84초 뒤 [0.5, 0, 0.71] 도착 |
| 1ms당 최대 이동 | 0.1000 mm (= v_max 0.1 m/s) |
| source_result_id | 받은 순간의 최근 A seq 3개 안에 있음 (수천 개 중 시작 직후 0~1개 예외) |
| 1ms 루프 최대 지터 (1초 창 최고값) | 30~83 us, overrun 0 (30초·45초 시험) |
| bridge 건너뛴 seq (정상 운전 중) | 0 |
| 가짜 C 폐기 | expired 30초에 약 3개 (파이썬 처리 지연, 생성 후 5.7~10ms에 처리) |
| fault_mode ik_fail / state_stop / none | 켜진 오류 [5] / 상태 나이 계속 증가 / 오류 꺼짐 |
| 가짜 C 없이 B만 | 2초 뒤 start_position에서 시작 + 경고 |

메모
- 처음엔 발행 스레드가 일반 스케줄링이라 가끔 0.5ms 넘게 늦게 깨서 seq를 초당 0~7개 건너뜀. 같은 시험에서 루프 최대 지터도
  200~400us까지 튐(다른 B 스레드가 CPU 6·7에 올라옴). 발행 스레드 FIFO 70 + 다른 스레드를 CPU 6·7에서 내보낸 뒤 둘 다 해결.
- 남은 문제: 시작 직후 첫 발행에서 발행 스레드가 약 333ms(또는 그 배수) 멈추는 일이 간헐적으로 있음 (seq 332~664개 건너뜀).
  구독자가 여럿(가짜 C + 다른 파이썬 구독자) 붙어 있을 때 재현되고, Fast DDS 공유메모리 전송을 끄면
  (`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`) 재현 안 됨. 1ms 루프는 영향 없음(지터·overrun 정상). 원인 조사 필요.

## 2026-10-06 safety_mode 시험 (가짜 C → 가짜 A → B, 120초 한 번 실행)

조건: `rt_control_node -p fifo:=true -p cpu:=6 -p mlock:=true -p "planned_position:=[0.5, 0.0, 0.71]"`,
가짜 A amp 0.04(sine일 때 accept_radius 5cm 안), 가짜 C 기본값. 출력 궤적은 /hal/final_command를 따로 기록해 확인.

| 시험 | B 로그 | 확인 |
|---|---|---|
| A stop → none | RUN → HOLD (EXPIRED), 약 5.6초 뒤 HOLD → RUN | HOLD 동안 출력 [0.5, 0, 0.71] 그대로, source_result_id 고정 |
| A bad_value → none | RUN → HOLD (RANGE), 약 1.5초 뒤 HOLD → RUN | HOLD 동안 출력 그대로 |
| A sine 중 C ik_fail → none | RUN → DECEL_STOP (C_ERROR[5]) → 오류 꺼지고 1.0초 뒤 limiter를 로봇 위치에서 다시 시작 → RUN | 0.082초 만에 멈춤(속도 약 0.03 m/s), 1초 최고 가속 0.5000 m/s², 재개 앞뒤 최대 한 걸음 0.019 mm |
| C state_stop → none | RUN → DECEL_STOP (STATE_TIMEOUT) → RUN | 재개 앞뒤 최대 한 걸음 0.0045 mm |
| C disconnect → none → reset_fault | RUN → FAULT (C_FATAL[9]), none 뒤에도 FAULT 유지, 서비스 호출 → 새 세션 → RUN | 재개 앞뒤 최대 한 걸음 0.0105 mm |
| C 없이 require_c_state:=false | RUN (기본 true면 DECEL_STOP, NO_C_STATE) | |

- 1ms 루프: 119초 동안 overrun 0, 1초 창 최대 지터 98.1us (80~100us인 초 6번). 이전 시험(최대 83us)보다 조금 큼
- bridge 건너뛴 seq: 120초에 7개 (대부분 `ros2 param set` 같은 CLI를 실행한 순간)
- c_state_timeout 20ms: 가짜 C(파이썬, 250Hz)에서 잘못된 STATE_TIMEOUT은 이번 시험에서 0번

## 2026-10-06 cycle_tracer 켜고 측정 (run_id 20261006_110750_check)

조건: 가짜 C → 가짜 A(amp 0.04) → B(`fifo·cpu 6·mlock`, planned [0.5, 0, 0.71], `run_tag:=check`).
정상 10초 → A stop 5초 → none 10초 → bad_value 5초 → none 10초 → sine 10초 (약 50초).
숫자는 rt.csv 전체 주기(49,783개)에서 직접 계산 (1초 창 최대값이 아님).

| 항목 | 결과 |
|---|---|
| rt.csv 줄 수 | 49,783 = 첫~마지막 깨어난 시각 49,782 ms + 1, cycle 0부터 빠짐없음 |
| trace 버린 줄 | 0 |
| overrun | 0 |
| 지터 평균 / 99% / 99.9% / 최대 | 4.14 / 8.48 / 13.26 / 35.39 us (100us 초과 0) |
| compute 평균 / 최대 | 0.61 / 10.37 us |
| 모드 | RUN → HOLD(EXPIRED) 5.2초 → RUN → HOLD(RANGE) 4.7초 → RUN |
| HOLD 동안 | 반영 0회, goal_y·y_y 변화 없음 |
| ai_rx.csv | 668줄 |

- 지난 safety 시험의 최대 98us(1초 창 값)보다 오히려 작음. 그때는 기록기·CLI가 많이 돌았음
- bridge 시작 직후 seq 332개 건너뜀(이전에 적은 Fast DDS SHM 시작 멈춤)이 이번에도 한 번 나옴

## 2026-10-06 발표용 기록 (run_id 20261006_112105_demo_hold)

가짜 A sine(amp 0.04) 15초 → stop 5초 → none 10초 → bad_value 5초 → none 10초. `tools/plot_rt.py`로 그래프.

| 항목 | 결과 |
|---|---|
| 주기 수 / overrun / trace 버린 줄 | 44,764 / 0 / 0 |
| 지터 평균 / 99% / 99.9% / 최대 | 4.30 / 9.00 / 17.74 / 46.58 us (100us 초과 0) |
| compute 최대 | 11.61 us |
| HOLD(EXPIRED) 15.81~20.88초 | y_y 변화폭 0.000 mm (+0.0365에 고정), 반영 0회 |
| HOLD(RANGE) 32.08~35.88초 | y_y 변화폭 0.000 mm (+0.0380에 고정), 반영 0회 |
| 재개 후 | 목표가 sine의 현재 값으로 한 번에 약 7cm 바뀌고, 출력은 v_max·a_max 안에서 약 1초 만에 따라잡음 |

- 시작 직후 HOLD(NO_DATA) 0.16초: B가 RUN이 된 순간 아직 A 명령을 받기 전

## 2026-10-06 Gazebo 부하 아래 지터 (단계별)

조건: 가짜 C → 가짜 A → B (`fifo·prio 80·cpu 6·mlock`, 성능모드, trace 켬). 다른 창에서 Gazebo(Franka FR3) 실행.
숫자는 `python3 tools/plot_rt.py <run_dir> --skip 15`로 rt.csv 전체 주기에서 계산. 안정 상태 = 시작 15초 이후.
조건은 run_tag 기준으로 적음 (malloc_tune 추가 전 측정).

| 단계 | run_id | 카메라 브리지 | 기록 시간 | 지터 평균 / 99% / 99.9% (us) | 최대 지터 전체 / 안정 상태 (us) | 100us 초과 전체 / 안정 상태 | compute 최대 (us) | overrun |
|---|---|---|---|---|---|---|---|---|
| 1. Gazebo 창 켬 | 20261006_143056_gazebo_load | (기록 없음) | 91.6 s | 23.47 / 73.05 / 105.76 | 214.95 / 214.95 | 162 / 135 | 112.26 | 0 |
| 3. 창 없음(headless) + 코어 격리 (isolcpus·nohz_full·rcu_nocbs 6,7) | 20261006_145458_isolated_headless | **없음** | 71.5 s | 11.18 / 33.21 / 53.73 | 430.41 / 430.41 | 33 / 13 | 14.12 | 0 |
| 4. 3 + CPU 6 C-state(state1~3) 끔 | 20261006_145733_isolated_nocstate | **없음** | 53.7 s | 4.81 / 23.10 / 39.14 | 383.58 / 214.10 | 18 / 3 | 12.17 | 0 |

- 2단계(창 켬 + Gazebo 코어 고정, 20261006_143728_gazebo_pinned)는 표에서 뺌. 같은 시간에 run 20261006_143719도 돌고 있어서
  (143728 전체가 143719 안) B 노드 두 개가 동시에 CPU 6을 썼음. 143728만 보면 최대 166.81us, 100us 초과 167번 (참고용)
- 4단계 100us 초과 18번 중 15번이 시작 후 15초 안 (그 뒤 약 40초에 3번, 최대 214us)
- 원인(추정): 1~2단계는 Gazebo 렌더링이 같은 코어·캐시·메모리 대역을 씀(평균·99%가 큼), 3~4단계에 남은 큰 튐은
  같은 프로세스의 다른 스레드가 메모리를 OS에 돌려줄 때(munmap·trim) 생기는 TLB shootdown 인터럽트가 CPU 6에 오는 것으로 의심
- 조치: Gazebo headless + 코어 격리 + C-state 끔(평균·99% 개선), `malloc_tune`(mallopt M_TRIM_THRESHOLD 끔, M_MMAP_MAX 0) 추가.
  malloc_tune 켠 뒤 같은 4단계 조건으로 다시 재서 시작 15초 안의 튐이 줄었는지 확인할 것

## 2026-10-06 최종 측정 (Gazebo headless + 카메라 브리지 + 카메라 구독 + 코어 격리 + C-state 켬)

### cyclictest

명령: `cyclictest -m -p 80 -a 6 -i 1000 -D 90`

| 항목 | 결과 |
|---|---|
| Avg / Max | 3 us / 175 us |

### B 1ms 루프

숫자는 rt.csv 전체 주기에서 직접 계산. "15초 이후"는 시작 직후 구간을 뺀 값.

| run_id | 카메라 브리지 | 구간 | 지터 99.9% | 최대 지터 | 100us 초과 | overrun |
|---|---|---|---|---|---|---|
| 20261006_153926_final_conditions | **없음** | 전체 | 53.6 us | 412.7 us | 31번 | 0 |
| 20261006_160332_final_with_camera | 있음 | 전체 | 37.6 us | 138.9 us | 5번 | 0 |
| 20261006_160332_final_with_camera | 있음 | 15초 이후 | 33.8 us | 101.4 us | 1번 | 0 |

메모
- 3·4단계 측정과 final_conditions는 **카메라 브리지가 빠진 상태**로 잰 값. 카메라까지 포함한 최종 조건 값은 final_with_camera.
- final_conditions 값은 이 표를 쓰면서 rt.csv에서 다시 계산함.
- final_with_camera는 측정 내내 CPU 6 deep C-state(state1~3)가 켜져 있었음 (meta.yaml `deep_cstates_on: [1, 2, 3]`, 측정 중 끈 적 없음).

## 2026-10-08 INIT 유지 명령(warmup_hold_ms)이 첫 발행 멈춤을 흡수하는지 (커밋 52a9fb7)

조건: fake_c + fake_a + B (launch 기본 순서, B는 launch +2초), ROS_DOMAIN_ID=87·LOCALHOST, Release, 20초씩.
멈춤을 일으키려고 launch +0.5초에 `/hal/final_command` 추가 구독자 4개(`ros2 topic echo`)를 띄움 (B 시작 무렵 discovery가 몰리게).
warmup_hold_ms만 바꾼 yaml 복사본으로 비교: **0 = 수정 전처럼 첫 발행 직후 RUN**, **100 = 수정본**.
"RUN까지" = B 첫 명령부터 RUN 진입까지. 멈춤 시점은 fake_c의 `오류 3 켜짐 (N ms 지남)` 시각을 rt.csv의 RUN 진입 cycle에 맞춰 계산 (±10ms).

| run_id | hold | B 시작 → 첫 명령 | RUN까지 | 최대 publish() | 발행 overrun | 멈춤 시점 (RUN 기준) | DECEL_STOP | 흡수 |
|---|---|---|---|---|---|---|---|---|
| 20261008_193047_h0s4a | 0 | 1880 ms | 344 ms | 333.7 ms | 333 | +307 ms (RUN) | 1 (STATE_TIMEOUT) | - |
| 20261008_193452_h0s4_1 | 0 | 2476 ms | 14 ms | 333.9 ms | 666 | +101 ms (RUN) | 1 (STATE_TIMEOUT) | - |
| 20261008_193517_h0s4_2 | 0 | 2083 ms | 15 ms | 4.4 ms | 4 | 없음 | 0 | - |
| 20261008_193542_h0s4_3 | 0 | 2263 ms | 13 ms | 666.2 ms | 667 | +320 ms (RUN) | 1 (STATE_TIMEOUT) | - |
| 20261008_193606_h0s4_4 | 0 | 66 ms | 101 ms | 333.7 ms | 333 | +3153 ms (RUN) | 0 | - |
| 20261008_193226_h100s4_1 | 100 | 2292 ms | 103 ms | 0.6 ms | 1 | 없음 | 0 | (멈춤 없음) |
| 20261008_193251_h100s4_2 | 100 | 55 ms | 438 ms | 333.1 ms | 333 | 첫 5번 publish 안 (INIT) | 0 | **예** |
| 20261008_193315_h100s4_3 | 100 | 1798 ms | 102 ms | 333.4 ms | 333 | +941 ms (RUN) | 1 (STATE_TIMEOUT) | **아니오** |
| 20261008_193340_h100s4_4 | 100 | 2427 ms | 102 ms | 1.4 ms | 2 | 없음 | 0 | (멈춤 없음) |
| 20261008_193405_h100s4_5 | 100 | 1989 ms | 102 ms | 1.0 ms | 1 | 없음 | 0 | (멈춤 없음) |

추가 구독자 없이 hold 100 (20261008_192922_repro0): B 시작 → 첫 명령 66 ms, RUN까지 102 ms, 최대 publish() 0.3 ms, 멈춤 없음.
다만 RUN 약 1.9초 뒤 STATE_TIMEOUT DECEL_STOP 한 번 (B 발행은 정상, fake_c 1초 요약이 125ms 늦고 받음 792개 -> 가짜 C 쪽 멈춤).

메모
- 멈춤은 거의 항상 **333ms(또는 666ms)**. 2026-10-06 "남은 문제"(공유메모리 전송, 구독자 여럿)와 같은 모양
- h100s4_2: 멈춤이 INIT 유지 명령 중에 나서 overrun으로 WARM_HOLD·WARM_PUBLISH가 다시 셈 -> RUN 438ms, DECEL_STOP 없음. 수정이 의도대로 동작
- 하지만 멈춤은 **첫 publish에만 생기지 않음**: 첫 명령 뒤 0.1~3초 아무 때나 남 (h0s4_4 +3.2초, h100s4_3 +0.94초).
  추가 구독자가 붙는 시각(echo가 처음 받은 seq로 추정)과 딱 맞지도 않음
- 멈춤 때 fake_c의 HalState도 같이 끊기는 경우가 많음 (DECEL_STOP 이유가 모두 STATE_TIMEOUT) -> B 혼자가 아니라 DDS 전송 쪽이 같이 막히는 것으로 보임
- 결론: warmup_hold_ms 100은 "B 시작 직후 100ms 안에 나는 멈춤"만 흡수. 늦게 나는 멈춤은 못 막음 -> 원인(공유메모리 전송) 쪽을 따로 봐야 함

## 2026-10-08 같은 재현 조건 + Fast DDS UDPv4만 (`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`)

조건은 바로 위 표와 같음 (추가 구독자 4개, warmup_hold_ms 0·100, 20초씩). 환경 변수는 실행 스크립트에서 export해
B·fake_a·fake_c·추가 구독자 모두에 걸림 (실행마다 6초 무렵 `/proc/<pid>/environ`으로 11개 프로세스 모두 UDPv4 확인).
멈춤 시점: fake_c `오류 3` 시각으로 계산. fake_c가 끊김을 못 남긴 h100_1·h100_5는 B 1초 요약의 창별 발행 수와
fake_c 1초 수신 수로 범위만 추정 (그 동안 fake_c도 같이 멈춘 것으로 보임).

| run_id | hold | RUN까지 | 최대 publish() | 멈춤 시점 (RUN 기준) | DECEL_STOP |
|---|---|---|---|---|---|
| 20261008_193941_udp_h0_1 | 0 | 15 ms | 0.47 ms | 없음 | 0 |
| 20261008_194012_udp_h0_2 | 0 | 97 ms | 1.13 ms | 없음 | 0 |
| 20261008_194036_udp_h0_3 | 0 | 16 ms | 333.1 ms | +466 ms (RUN) | 2 (STATE_TIMEOUT: +489 ms, +11.9초는 C 쪽 9 ms 끊김) |
| 20261008_194102_udp_h0_4 | 0 | 16 ms | 1.02 ms | 없음 | 0 |
| 20261008_194126_udp_h0_5 | 0 | 16 ms | 0.82 ms | 없음 | 0 |
| 20261008_194151_udp_h100_1 | 100 | 101 ms | 333.7 ms | 약 +100~430 ms (RUN, 창 추정) | 0 |
| 20261008_194216_udp_h100_2 | 100 | **437 ms** | 335.7 ms | **#1 publish 335.7 ms (INIT, 흡수)** + 다시 +348 ms (RUN) | 1 (STATE_TIMEOUT) |
| 20261008_194241_udp_h100_3 | 100 | 101 ms | 0.80 ms | 없음 | 0 |
| 20261008_194305_udp_h100_4 | 100 | 102 ms | 1.19 ms | 없음 | 0 |
| 20261008_194330_udp_h100_5 | 100 | 101 ms | 333.3 ms | 약 +10~665 ms (RUN, 창 추정) | 0 |

공유메모리(기본)와 비교 (333 ms 멈춤이 난 실행 수)

| 전송 | hold 0 | hold 100 | 합계 | RUN 중 멈춤 | 멈춤 때문에 난 DECEL_STOP |
|---|---|---|---|---|---|
| 공유메모리 + UDP (기본) | 4/5 | 2/5 | 6/10 | 5 | 4 |
| UDPv4만 | 1/5 | 3/5 | 4/10 | 4 | 2 |

평소 값 (1초 요약 중 RUN이고 건너뛴 seq가 0인 창만, 처음 RUN 창 제외, 실행마다 12~16개 창)

| 항목 | 공유메모리 (10번) | UDPv4 (10번) |
|---|---|---|
| 루프 최대 지터, 창별 중앙값 | 38.5~55.8 us | 42.5~58.0 us (h100_1만 107.5 us) |
| 루프 최대 지터, 실행별 최대 | 53.0~102.4 us | 52.9~81.9 us (h100_1 208.4, h100_5 148.6 us) |
| B 발행 수 / 초 (평균) | 999.7~1000.4 | 999.6~1000.4 |
| fake_c 받음 / 초 (평균) | 902~975 | 900~957 |
| fake_c 짧은 끊김(오류 3, 100 ms 미만), 실행별 | 0~2번, 최대 5.2 ms | 0~4번, 최대 9.4 ms |
| 첫 발행 #2~5 publish() 최대 (멈춤 실행 빼고) | 19.1~49.5 us | 11.8~64.3 us |
| 최대 publish() 누적 (멈춤 없는 실행만) | 0.65~4.4 ms (4번) | 0.47~1.19 ms (6번) |

메모
- **UDPv4만으로는 333 ms 멈춤이 없어지지 않음** (10번 중 4번). 2026-10-06 메모("UDPv4면 재현 안 됨")는 이 조건(추가 구독자 4개)에서는 안 맞음.
  모양도 같음: 거의 정확히 333 ms, B와 fake_c가 같이 막힘 -> 공유메모리 전송만의 문제는 아닌 것으로 보임
- udp_h100_2: 첫 publish() 자체가 335.7 ms 막혔고 INIT 유지 명령이 흡수함 (RUN 437 ms). 하지만 RUN +348 ms에 한 번 더 막혀 DECEL_STOP
- 1초 요약의 "최대 publish()"는 시작부터 누적한 최대값이라 창별 평소 publish() 시간은 알 수 없음. 그래서 첫 발행 측정(#2~5)과
  멈춤 없는 실행의 누적 최대로 비교함. 두 전송 모두 평소 publish()는 수십 us, 가끔 1 ms 안팎
- UDPv4에서 평소 성능이 뚜렷하게 나빠지지는 않음 (발행 수·C 수신 수·지터 범위 비슷). UDPv4의 짧은 끊김이 조금 많음 (최대 9.4 ms vs 5.2 ms).
  udp_h100_1의 높은 지터(중앙값 107 us)는 그 실행 내내 높았고 다른 UDPv4 실행에서는 안 나타남. 1ms 루프는 DDS를 직접 부르지 않아 전송 탓인지는 불확실
- 표본이 실행 5번씩이라 멈춤 빈도(6/10 vs 4/10) 차이는 우연 범위 안

## 2026-10-08 추가 구독자를 B 시작 5초 전에 미리 띄운 조건 (hold 100, 공유메모리 기본)

조건: 추가 구독자 4개(`ros2 topic echo /hal/final_command`)를 launch 3초 전(B 시작 5초 전)에 띄우고, 그 뒤로는 아무것도 새로 띄우지 않음.
launch 직전 `ros2 topic info`로 매번 Subscription count 4 확인. warmup_hold_ms 100, 나머지는 위와 같음.
"구독자 첫 수신" = 각 구독자가 처음 받은 command_seq (1이면 B 첫 명령부터 받음).

| run_id | RUN까지 | 최대 publish() | 333 ms 멈춤 | 멈춤 시점 | DECEL_STOP | 구독자 첫 수신 seq |
|---|---|---|---|---|---|---|
| 20261008_194754_pre_h100_1 | 436 ms | 333.5 ms | **예 (2번)** | #1 publish (INIT, 흡수) + RUN +341 ms | 1 (STATE_TIMEOUT) | 1, 1, 1, 1 |
| 20261008_194833_pre_h100_2 | 102 ms | 0.40 ms | 아니오 | - | 0 | 1, 1, 1, 1 |
| 20261008_194904_pre_h100_3 | 102 ms | 1.03 ms | 아니오 | - | 0 | 1, 66, 344, 145 |
| 20261008_194937_pre_h100_4 | 102 ms | 2.00 ms | 아니오 | - | 2 (STATE_TIMEOUT, C 쪽) | 1, 1, 1, 1 |
| 20261008_195008_pre_h100_5 | 102 ms | 0.42 ms | 아니오 | - | 0 | 1, 1, 1, 1 |

메모
- 미리 띄워도 **333 ms 멈춤이 5번 중 1번** 남. pre_h100_1은 udp_h100_2와 같은 모양: 첫 publish() 333 ms(INIT 유지 명령이 흡수)
  -> RUN 진입 약 340 ms 뒤 한 번 더 333 ms -> DECEL_STOP
- 구독자 4개가 RUN 전부터 데이터를 받고 있던 실행(pre_h100_1)에서도 RUN 중 멈춤이 났으므로, "RUN 중 새 구독자가 붙어서"만으로는 설명 안 됨
- pre_h100_3은 launch 전 목록에는 4개였지만 2개는 B 명령을 RUN 뒤(+42 ms, +241 ms)에 처음 받음. B 쪽 연결은 B가 뜬 뒤에야
  맺어지므로 미리 띄워도 B 시작 직후 연결은 피할 수 없음. 이 실행은 멈춤 없음
- pre_h100_4의 DECEL_STOP 2번은 B 발행은 정상(최대 2.0 ms)이고 fake_c 쪽 끊김(205 ms, 4 ms) -> 333 ms 멈춤과는 다른 것 (위 repro0과 같은 가짜 C 쪽 멈춤)

## 2026-10-08 warmup_hold_ms 1500 (같은 재현 조건: 추가 구독자 4개, launch +0.5초, 공유메모리 기본)

warmup_hold_ms만 1500으로 바꾼 yaml 복사본. 나머지 조건·계산 방법은 위 표들과 같음. "RUN까지" = B 첫 명령부터 RUN 진입.
멈춤 수는 발행 overrun(333개 = 한 번)으로 셈. INIT에서 HalState가 끊겨 유지 명령을 안 쓰는 동안 난 멈춤은 overrun만 늘고 건너뛴 seq는 안 늚.

| run_id | RUN까지 | 최대 publish() | 333 ms 멈춤 (INIT / RUN) | RUN 중 멈춤 시점 | DECEL_STOP | 흡수 |
|---|---|---|---|---|---|---|
| 20261008_195432_h1500_1 | 2280 ms | 333.1 ms | 2 / **2** (overrun 1329, 건너뛴 seq 705) | +254 ms, +838 ms | 1 (STATE_TIMEOUT) | **아니오** |
| 20261008_195506_h1500_2 | 2556 ms | 333.3 ms | 1 / 0 (첫 명령 뒤 +0.17~1.17초 창, INIT) | - | 0 | 예 |
| 20261008_195530_h1500_3 | 2216 ms | 333.5 ms | 1 / 0 (RUN 기준 -1838 ms) | - | 0 | 예 |
| 20261008_195555_h1500_4 | 1502 ms | 0.69 ms | 0 / 0 | - | 1 (STATE_TIMEOUT +14.8초, C 쪽) | (멈춤 없음) |
| 20261008_195620_h1500_5 | 1501 ms | 1.07 ms | 0 / 0 | - | 0 | (멈춤 없음) |

메모
- hold 1500에서도 RUN 중 멈춤이 5번 중 1번 남. h1500_1은 INIT에서 2번 멈춘 뒤(HalState도 2번 끊김) RUN에서 또 2번 멈춤 -> 유지 시간을 늘려도 막히지 않음
- 멈춤이 INIT 안에서 한 번만 난 실행(h1500_2, h1500_3)은 흡수됨. 대신 RUN 진입이 2.2~2.6초로 늦어짐
- h1500_4의 DECEL_STOP은 B 발행 정상(최대 0.69 ms), C 상태 끊김 -> 333 ms 멈춤과 다른 것. 같은 실행의 HOLD(EXPIRED)는 A 쪽
- RUN까지가 3초를 넘는 실행은 없었지만 warmup_max_s(3.0)에 가까움 (B 시작부터 세면 3.6~4.4초인 실행이 있음)

### B 프로세스 스레드 (위 5번 실행마다 launch +10초, RUN 중에 기록)

명령: `ps -eLo pid,tid,comm,psr,cls,rtprio` (B pid만), 스레드마다 `taskset -p`·`taskset -pc`. 5번 모두 스레드 19개, 이름·정책별 개수 같음.
아래는 20261008_195432_h1500_1 값 (PSR = 기록한 순간 돌던 CPU).

| 스레드 이름 | 개수 | 정책 / 우선순위 | affinity (taskset -pc) | 마스크 (taskset -p) |
|---|---|---|---|---|
| rt_control_node (1ms 루프) | 1 | **FIFO 80** | **6** | 40 |
| rt_control_node (발행 스레드, bridge) | 1 | FIFO 70 | 0-5,8-17 | 3ff3f |
| rt_control_node (메인·executor 등) | 7 | TS (일반) | 0-5,8-17 | 3ff3f |
| rt_control_-ust (LTTng) | 2 | TS | 0-5,8-17 | 3ff3f |
| dds.shm.wdog, dds.ev.0, dds.asyn.0.0 | 3 | TS | 0-5,8-17 | 3ff3f |
| dds.udp.* (29174, 29175, 7008) | 3 | TS | 0-5,8-17 | 3ff3f |
| dds.shm.* (29175, 7008) | 2 | TS | 0-5,8-17 | 3ff3f |

- **DDS 스레드는 CPU 6에 없음**: 8개 모두 affinity 0-5,8-17 (CPU 6·7 제외). CPU 6에 묶인 스레드는 1ms 루프 하나뿐
- 발행 스레드(FIFO 70)는 CPU 6·7 밖에서 돌고, 5번 동안 PSR 11·12·13·15에서 봄
- DDS 스레드는 모두 일반 스케줄링(TS). 발행 스레드(FIFO 70)가 publish() 안에서 DDS 스레드가 쥔 잠금을 기다리는 구조라
  우선순위 역전 가능성은 있음 (이번 측정으로 확인한 것은 아님)
