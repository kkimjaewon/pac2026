# intelion_interfaces

C(김재원) 주도 공통 메시지. 사용자 제공 인터페이스 문서 v0.3의 필드·상수 그대로 구현한다.

| 파일 | 연결 | 토픽 | QoS |
|---|---|---|---|
| ControlCommand.msg | A → B | /ai/control_command | Best effort / Volatile / Keep last 5 |
| HalCommand.msg | B → C | /hal/final_command | Best effort / Volatile / Keep last 1 |
| HalState.msg | C → B | /hal/state | Best effort / Volatile / Keep last 1 |
| HalError.msg | C → B | /hal/error | Reliable / Volatile / Keep last 32 |

- B→C는 PC-RT 같은 호스트의 CLOCK_MONOTONIC ns. header.stamp는 시뮬레이션 시간 참고용.
- A→B의 age_ns는 A 내부 경과 시간이다. PC 간 monotonic 절대값을 비교하지 않는다.
- HalCommand 목표는 map TCP 절대 pose, m, quaternion xyzw. 유효기간은 5ms.
- HalState의 pose_valid/twist_valid=false이면 숫자 값을 사용하지 않는다.
- HalState.CMD_SENT는 어댑터 전송 의미이며 로봇 적용·목표 도달 ACK가 아니다.
- source_result_id=0은 AI 입력 없음과 첫 AI seq=0이 겹친다. v0.3 그대로 유지하고 개선은 팀 합의 사항이다.

C++: `#include <intelion_interfaces/msg/hal_command.hpp>`
Python: `from intelion_interfaces.msg import ControlCommand, HalCommand, HalState, HalError`

상세 계약·설정·사용법: `../../docs/hal-interface-v03.md`.
