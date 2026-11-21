# 기여 가이드 (CONTRIBUTING)

> PAC 2026 팀 기여 규칙. 프로젝트 개요·공통 규약(좌표·단위·시간)·인터페이스 계약은 [README.md](README.md)를 기준으로 한다.

---

## 1. 브랜치 전략
- 각자 **자기 이름 브랜치**에서 작업한다. (예: `yeonj`, `nahyun`, `jaewon`)
- 작업 단위로 `main`에 **PR**을 올린다.
- `main`은 항상 빌드 가능한 상태를 유지한다. `main`에 직접 push하지 않는다.

## 2. PR 정책
- **최소 1명 리뷰 승인** 후 머지.
- 리뷰가 지연될 때는 셀프 머지 허용.
- **인터페이스 변경**(공유 메모리 레이아웃·토픽·메시지·좌표계·시간 기준) PR은 README도 함께 수정하고 전원에게 공지한다.

## 3. 커밋 메시지 규칙
형식: **`[type] 설명`** (예: `[feat] rt_control_loop 1ms 스케줄러 추가`)

| 타입 | 설명 |
| --- | --- |
| **feat** | 새로운 기능 추가 |
| **fix** | 버그 수정 |
| **docs** | 문서 수정 (README, 주석 등) |
| **style** | 포맷팅 등 동작 변화 없는 변경 |
| **refactor** | 기능 변화 없는 리팩토링 |
| **test** | 테스트 코드 추가·수정 |
| **chore** | 빌드·패키지 관리 작업 |
| **perf** | 성능 개선 |

## 4. 패키지 소유권

| 계층 | 담당 | 패키지 |
| --- | --- | --- |
| A. AI 추론 | 김나현 | `pac_ai_inference` |
| B. 실시간 제어 | 홍연주 | `pac_rt_control` |
| C. 통신·HAL | 김재원 | `pac_hal`, `intelion_interfaces` |
| 공통 인프라 | TBD | `pac_worlds`, `pac_description`, `pac_gz_bridge`, `pac_bringup` |

- 계층끼리는 **README 4장의 인터페이스로만 결합**한다. 다른 패키지의 내부 코드를 직접 import/호출하지 않는다.
- 남의 패키지를 고쳐야 하면 담당자와 PR로 협의한다.

## 5. 코드 규약
- **설명 가능한 것만 넣는다.** 넣은 코드·라이브러리·좌표 변환은 스스로 설명할 수 있어야 한다.
- 공통 규약(좌표·단위·CLOCK_MONOTONIC ns)은 함부로 바꾸지 않는다.
- **RT 스레드(B) 안에서는** 동적 메모리 할당, 파일 입출력, 잠금 대기, ROS 호출, 예외(throw)를 쓰지 않는다.
- Python `ament_flake8`, C++ `ament_cpplint` 통과 권장.

## 6. 작업 순서 (매번)

```bash
git checkout main && git pull          # ① 최신 main 받기
git checkout yeonj && git merge main   # ② 내 브랜치에 반영
# ③ 코드 작성
colcon build --symlink-install && source install/setup.bash   # ④ 빌드
# ⑤ 실행해서 확인
git add . && git commit -m "[feat] 설명"   # ⑥ 기록
git push origin yeonj                      # ⑦ 올리기 → GitHub에서 PR 생성
```

## 7. 라이선스
- **미지정**(대회용 내부 프로젝트). 각 `package.xml`의 `<license>` 필드는 `TODO`로 둔다.
