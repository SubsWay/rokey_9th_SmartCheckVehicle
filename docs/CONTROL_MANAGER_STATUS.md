# 관제 A 구현 현황 및 PM 인수인계

- 작성일: 2026-10-12
- 담당: 관제 A (`inspection_manager`)
- 브랜치: `feature/control`
- 기준 HEAD: `7da29cc` (`main e24ffd5` 동시 주행 시나리오 반영)
- 인수인계 사유: 담당자 건강상 조퇴
- Git 상태: 아래 구현 변경은 **아직 커밋·push하지 않은 로컬 변경**
- **10/13 갱신 (현준)**: `temp/bundle-test-1012`(`faa34c4`)와 `feature/driving`(`630f07a`)을
  `feature/control`에 병합했다(`200f4f3`·`e1b27ee`, 10/13 밤 push). 2절 표, 5.7절, 6절의 「10/13」 줄,
  7절이 병합 뒤 상태다. 1·3·4·8절과 5.1–5.6절은 10/12 인수인계 당시 기록 그대로다.
  남은 일은 `docs/control/PLAN.md`를 따른다.

## 1. 요약

10/11의 `robot4 측정 → robot3 설비 점검 → robot4 재측정` 시나리오는
폐기하고, 10/12 PM 공지의 두 로봇 동시 주행 시나리오에 맞춰 관제 A를
전환했다.

현재 다음 구간은 구현과 mock ROS 시험이 완료됐다.

```text
robot4: Z1 SURVEY ──→ DOCK ──┐
                              ├─→ WAITING_CANDIDATES
robot3: Z2 SURVEY ──→ DOCK ──┘
```

후보 수신·로봇별 재측정 작업 생성 로직도 구현했지만, RSSI 팀이 후보 분석을
완료했다는 신호가 아직 없으므로 운영 코드에서는
`WAITING_CANDIDATES` 이후를 자동으로 시작하지 않는다. 시간 지연이나 임의의
timeout으로 완료를 추정하지 않고 명시적인 완료 신호를 기다리는 상태다.

## 2. 현재 순찰 단계

관제 내부 단계는 다음과 같다.

```text
SURVEY
  → WAITING_CANDIDATES
  → REINSPECTION
  → WAITING_KEEPOUT
  → FACILITY
  → COMPLETED
```

| 단계 | 현재 구현 상태 | 진입·종료 조건 |
| --- | --- | --- |
| `SURVEY` | 구현·ROS 시험 완료 | 두 로봇에 섹터 측정 후 Dock 배정. 경로는 벽에서 0.45 m 안쪽, 줄은 칸 중심 |
| `WAITING_CANDIDATES` | 구현·mock 시험 완료 (10/13) | 두 로봇의 측정·Dock 종료 후 진입. `cell_map_writer`의 `commit_Z1`·`commit_Z2`를 부르고, 답이 오거나 10초가 지나면 칸 맵을 읽어 재측정 칸을 고른다 |
| `REINSPECTION` | 구현·mock 시험 완료 (10/13) | 미검출 → 음영 순, 로봇마다 최대 5칸 (미측정 칸은 10/14 부터 대상에서 뺌, `include_unmeasured:=true` 로 되돌릴 수 있음). 결과로 칸 상태를 다시 써서 `CLEARED`·`REMAINS` 알림 |
| `WAITING_KEEPOUT` | 구현·mock 시험 완료 (10/13) | 재측정·Dock 종료 또는 후보 0건일 때 진입. 다음 dispatch 주기에 바로 `FACILITY`로 |
| `FACILITY` | 구현·mock 시험 완료 (10/13) | 남은 칸(표본 2개 이상, Dock 반경 0.6 m 제외)을 두 로봇 마스크로 내고, 관측 자리가 든 섹터의 로봇에 점검·Dock 배정 |
| `COMPLETED` | 구현·mock 시험 완료 (10/13) | 모든 작업 종료 후 회차를 닫고(`done`·`partial`) 마스크를 비운다 |

단계는 회차 전체에 하나다. 먼저 끝난 로봇은 다른 로봇을 기다린다 (`docs/control/PLAN.md` 3-1).
실기로는 10/12 밤 robot4 한 대로 재측정 대상 선정까지만 확인됐다.

## 3. 완료한 구현

### 3.1 두 로봇 동시 dispatch

- 전역 작업 한 건만 실행하던 제한을 제거했다.
- `ManagerCore.next_pending_task(robot_id)`가 로봇별 실행 상태와
  `preferred_robot_id`, `allowed_robot_ids`를 검사한다.
- 한 로봇에는 한 번에 한 작업만 전송하면서 다른 로봇은 독립적으로 작업할
  수 있다.
- 기본 섹터 매핑은 다음 파라미터 순서를 사용한다.

```text
robot_ids = [robot4, robot3]
zone_ids  = [Z1,     Z2]
```

### 3.2 1차 측정과 Dock

`/inspection/start`를 호출하면 다음 4개 작업을 생성한다.

```text
robot4: run-<id>-robot4-survey
robot4: run-<id>-robot4-survey-dock
robot3: run-<id>-robot3-survey
robot3: run-<id>-robot3-survey-dock
```

- 두 `SURVEY_SEGMENT`는 동시에 실행된다.
- 각 로봇은 자기 Survey가 끝난 뒤 `DOCK`을 실행한다.
- 두 로봇 작업이 모두 종료되면 회차를 완료하지 않고
  `WAITING_CANDIDATES`로 전환한다.

### 3.3 후보와 재측정

- `/inspection/candidates`의 네트워크 후보만 처리한다.
- 다른 `run_id`, 중복 `candidate_id`, 알 수 없는 `robot_id`는 무시한다.
- `Candidate.robot_id`를 후보를 측정한 로봇으로 해석한다.
- 후보는 분석 완료 신호가 올 때까지 `pending_candidates`에 보관한다.
- `begin_reinspection_phase()`는 다음 작업을 만든다.
  - 후보 한 건당 해당 로봇의 `REINSPECTION` 한 건
  - 해당 로봇의 모든 후보 뒤 `DOCK` 한 건
- 후보가 없는 로봇은 재측정 순찰을 건너뛴다.
- 후보가 전체 0건이면 바로 `WAITING_KEEPOUT`로 전환한다.

후보를 다중 waypoint 한 작업으로 묶지 않은 이유는 현재 주행팀
`mission_executor`가 REINSPECTION의 마지막 waypoint에서만
`survey/precise`를 한 번 호출하기 때문이다.

### 3.4 RobotStatus와 OFFLINE 처리

- `/robot3/status`, `/robot4/status`를 각각 수신한다.
- 기본 timeout은 29초다.
- OFFLINE 전환 시 해당 로봇의 `ASSIGNED`·`RUNNING` 작업을
  `FAILED(reason=OFFLINE)`로 종료한다.
- 해당 goal을 `active_goals`에서 제거하고 취소를 요청한다.
- 늦게 도착한 goal 응답이나 결과가 `FAILED`를 다시 덮어쓰지 못하게 했다.
- 다른 로봇의 dispatch는 계속 진행할 수 있다.
- OFFLINE 로봇의 후속 고정 작업은 로봇이 복귀하거나 PM 정책이 결정될 때까지
  대기한다.

### 3.5 비전 결과 수신

- `/robot3/inspect/result`와 `/robot4/inspect/result`를 모두 구독한다.
- `NORMAL` → `INSPECTION_NORMAL`
- `UNCONFIRMED` → `E6`
- 그 외 이상 판정 → `CONFIRMED`
- 동일 판정 결과는 중복 처리하지 않는다.

### 3.6 BLOCKED·비상정지

- `BLOCKED`뿐 아니라 주행 노드의 실제 형식인 `BLOCKED skipped=N`도
  `TaskState.BLOCKED`와 E1로 처리한다.
- 비상정지 시 두 로봇의 실행 goal을 모두 취소한다.
- 재개 시 취소 작업을 다시 PENDING으로 바꾸며 로봇별로 한 번만 재전송한다.

## 4. 변경 파일과 Git 상태

현재 다음 파일이 수정됐으며 아직 커밋되지 않았다.

```text
M docs/CONTROL_MANAGER_STATUS.md
M docs/daily/lhm3185/2026-10-12.md
M ros2_ws/src/scv_manager/scv_manager/inspection_manager.py
M ros2_ws/src/scv_manager/scv_manager/manager_core.py
M ros2_ws/src/scv_manager/test/data/zones.yaml
M ros2_ws/src/scv_manager/test/mock_execute_task_server.py
M ros2_ws/src/scv_manager/test/test_inspection_manager.py
M ros2_ws/src/scv_manager/test/test_manager_core.py
M ros2_ws/src/scv_msgs/msg/Candidate.msg
```

`Candidate.msg`는 ABI 변경 없이 설명 주석만 새 시나리오에 맞게 수정했다.

다음 담당자는 먼저 아래 명령으로 변경 내용을 확인해야 한다.

```bash
cd /home/hymi/B_0102_Final_Project
git status --short
git diff --check
git diff
```

## 5. 시험 결과

### 5.1 빌드와 단위 테스트

빌드 성공 패키지:

```text
scv_msgs
scv_bringup
scv_manager
```

단위 테스트 결과:

```text
22 passed, 0 failed
```

포함된 시험:

- 작업 상태 전이
- 로봇별 동시 작업 선택
- 다른 로봇의 고정 작업 선택 방지
- 비상정지와 재개
- OFFLINE 경계와 온라인 복귀
- OFFLINE 로봇의 실행 작업 실패 처리
- `BLOCKED skipped=N` 처리
- 시작 시각 기반 `run_id`
- 구역 scanline 경로 생성
- 두 로봇의 `SURVEY → DOCK` 작업 생성
- 후보 중복·다른 run·알 수 없는 로봇 거절
- 후보를 측정 로봇에 고정한 재측정·Dock 생성
- 후보 0건 재측정 생략
- Survey 종료 후 후보 분석 대기 전환

재현 명령:

```bash
cd /home/hymi/B_0102_Final_Project
source /opt/ros/jazzy/setup.bash
colcon build --packages-select scv_msgs scv_bringup scv_manager --symlink-install
source install/setup.bash
colcon test --packages-select scv_manager --event-handlers console_direct+
colcon test-result --verbose
```

### 5.2 두 로봇 동시 Survey·Dock ROS 왕복

mock이 `/robot3/status`, `/robot4/status`를 1 Hz로 발행하고
`require_robot_ready=true`인 상태에서 시험했다.

확인된 흐름:

```text
robot4 Z1 SURVEY 시작 ─┐
                       ├─ 동시 실행
robot3 Z2 SURVEY 시작 ─┘

robot4 SURVEY 완료 → DOCK
robot3 SURVEY 완료 → DOCK
두 로봇 DOCK 완료 → WAITING_CANDIDATES
```

- 시험 run: `1791783098`
- 결과: 성공

### 5.3 후보 수신

동일 run에 다음 후보를 실제 ROS 토픽으로 주입했다.

```text
candidate-z1-01 → robot4
candidate-z2-01 → robot3
```

두 후보 모두 해당 로봇의 `pending_candidates`로 저장되는 것을 확인했다.

### 5.4 비상정지·재개

- 시험 run: `1791783172`
- 두 로봇 Survey 실행 중 `/inspection/estop` 호출
- 두 goal cancel 수락과 취소 결과 확인
- `/inspection/resume` 호출
- 동일 task_id가 로봇별로 정확히 한 번 다시 전송됨
- 재실행 후 두 로봇 Survey·Dock 완료
- 결과: 성공

### 5.5 OFFLINE

- 시험 run: `1791783227`
- 시험용 timeout: 3초
- 두 로봇 Survey 실행 중 mock의 RobotStatus 발행 중단
- 두 로봇의 실행 작업이 `FAILED(OFFLINE)`으로 종료됨
- E3 경보 발행 경로와 로봇별 대기 로그 확인
- `active_goals`가 영구적으로 dispatch를 막지 않는 것 확인
- 결과: 성공

### 5.6 IF-19

실제 ROS 토픽 주입 결과:

```text
/robot4/inspect/result NORMAL
  → INSPECTION_NORMAL, ref=RCK-01

/robot3/inspect/result ABNORMAL
  → CONFIRMED, ref=EXT-01
```

두 로봇 subscription 모두 정상 동작했다.

### 5.7 병합 뒤 빌드·시험 (10/13)

- 빌드: `scv_msgs`·`scv_bringup`·`scv_survey`·`scv_manager`·`scv_nav` 성공
- 단위 시험: `scv_manager/test` + `scv_survey/test` 87 passed
- mock 한 회차 (관제 PC 한 대, `cell_map_writer` + `inspection_manager` +
  `mock_execute_task_server.py`, 칸 맵은 파일, 격리 도메인):

```text
측정·Dock → 칸 맵 확정 Z1 30칸(음영 1 · 미검출 1 · low 7), Z2 30칸(음영 1 · 미검출 0 · low 7)
→ 재측정 3칸 (CLEARED 1 · REMAINS 2) → Dock → 회피 칸 2개
→ SHT-02 점검, SHT-01 은 KEEPOUT_BLOCKED, 좌표 없는 설비 FACILITY_SKIPPED → Dock → RUN_COMPLETED
```

재현 명령은 `ros2_ws/src/scv_manager/BUNDLE_TEST.md`에 있다. 실제 로봇·Nav2·DB 백엔드로는
병합 뒤 돌리지 않았다.

## 6. 반드시 합의할 인터페이스·정책

### 6.1 RSSI 후보 분석 완료 신호

현재 `network_map_engine`에는 후보 발행과 후보 분석 완료 신호가 없다.
관제는 다음을 구분할 수 있어야 한다.

- 후보가 아직 계산 중
- 후보 발행 완료, 후보 N건
- 후보 발행 완료, 후보 0건
- 분석 실패

완료 신호에는 최소 `run_id`와 성공 여부가 필요하다. 이 신호를 받은 콜백에서
`begin_reinspection_phase()`를 호출하면 된다. 새 메시지·서비스 소유자는
PM과 RSSI 팀이 정해야 한다.

10/13: 완료 신호 대신 관제가 `cell_map_writer`의 확정 서비스를 부르고 칸 맵을 직접 읽어
재측정 칸을 고르는 것으로 풀었다. IF-15 후보 토픽도 여전히 받으므로 두 경로를 함께 켜면
같은 칸이 두 번 들어올 수 있다 (PLAN D3 · 3-6, 미정).

### 6.2 최종 keepout READY/ACK

관제는 재측정이 끝난 뒤 남은 음영·미검출 칸이 두 로봇의 주행 costmap에
반영됐는지 확인한 후에만 시설 점검을 시작해야 한다.

확정할 내용:

- `/robot3`, `/robot4` 양쪽 keepout mask 제공
- mask가 어느 `run_id`의 결과인지 표현
- 두 로봇에 적용됐음을 알리는 READY 또는 ACK
- 갱신 실패 시 재시도·중단 정책

현재 IF-10 문서가 `/robot3`만 대상으로 되어 있다면 두 로봇 대상으로 수정해야
한다.

10/13: 관제가 `/robot4/keepout_mask`·`/robot3/keepout_mask`를 직접 낸다 (켜자마자 빈 마스크,
점검 전 남은 칸, 종료 뒤 다시 빈 마스크). READY/ACK는 없다. 마스크를 낸 다음 dispatch
주기(0.5초)에 점검 작업을 보내며 costmap 반영은 확인하지 않는다 (PLAN 3-2).

### 6.3 설비의 섹터 배정

`facilities.yaml`에는 `zone_id`가 없다. 따라서 관제가 설비를 robot4·robot3 중
누구에게 배정할지 확정할 수 없다.

권장안:

```yaml
- id: EXT-01
  zone_id: Z1
```

좌표 polygon 포함 여부로 자동 계산하는 방법도 가능하지만, 경계 설비 처리까지
고려하면 명시적인 `zone_id`가 더 안전하다.

10/13: 관측 자리(`view`)가 든 섹터의 polygon으로 정한다. `zone_id` 명시는 아직 결정되지
않았다 (PLAN 3-10).

### 6.4 OFFLINE·BLOCKED 섹터 정책

실행 중 작업을 FAILED로 닫는 기술적 교착은 해결했다. 그러나 해당 섹터의 남은
작업 정책은 결정되지 않았다.

- 해당 섹터 실패 처리 후 다른 로봇은 계속
- 다른 로봇이 해당 섹터를 인계
- 전체 순찰 중단

두 로봇이 동일 기능을 갖게 됐어도 배터리·시간·섹터 충돌 문제가 있으므로 PM과
주행팀 결정이 필요하다.

### 6.5 IF-19 판정 의미

현재 관제는 `ABNORMAL`, `NOT_DETECTED`, `OCCLUDED`,
`POSITION_ANOMALY`를 즉시 `CONFIRMED`로 발행한다. 비전 노드가 내부에서 최대
3회 관측을 끝낸 최종 결과만 `InspectionResult`로 보내는 구조라면 유지하면 된다.
그렇지 않다면 Candidate 기반 재검증 상태가 추가로 필요하다.

### 6.6 주행·공유 문서 확인

- 주행 `mission_executor`에 `RCK` 시설 ID를 `KIND_RACK`으로 변환하는 매핑이
  필요하다. (10/13: 주행 `86ceff8`에서 끝났고 이 브랜치에 병합됐다.)
- 재측정은 현재 후보 한 건당 action 한 건으로 관제에서 구현했다. 주행팀에 이
  계약을 공유해야 한다.
- 관제 B와 다음 Alert code를 공유하고 `interfaces.md`에 반영해야 한다.

```text
RUN_STARTED
RUN_COMPLETED
RESUMED
REINSPECTION_DONE
INSPECTION_NORMAL
TASK_FAILED
```

## 7. 현재 제한 사항

10/13 병합 뒤 기준이다.

- 단계가 회차 전체에 하나라 먼저 끝난 로봇이 다른 로봇을 기다린다.
- 실기 확인은 robot4 한 대로 재측정 대상 선정까지다 (10/12 밤, 임시 브랜치).
  재측정 이후 · 두 대 동시 · DB 백엔드는 실기로 확인된 적이 없다.
- `scv_manager`가 `scv_survey`의 `cellmap_store`·`cell_map_writer`·`precise_server`에
  기댄다. 이 세 파일은 `feature/survey`·`main`에는 아직 없다.
- 운영 `facilities.yaml`은 SHT-01·SHT-02만 좌표가 있다. EXT-01·EXL-01·RCK-01은
  (0, 0)이라 건너뛴다 (`FACILITY_SKIPPED`). robot3 섹터에는 점검할 설비가 없다.
- 비전의 `inspect_facility` 서버가 없어 `robot_pc.launch.py`는 가짜 비전
  (`mock_inspect_facility_server.py`, 2초 뒤 NORMAL)을 띄운다.
- 관제 웹(`scv_app`)과 `inspection_manager`는 이어져 있지 않다 (PLAN D2).
- 재측정은 로봇마다 5칸에서 잘리고, 표본 1개(low) 칸은 마스크에 넣지 않는다.
- 관제를 다시 켜면 진행 중이던 회차·작업 상태가 사라진다.
- `apply_remeasure`는 `n_samples`가 0이면 `reason` 문자열에서 값을 다시 읽는다 (PLAN 3-12).

## 8. 다음 담당자 작업 순서

10/13: 아래 1·2·4·7번은 끝났고, 8번은 구역 좌표와 셔터 두 건까지 들어갔다. 남은 일은
`docs/control/PLAN.md` 4절을 따른다.

1. 현재 diff를 검토하고 테스트를 한 번 재실행한다.
2. 변경 내용을 커밋하고 `feature/control`에 push한다.
3. PM·RSSI 팀과 후보 분석 완료 인터페이스를 확정한다.
4. 완료 콜백에서 `begin_reinspection_phase()`를 호출한다.
5. PM·주행·RSSI 팀과 최종 keepout READY/ACK를 확정한다.
6. `facilities.yaml`의 `zone_id` 또는 대체 배정 규칙을 확정한다.
7. 로봇별 시설 작업과 최종 Dock 생성 로직을 연결한다.
8. 운영 좌표를 입력한다.
9. 주행·RSSI·비전 실제 노드와 전체 시나리오를 통합 시험한다.
10. 시험 결과를 오늘 일지와 본 문서에 갱신한다.

커밋 전 최소 확인 명령:

```bash
cd /home/hymi/B_0102_Final_Project
source /opt/ros/jazzy/setup.bash
colcon build --packages-select scv_msgs scv_bringup scv_manager --symlink-install
source install/setup.bash
colcon test --packages-select scv_manager
colcon test-result --verbose
git diff --check
git status --short
```
