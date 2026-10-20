# 관제 A (inspection_manager) 인수 작업 계획 (v0.5)

> 2026-10-15 추가: `temp/integration-1014`에서 분기한 `feature/Integration_RSSI`의 재점검 구현은 [RSSI 재점검 구현 계획](RSSI_REINSPECTION_PLAN.md)을 따른다. 이번 요청의 기준은 **기준선 없이 칸 평균·미검출 비율로 후보 선정 → 같은 섹터 로봇의 10~15초 정지 측정 → 해제/남음·회피**다. 아래의 기존 중앙값·30초 기록과 구분하며, 세 결함의 수정·격리 검증을 완료했으며, 전체 통합의 남은 범위는 새 계획의 진행 기록과 [수정 인계서](../daily/HJAE99/2026-10-15_RSSI_재점검_3가지_수정_인계서.md)를 참고한다.

2026-10-13 저녁 · 담당 현준 (10/13 관제 A 에 추가 — 관제는 정섭 · 현민 · 현준, 주행도 그대로) · 기준: COOP Hub 일정 T6·T2·T5-8, `docs/interfaces.md` v0.2.11, `docs/CONTROL_MANAGER_STATUS.md`(10/12 인수인계), `docs/TEST_PLAN.md`, `docs/CRITICAL_PATH.md`

현민 님이 10/12 에 조퇴하며 남긴 인수인계를 이어받아, 관제 A 가 10/14 전체 통합까지 해야 할 일을 정리했습니다. 아래 「9. 근거」에 무엇을 직접 확인했고 무엇을 확인하지 못했는지 적었습니다. 결정이 나면 「3. 먼저 정할 것」 표의 상태를 고칩니다.

v0.2 (10/13 밤): COOP Hub 화면을 직접 읽어 반영 — Step 별 완료 기준(1절), PM 의 10/12 밤 실기 시험 결과(2.4), D1·D5·D6, 빠져 있던 시험 항목(단계 2·3·4·5).

v0.3 (10/13 밤): `temp/bundle-test-1012`(`faa34c4`)와 `feature/driving`(`630f07a`)을 `feature/control` 에 병합했습니다 (`200f4f3` · `e1b27ee`, 충돌 없음, 10/13 20:40 push). 단계 0-3 · 0-4 · 1-1 – 1-4 · 2-1 이 끝났습니다 (4절 「진행」). 아래 0절 1–3번과 2.1 표의 `feature/control` 열은 **병합 전** 상태를 적은 것이고, 지금 `feature/control` 은 표의 임시 브랜치 열과 같습니다.

v0.4 (10/13 20:20): 남은 일을 순서대로 4절 맨 위 「지금 남은 일」에 모았습니다. 끝난 항목은 표의 우선 칸에 「끝」으로 적었습니다. D8 확정.

v0.5 (10/14 새벽): 10/13 밤 robot3 실기 결과와 새 측정 경로 계획을 넣었습니다 (4절 「새 측정 경로」). 「지금 남은 일」을 그 순서로 다시 짰습니다.

v0.6 (10/14 오후): 통합에 빠진 `survey_buffer`(IF-08·09)와 DB 연결(2-10 · D4 · 4-7 · 4-4)을 작업 표로 넣었습니다 (4절 「survey_buffer 추가」 · 「DB 연결」, 「지금 남은 일」 12 · 13번). 로봇 PC 는 지금 시험용 `manual_tagger` 가 그 자리를 대신하고, `backend:=db` 는 실제 DB 로 돌린 적이 없습니다.

## 0. 한눈에

1. `feature/control` 의 `inspection_manager` 는 **두 로봇 동시 측정 → Dock → 대기**까지만 갑니다. 그 뒤 단계(재측정 · 회피 마스크 · 설비 점검 · 회차 종료)는 이 브랜치에 없습니다.
2. 그 뒤 단계는 **PM 이 10/12 저녁에 `temp/bundle-test-1012` 브랜치에 임시로 구현해 두었습니다** (허브 T2-6 「현민 코드 기준 수정·시험」). 빈 마스크 발행, 칸 맵 확정, 재측정, 마스크, 설비 점검, 종료, launch 까지 들어 있습니다. `feature/control` · `main` 어디에도 아직 합쳐지지 않았습니다.
3. 따라서 관제 A 의 첫 일은 새로 짜는 것이 아니라 **임시 구현을 넘겨받아 검토 · 반영 · 실기 시험**하는 것입니다 (단계 1–2). 넘겨주는 일은 T2-6 의 남은 할 일로 PM 에게 잡혀 있습니다.
4. 관제 B 의 웹(`ros2_ws/scv_app`)은 `inspection_manager` 와 **연결돼 있지 않습니다.** 웹 서버가 자체로 작업을 만들어 WebSocket 으로 제어 PC 에 보내는 별도 흐름입니다. T6-4 의 완료 기준이 "웹에서 시작부터 종료까지 보인다"(P0)라서, 어느 쪽이 작업의 원본인지 정해야 T6-4 와 T6-6 이 풀립니다 (D2).
5. 실기는 PM 이 10/12 밤 robot4 한 대로 **재측정 대상 선정까지** 갔습니다 (2.4). 재측정 · 마스크 · 설비 점검, DB 경로, 두 대 동시는 실기로 확인된 적이 없습니다.

## 1. 일정 (COOP Hub)

허브 스케줄 화면(10/13 저녁, 현준 계정)에서 읽었습니다. T6 업무의 허브 담당은 정섭 님이고, T6-3–6 은 모두 아직 제출이 없습니다.

| Step | 내용 | 기간 | 상태 | 완료 기준 (허브) |
| --- | --- | --- | --- | --- |
| T6-2 | (현민) 작업 생성·배정·완료 — inspection_manager 독립 개발 | 10/10–10/12 | OK (3회차) | — |
| T6-3 | 핵심 연결 (IF-11·12·16 먼저 → IF-17·15·19·08·09) | 10/12–10/13 | 진행 중 | ① 후보를 주입하면 그 섹터 로봇에 재측정 배정 ② 중복 NetSample 은 1건만 저장 ③ seq 누락 뒤 sync 로 누락 0 ④ 후보 · 해제가 5초 안에 웹에 ⑤ 시작부터 빈 마스크가 두 토픽에, 점검 전에 남은 칸 마스크. **IF-11 왕복 · IF-12 상태와 오프라인 · IF-16 시작과 비상정지는 T5-8 전에 먼저** |
| T6-4 | 로봇 한 대 통합 (Undock → 측정 → 설비 점검 → 저장·표시 → Dock) | 10/12–10/13 | 진행 중 | 한 대 순찰 전체가 **웹에서 시작부터 종료까지** 보임 (P0). 통합은 `main` 에서 |
| T6-5 | 로봇 두 대와 예외 (재측정 · sync · 비상정지 · BLOCKED) | 10/13 | 진행 중 | ① 응답 없음을 30초 안에 감지 · 화면 경보 ② 관제 PC 를 끊어도 로봇은 받은 작업을 끝내고 재연결 뒤 복구 ③ 비상정지 때 두 로봇 goal 취소 · 정지 · 화면 경보 ④ E8: 큐 · 재시도 · 경보 |
| T6-6 | 보고서·점검표 출력 | 10/13 | 진행 중 | 순찰 종료 뒤 지도 · 보고서 · 점검표가 저장되고 화면에 보임 |
| T5-8 | [묶음 시험] 주행 + 관제 — 작업 배정 왕복 · status · 비상정지 | 10/13 | 진행 중 | (세부를 열어 보지 않음) |
| T2-6 | RSSI + 주행 + 관제 묶음 통합 시험 (현민 코드 기준 수정·시험) | 10/12–10/13 | 진행 중 | ① 로봇 없이 한 회차 완주 (통과, 10/12) ② 로봇 한 대 이상으로 완주 · 기록 ③ 바뀐 곳을 정리해 관제 A(현민 · 현준)에게 넘김 |
| T2-5 | System Monitor 코드리뷰 | 10/14 10:30 | 예정 | — |
| T2-2 | 전체 통합 — 두 대 동시 풀사이클 · 시연 리허설 | 10/14 | 예정 | — |

일지 마감은 매일 21:00 입니다. `docs/daily/lhm3185/2026-10-13.md` 는 양식 그대로(미제출)입니다.

## 2. 지금 상태

### 2.1 관제 A 코드 — 브랜치별

`feature/control` 은 `ddc5f4d`, 임시 브랜치는 `faa34c4` 기준입니다. 임시 브랜치는 현민 님 커밋 `c54bed0` 에서 갈라졌고, 그 뒤 `feature/control` 에 들어온 정섭 님 커밋(`cbeb6cc`)은 포함하지 않습니다.

| 기능 | `feature/control` | `temp/bundle-test-1012` |
| --- | --- | --- |
| 두 로봇 동시 SURVEY → DOCK, 로봇별 dispatch | 있음 (mock 왕복 통과) | 같음 |
| RobotStatus 감시 · OFFLINE 시 실행 작업 FAILED · E3 | 있음 | 같음 |
| 비상정지 · 재개 · 재배정 서비스 (IF-16) | 있음 | 같음 |
| IF-19 판정 수신 → 알림 | 있음 | 같음 |
| IF-10 빈 마스크를 켜자마자 두 로봇 토픽에 | **없음** | 있음 (`MASK_QOS` reliable · transient_local · depth 1) |
| 측정 경로를 벽에서 0.45 m 안쪽으로 · 줄을 칸 중심에 | **없음** — 운영 `zones.yaml` 로 만들면 경로 점이 x 0.08 · 3.18(벽 선)과 y 0.20 에 놓임 | 있음 (`wall_bounds` 사각형 기준, 섹터당 12점 · x 0.53 ↔ 2.73) |
| 측정 뒤 칸 맵 확정 요청 | **없음** (`WAITING_CANDIDATES` 에서 멈춤) | 있음 — `cell_map_writer` 의 `commit_Z1`·`commit_Z2` 서비스 호출, 10초 안에 답이 없으면 저장된 값으로 진행 |
| 재측정 대상 선정 → REINSPECTION 작업 | 작업 생성 함수만 있고 부르는 곳이 없음 | 있음 — 칸 맵에서 미검출 → 음영(낮은 값부터) → 미측정 순, 로봇마다 최대 5칸 |
| 재측정 결과로 칸 상태 고치기 | **없음** | 있음 — `n_samples` 0 이면 미검출, 중앙값 < −72 dBm 이면 음영, 아니면 정상. 알림 `CLEARED`·`REMAINS` |
| 점검 전 회피 마스크 (남은 칸, Dock 반경 0.6 m 제외) | **없음** | 있음 — 표본 2개 이상(`confidence` ok) 칸만 |
| 설비 점검 작업 생성 → DOCK | **없음** | 있음 — 관측 자리가 든 섹터의 로봇에 배정, 좌표 (0, 0) 설비는 건너뜀, 관측 자리가 회피 칸이면 `KEEPOUT_BLOCKED` |
| 회차 열기 · 닫기 (run_id) | 시작 시각을 run_id 로 | 칸 맵 저장소가 run_id 를 줌 (파일 또는 DB `runs`) |
| launch | 뼈대만 (노드 없음) | `control_pc.launch.py` · `robot_pc.launch.py` |
| 단위 시험 | 22개 (이 중 `manager_core`·`route_builder` 14개는 10/13 재실행해 통과) | 문서상 `scv_manager` + `scv_survey` 87개 통과 (재실행하지 않음) |

임시 브랜치의 구현은 `scv_survey` 의 `cellmap_store` · `cell_map_writer` · `precise_server` · `heatmap` 에 기대고 있습니다. 이 가운데 `heatmap.py` 만 `feature/survey` 에 있고 나머지 셋은 임시 브랜치에만 있습니다.

### 2.2 다른 팀 쪽 전제

| 팀 | 확인한 것 (원격 브랜치 기준) | 관제 A 에 주는 영향 |
| --- | --- | --- |
| 주행 (`feature/driving`) | `nav2.yaml`·`nav2_rpp.yaml` 의 `keepout_layer` 가 이미 `enabled: true`, 토픽은 `{ns}/keepout_mask` | **관제가 마스크를 내지 않으면 costmap 이 준비되지 않아 경로 계획이 실패합니다.** 관제 PC 를 로봇 PC 의 Nav2 보다 먼저 켜야 합니다 |
| 주행 | `mission_executor`: Dock 상태에서 DOCK 이 아닌 작업을 받으면 스스로 undock. DOCK 은 waypoints 가 비면 undock 위치로 가서 dock | 관제는 undock 을 따로 보내지 않아도 됩니다. 지금 DOCK 작업의 빈 waypoints 와 맞습니다 |
| 주행 | 건너뛴 점이 하나라도 있으면 `ok=false`, `reason="BLOCKED skipped=N"` | 관제는 이미 `BLOCKED` 로 시작하는 사유를 BLOCKED · E1 로 처리합니다 |
| 주행 | 바쁘거나 배터리 30 % 이하(DOCK 제외)면 goal 거절. 20 % 이하면 작업을 끊고 스스로 dock (`LOW_BATTERY`) | 관제는 거절을 `GOAL_REJECTED` 로 바로 FAILED 처리하고 다시 보내지 않습니다. 배터리 정책이 필요합니다 (D5) |
| 주행 | REINSPECTION 은 마지막 waypoint 에서 30초 정밀 측정 한 번, 결과를 `rssi_median`·`rssi_iqr`·`n_samples` 에 넣음 | 후보 한 건당 작업 한 건이라는 지금 구조와 맞습니다 |
| 주행 | `RCK` → `KIND_RACK` 매핑이 이미 있음 | 인수인계 6.6 의 요청은 끝났습니다 |
| RSSI (`feature/survey`) | `network_map_engine` 은 후보(IF-15)를 내지 않고 DB 에도 쓰지 않음. 마스크를 자기가 `/keepout_mask` 로 냄 (예전 설계) | 후보 토픽으로는 아무것도 오지 않습니다. 임시 브랜치는 칸 맵을 직접 읽어 후보를 만듭니다 (D3) |
| 비전 (`feature/vision`) | `scv_detection` 의 `.py` 에서 `inspect_facility` 액션 서버와 IF-19 발행 코드를 찾지 못함 (다른 위치일 수 있음) | 실제 설비 점검 통합은 비전 쪽 준비를 먼저 확인해야 합니다. 임시 브랜치는 가짜 비전 서버(2초 뒤 NORMAL)를 씁니다 |
| 관제 B (`ros2_ws/scv_app`) | `server_page.py` 에 ROS · `/inspection/*` · `scv_msgs` 가 없음. 자체 `POST /api/runs` · WS `task_assign`. 제어 PC 의 `client_page.py` 가 Nav2 를 직접 부름. `scv_monitor`(`ros_bridge`)는 빈 패키지 | 웹의 시작·비상정지 버튼과 관제 A 가 이어져 있지 않습니다 (D2) |
| 설정 (`scv_bringup/config`) | `zones.yaml` 은 Z1·Z2 실좌표와 `robot:`·`dock:` 이 들어 있음. `facilities.yaml` 은 SHT-01·02 만 좌표가 있고 EXT-01·EXL-01·RCK-01 은 0.0 | 지금은 robot4 의 셔터 두 건만 점검 대상입니다. robot3 섹터에는 설비가 없습니다 |

### 2.3 인수인계 문서와 달라진 점

- 「아직 커밋·push 하지 않은 로컬 변경」 → `c54bed0` 으로 push 됐습니다. 「다음 담당자 작업 순서」 1·2 번은 끝났습니다.
- 「운영 `zones.yaml` 좌표가 모두 0」 → 실좌표가 들어 있습니다. 다만 `feature/control` 의 경로 생성은 벽 여유를 보지 않습니다 (2.1).
- 「RSSI 후보 분석 완료 신호 대기」(6.1) · 「최종 keepout READY/ACK」(6.2) → `interfaces.md` v0.2.9 가 "노드 사이에 요청을 주고받지 않고 DB 를 본다"로 정했고, 임시 브랜치는 칸 맵 확정 서비스와 저장소 읽기로 풀었습니다.
- 「설비의 섹터 배정」(6.3) → 임시 브랜치는 관측 자리가 든 섹터의 polygon 으로 정합니다. `zone_id` 를 명시하자는 권장안은 아직 결정되지 않았습니다.
- 6.4 OFFLINE·BLOCKED 정책, 6.5 IF-19 판정 의미, 6.6 의 Alert code 공유는 그대로 남아 있습니다.

### 2.4 PM 의 10/12 밤 실기 시험 (허브 T2-6)

robot4 한 대, 관제 PC = VICTUS, 로봇 PC = MSI, 칸 맵은 파일로 돌렸습니다.

| 구분 | 내용 |
| --- | --- |
| 이어진 것 | 관제 → 측정 작업 → 주행 → RSSI 샘플 → 칸 확정 → 재측정 대상 5칸 선정 |
| 안 된 것 | 핫스팟이 꺼져 있어 17칸 중 16칸이 미검출. Dock 작업이 로봇 PC 쪽에서 취소돼 재측정은 보내지 못함 |
| PM 의 남은 할 일 | ① 핫스팟을 켜고 한 대로 다시 완주, 결과(칸 수 · 재측정 전후 · 마스크) 기록 ② 측정이 진행률 50 % 에서 완료로 끝난 원인 확인 ③ 칸 맵을 DB 로 쓰는 경로를 실기로 확인 ④ 두 대 동시 ⑤ 바뀐 곳을 정리해 관제 A 에 넘김 |

## 3. 먼저 정할 것

| # | 정할 것 | 선택지 | 권장 | 누구와 | 상태 |
| --- | --- | --- | --- | --- | --- |
| D1 | 임시 브랜치의 관제 A 구현을 언제 · 어떻게 넘겨받나 | 허브 T2-6: 수정과 시험은 PM 이 임시 브랜치에서 하고 `main` · 팀 브랜치는 그대로 둠. 넘기는 방법은 (가) PM 이 `main` 에 합치고 팀 브랜치에 내려 줌 · (나) 관제 A 가 `scv_manager` 와 launch 만 골라 옴 · (다) 10/14 통합까지 임시 브랜치에서 계속 | (가). `scv_manager` 가 `scv_survey` 의 세 파일에 기대므로 (나)는 혼자 빌드되지 않습니다. 미리 본 병합(`feature/control` ← 임시 브랜치)은 충돌이 없었습니다. 인계 전에는 임시 브랜치를 읽고 시험만 합니다 (단계 0) | PM · RSSI | **끝 (10/13 밤)** — 관제 A 가 임시 브랜치 전체를 `feature/control` 에 병합 (`200f4f3`). `main` 반영과 `scv_survey` 세 파일의 주인은 남음 (D3) |
| D2 | 순찰 시작과 작업의 원본이 어디인가 | (가) `inspection_manager` 가 원본, 웹은 `/inspection/start`·`estop`·`resume` 을 부르고 알림·상태를 받아 보여 줌 · (나) 웹 서버가 원본, `inspection_manager` 는 쓰지 않음 · (다) 시연은 (가)로, 웹은 DB 를 읽어 표시만 | (가). `interfaces.md` 가 "로봇 PC ↔ 관제 PC 는 ROS 그대로"이고 주행 · RSSI 가 그 인터페이스로 맞춰져 있습니다. T6-4 가 "웹에서 시작"을 요구하므로 (다)는 T6-4 를 채우지 못합니다 | 정섭 · PM | 미정 |
| D3 | 칸 맵 저장과 후보의 주인 | (가) 임시 구조 유지: `cell_map_writer` 가 저장, 관제가 읽어 재측정 칸을 고름 · (나) v0.2.9 대로 `network_map_engine` 이 DB 에 쓰고 IF-15 후보를 냄 | 10/14 통합까지는 (가). 두 경로를 같이 켜면 같은 칸이 후보로 두 번 들어올 수 있습니다 | RSSI · PM | 미정 |
| D4 | DB 쓰기 경로 | 임시 `DbStore` 는 `runs`·`grid_stats`·`aps` 에 psycopg 로 직접 씀 ↔ v0.2.9 는 "DB 쓰기는 모두 관제 API 로". 웹 서버도 자기 `runs` 를 만듦 | D2 와 함께 정합니다. 그때까지 시연 경로는 `backend:=file` | 정섭 · PM | 미정 |
| D5 | 단계를 로봇마다 따로 넘기기 | 지금(두 브랜치 모두)은 회차 전체에 단계 하나 — 먼저 끝난 로봇이 다른 로봇을 기다림. 허브 T6-3·T6-5 는 "두 로봇은 서로 기다리지 않는다(한 대만 재측정이면 다른 한 대는 바로 점검)"로 정했고, T6-5 가 이것을 「남은 차이」로 적어 둠 | 고쳐야 하는 항목입니다. 한 대 완주(2-6–2-8)가 확인되면 바로 합니다 (단계 3-1) | — | 해야 함 (T6-5) |
| D6 | OFFLINE · BLOCKED · 배터리 때 그 섹터의 남은 작업 | 그 섹터만 실패로 두고 회차는 「부분 완료」로 끝냄 · 다른 로봇이 인계 · 전체 중단 | 그 섹터만 실패 (허브 · `TEST_PLAN.md` 가 재배정 · 인계를 풀사이클 뒤 추가 기능으로 둠) | PM · 주행 | PM 이 정해서 알림 (허브 T6-5) |
| D7 | IF-19 의 `InspectionResult` 가 3회 관측을 끝낸 최종 판정인가 | 최종이면 지금처럼 바로 `CONFIRMED` · 아니면 재검증 상태가 필요 | 최종으로 가정 (`mission_executor` 가 자세 보정까지 끝낸 뒤 결과를 받음) | 비전 | 미정 |
| D8 | 현민 님 일지를 어디에 쓰나 | `docs/daily/lhm3185/` 에 대신 씀 · 현준 일지에 `with: 현민` | PM 에게 확인 | PM | **끝** — PM 이 현준 일지 양식을 두 업무용으로 바꿈 (`630f07a`). 관제 일은 `docs/daily/barlide/` 의 「관제 (T6)」 칸에 씀 |

## 4. 작업 목록

우선순위: **P0** 오늘 밤 · **P1** 10/14 10:30 코드리뷰 전 · **P2** 10/14 전체 통합 중 · **P3** 풀사이클이 한 번 돈 뒤.

### 지금 남은 일 (10/14 새벽 기준, 위에서부터)

10/13 밤 robot3 한 대 실기까지 `origin/feature/control` 에 올라가 있습니다 (`d8f7c26` · `f07e6a6`). 확인된 것: 빈 회피 마스크를 Nav2 가 받음(2-2), 운영 설정 `nav2_rpp.yaml` 로 뜸(3-13). **10/14 11:20 robot3 한 대로 새 측정 경로 첫 완주 (N-6)** — 측정 12점 → Dock → 칸 확정 → 미측정 칸 1개 정밀 측정 → Dock → 종료 (2-3 · 2-11 · 2-12). 음영 후보 재측정(2-6)은 아직. 설비 점검은 Z2 에 설비가 없어 건너뜀, 회피 칸은 0개라 마스크 주행(2-7)은 아직.

| 순서 | 할 일 | 표의 번호 | 기한 | 막는 것 |
| --- | --- | --- | --- | --- |
| 1 | 새 측정 경로 — 경로 생성 · 작업 배정 · 시험 · 그림 확인 | N-1 – N-4 | 10/14 오전 | — |
| 2 | 새 경로로 mock 한 회차 | N-5 | 10/14 오전 | 1번 |
| 3 | robot3 한 대로 측정 → Dock 완주. 새 inflation(0.30 · 10)의 벽 · Dock 접근 거리도 같이 | N-6 · 2-3 · 2-11 · 2-12 | 10/14 10:30 전 | 2번 · 로봇 |
| 4 | 상태 끊김 → 비상정지 · 재개 | 2-4 · 2-5 | 10/14 10:30 전 | 3번 |
| 5 | 알림 코드 표를 정섭 님에게 전달, PM 에 D2 · D6 확인 | 4-3 · 0-1 | 10/14 10:30 전 | — |
| 6 | 코드리뷰 준비 (단계 흐름 그림, 경로 그림, 시험 결과) | 6-1 | 10/14 10:30 | 3 · 4번 결과 |
| 7 | 핫스팟을 켠 채 한 대 완주: 칸 확정 · 재측정 → 마스크 → 설비 | 2-6 · 2-7 · 2-8 | 10/14 통합 중 | 3번 |
| 8 | 빈 칸 메우기 — 재측정과 다른 단계로. 측정 직후 Dock 전에 미측정 칸을 지나가며 잼 (칸 안의 다른 점, 5칸 제한 없이). 10/14: 재측정 대상에서 미측정 칸을 뺌(`include_unmeasured` 기본 false) | N-8 · 3-4 | 한 대 완주 뒤 | 7번 |
| 9 | 두 대 동시, 이어서 DB 백엔드 | 2-9 · 2-10 (DB-1 – DB-3) | 10/14 통합 중 | 7번 · D4 |
| 10 | 단계를 로봇마다 따로 (D5), 마스크 반영 대기 | 3-1 · 3-2 | 한 대 완주 뒤 | 7번 |
| 11 | 예외 시험 (통로에 상자 5-3 포함), 웹과 잇기 | 단계 5 · 단계 4 | 10/14 통합 중 | D2 · D4 · D6 |
| 12 | survey_buffer 추가 — run_id · 버퍼 · `survey/sync` · CSV, 끊김 복구 시험 | S-1 – S-4 | 10/14 통합 중 | 7번 · 로봇 · RSSI A 합의 |
| 13 | DB 연결 — 시험 DB · 지도 0.5 m 격자 · db 한 회차 · 원본 샘플 · 끊김 복구 | DB-1 – DB-6 | 10/14 통합 중 | 12번 · D4 · 정섭 (지도 등록) |

### 새 측정 경로 (칸 기준 · 줄 묶기 · 구간 나누기)

왜: 10/13 밤 실기에서 Z2 5번 점 (0.53, 4.5) 앞 0.41 m 에서 로봇이 못 나아갔습니다. 벽 조각 아래 막다른 자리라 경로가 갔다가 같은 길로 되돌아오는데, RPP(lookahead 0.4)는 끝에 닿기 전에 돌아나오는 쪽을 따라가서 점이 지워지지(0.35 m) 않습니다. 지금 알고리즘은 섹터 사각형에 가로줄을 긋고 줄 끝만 벽에서 띄우므로, 지도가 바뀌면 줄 가운데 벽 · 닫힌 공간 · 막다른 자리를 처리하지 못합니다.

방식: ① RSSI 칸마다 방문 점(칸 중심, 벽에 가까우면 칸 안쪽 10 cm 이상 · 벽에서 `obstacle_clearance_m` 이상인 점) ② 두 점 사이 직선이 벽에 걸리지 않는 이웃 칸만 연결 ③ Dock 에서 이어진 칸만 남김 ④ 가로로 이어지고 높이가 같은 칸을 줄로 묶어 양 끝 두 점만 경로에 ⑤ 가까운 줄 끝으로 지그재그 ⑥ 줄과 줄 사이 직선이 벽에 걸리면 거기서 작업을 나눔 (막다른 끝이 작업의 마지막 점이 되어 Nav2 가 끝까지 도착).

장애물(지도에 없는 것)은 지금처럼 Nav2 가 피하고 `mission_executor` 가 못 가는 점을 건너뜁니다(`BLOCKED skipped=N` → E1). 빠진 칸은 N-8 의 메우기가 다시 방문합니다.

| # | 할 일 | 바뀌는 곳 | 완료 기준 | 상태 |
| --- | --- | --- | --- | --- |
| N-1 | 경로 생성을 칸 기준으로 | `route_builder.py` (`build_survey_routes` 가 섹터마다 구간 목록을 돌려줌) | 실제 지도에서 모든 점이 벽에서 0.30 m 이상, 줄이 수평, Z2 벽 조각에서 구간이 나뉨 | 끝 (10/14) — 실제 지도: Z1 12점 · 3구간, Z2 12점 · 3구간, 벽 최소 0.33 m |
| N-2 | 구간마다 측정 작업 | `inspection_manager.py` (`…-survey-1`, `-survey-2`, … 뒤에 `-survey-dock`) | 구간 1이 막힘으로 끝나도 구간 2 · Dock 이 나감 | 끝 (10/14) — 구간이 하나면 이름은 예전처럼 `-survey` |
| N-3 | 시험 (기존 파일 수정) | `test_bundle_logic.py` · `test_inspection_manager.py` | 열린 방 · 줄을 가로지르는 벽 · 닫힌 방 · 막다른 자리 · 구간별 작업 | 끝 (10/14) — 93 passed |
| N-4 | 계산으로 확인 | — | 실제 지도와 가상 지도 그림, 지금 경로와 비교표(칸 · 점 · 구간 · 길이). 로봇을 움직이지 않고 플래너로 점 사이 경로 계산 | 그림 · 비교표 끝 (10/14). 플래너 계산은 Nav2 가 꺼져 있어 못 함 → N-6 에서 |
| N-5 | mock 한 회차 | 필요하면 `mock_execute_task_server.py` | `RUN_COMPLETED` 까지 | 끝 (10/14) — 두 로봇 mock: 측정 3구간씩 → Dock → 칸 확정 Z1 28칸 · Z2 27칸 → 재측정 3칸(CLEARED 1 · REMAINS 2) → 회피 칸 3개 → 마스크 → 설비 → `RUN_COMPLETED`. mock 서버는 고치지 않음 |
| N-6 | robot3 실기 | — | 구간 1이 막다른 끝에 도착, 구간 2가 이어서 나감, 진행률이 끝까지, Dock 복귀. 회차 시간 기록 | **끝 (10/14 11:20, run_id=3, robot3 한 대 · 핫스팟 turtle09)** — 첫 완주. 측정 구간 1(6점) 59초 → 구간 2(막다른 한 점) 33초 → 구간 3(6점) 41초, 건너뛴 점 0. Dock 40초 → 칸 확정 Z2 28칸(음영 0 · 미검출 0 · low 0) → 미측정 칸 1개 (4,10) 를 30초 정밀 측정(median −49 dBm · n 30) → Dock → 회피 칸 0 → 종료. **음영 후보의 재측정은 이 회차에 없었음** — 문서의 재측정(음영 · 미검출 후보의 30초 정밀 측정, `TEST_PLAN.md`)은 아직 실기 미확인 (2-6, 핫스팟을 끄고). 시작부터 종료까지 4분 29초. `mission_executor` 기동 멈춤 없음. Nav2 경고 없음 |
| N-7 | 문서 | `BUNDLE_TEST.md` · `CONTROL_MANAGER_STATUS.md` · 이 문서 | 측정 설명이 코드와 같음 | |
| N-9 | 측정 줄 간격 0.25 m · 한 줄 건너 돌기 (칸 한 줄을 1/4 · 3/4 높이로 두 번) | `route_builder.py` · `inspection_manager.py`(`survey_spacing_m`) · `test_bundle_logic.py` | 칸마다 두 줄, 줄 사이 0.5 m, 다음 점이 0.35 m 안이면 그 점을 혼자 한 작업으로 | 구현 · 시험(94 passed) · mock 한 회차 끝 (10/14) — 실제 지도 Z1 29.0 m · 27점 · 6작업, Z2 27.0 m · 28점 · 8작업. mock 두 로봇 `RUN_COMPLETED`, 칸 확정 Z1 29칸 · Z2 28칸. **실기 끝 (10/14 12:02, run_id=4, robot3)** — 측정 작업 8개 모두 `ok=True`, 건너뛴 점 0, 측정 4분 26초(0.5 m 때 2분 13초) · 회차 5분 19초. 칸 확정 Z2 29칸 전부 정상 → 재측정 0칸(미측정 칸 제외 확인) → 회피 칸 0 → 종료. 칸당 표본 수 중앙값 4 → 8, 0개인 칸 1 → 0, 2개 이하 2 → 1 (run 3 대비) |
| N-10 | 모서리 처리 비교 시험: 지움 반경을 줄여 모서리 파먹기를 줄이되 못 지울 모서리를 구간 끝으로(2번 `corner_mode:=angle`) vs 모서리 뒤 0.2 m 에 보조 점(3번 `corner_mode:=guide`) | `route_builder.py` · `inspection_manager.py`(`corner_mode` · `passed_goal_radius_m` · `rpp_lookahead_m` · `corner_guide_m`) · 시험 3개 | robot3 · 0.25 m 간격으로 A(none, 지움 0.35) · B(angle, 지움 0.15 — BT 를 시험용으로 바꿈) · C(guide, 지움 0.35) 세 회차. 회차마다 `/robot3/amcl_pose` bag → 점에 가장 가까이 간 거리 · 모서리 이탈 · 줄 이탈, 측정 시간, 작업 수, 칸별 표본 수 | 구현 · 시험(97 passed) · mock 끝 (10/14) — Z2 경로: A 28점 · 8작업, B 45점 · 24작업, C 44점 · 8작업 (길이 모두 27.0 m). mock 두 로봇 B · C 모두 `RUN_COMPLETED`. **A 끝 (12:24, run_id=5)**: 측정 작업 8개 모두 성공, 건너뛴 점 0, 시작부터 종료 5분 28초. 점에 가장 가까이 간 거리 최대 0.32 m (0.15 m 넘는 점 5 / 28), 모서리 0.5 m 안 계획 경로 이탈 평균 0.14 · 최대 0.32 m, 줄 가운데 계획 줄과의 거리 중앙값 0.135 m (처음 적은 0.074 는 0.125 m 넘게 벗어난 점을 버린 계산이라 작게 나옴 — 구간 순서대로 짝지어 다시 잼). 칸 29 · 표본 중앙값 7 · 최소 4. bag `~/scv_logs/bags/2026-10-14_N10_A_none`. **B 끝 (12:34, run_id=6, BT 지움 반경 0.15)**: 측정 작업 24개 모두 성공, 건너뛴 점 0, 측정 6분 13초(A 4분 47초). 마지막 Dock 은 `DOCK_FAILED`(undock 자리까지는 감, Create3 도킹 단계에서 실패 — 원인 미확인). bag 은 기록기를 Ctrl-C 없이 닫아 0 바이트 → 궤적은 RSSI 샘플 CSV(1 Hz) 로 A 와 같은 방식 비교: 점에 가장 가까이 간 거리 최대 0.10 m(A 0.32), 모서리 이탈 평균 0.10 m(A 0.12), 줄 가운데 계획 줄과의 거리 중앙값 0.048 m(A 0.135). 칸 28 · 표본 중앙값 13 · 최소 5. **C 끝 (12:48, run_id=7, guide 0.2 · 지움 0.35)**: 측정 작업 8개 모두 성공, 건너뛴 점 0, 측정 4분 39초, Dock 성공. 기준 28점 기준 가장 가까이 간 거리 최대 0.25 m(0.15 m 넘는 점 5 / 28), 모서리 이탈 평균 0.12 m, 줄 가운데 계획 줄과의 거리 중앙값 0.092 m. 칸 29 · 표본 중앙값 7 · 최소 5. bag `~/scv_logs/bags/2026-10-14_N10_C_guide020`. **비교 (CSV 1 Hz, 기준 28점)**: A 0.32 m · 6점 · 0.12 · 0.135 · 4:47 / B 0.10 m · 0점 · 0.10 · 0.048 · 6:13 / C 0.25 m · 5점 · 0.12 · 0.092 · 4:39 (가장 가까이 간 거리 최대 · 0.15 m 넘는 점 · 모서리 이탈 평균 · 줄 가운데 계획 줄과의 거리 중앙값 · 측정 시간). B 의 +86.5 초 = 작업 사이 대기 +13 초(작업 8 → 24개) + 모서리 정지 · 정렬 +73 초. **결론 (10/14 확정): B 채택** — 관제 기본값 `corner_mode` angle. 3번(guide)은 효과가 없어 코드에서 지움. 남은 것: B 회차의 Dock 실패(Create3 도킹 단계, 1회) 재현 확인, 작업 사이 대기(배정 타이머 0.5 초) 줄이기. **바로잡음 (10/14 오후)**: B 경로는 90도 모서리마다 구간이 끝나 작업 24개가 모두 직선 하나(시작점 + 끝점)였다 → 지울 중간 점이 없어 BT 지움 반경 0.15 는 효과가 없었고, 개선은 줄마다 골 하나로 멈췄다 정렬한 덕. 반면 0.15 는 lookahead 0.4 보다 짧아 구간 가운데 점을 한 번 놓치면 되돌아가도 못 지운다 — `course_run`(90도 모서리 4개를 한 번에) 반경 0.15 로 1회차 성공 46 초, 2회차 (2.2, 4.2) 를 0.152 m 로 놓친 뒤 y 4.5↔5.1 을 90 초 왕복(취소). 그래서 BT 는 주행 원본(반경 0.35, 3 초 주기 안)으로 되돌리고, 관제는 지움 반경 대신 꺾는 각으로 끊는다(`corner_break_deg` 90, `passed_goal_radius_m` 0.35 — `rpp_lookahead_m` 은 지움). Z2 경로는 B 와 같은 점·같은 줄(0.25 m 짧은 줄 바꿈 한 곳이 점 하나짜리 작업이 되어 44점 · 24작업). 시험 37 passed. bag `~/scv_logs/bags/2026-10-14_course_tick_1`(1·2회차) · `_tick_2`(2회차) |
| N-11 | 지도에 없는 장애물 처리: 측정 줄을 0.5 m 중간 점으로 나누고, 막힌 중간 점은 건너뛰고 · 막힌 골은 줄 위 빈 점으로 줄이고 · 못 간 칸은 채우기에서 뺌 | `route_builder.py`(`_split_segment`) · `inspection_manager.py`(`survey_split_m` 0.5 · 막힌 칸 기록) · `manager_core.py`(`blocked_cells` · `remeasure_targets(blocked)`) · `scv_nav/scripts/mission_executor.py`(주행 쪽 코드, 막힘 처리 E1) · 시험 | 시뮬(도메인 87: 실제 Nav2 `nav2_rpp` + mission_executor + 가짜 로봇 · 라이다)에서 상자로 중간 점 1개 · 2개 · 골(작은 상자 · 큰 상자)을 막아 갇히지 않고 다음 작업으로 넘어감 | **끝 (10/14 오후, 시뮬)** — 실기 전. 처음(끝점 하나만 막아 본 시험): 로봇이 장애물 안쪽 골로 다가가다 2–5 cm 까지 붙어 회전도 못 하고 갇힘(BackUp 복구 없음) → 다음 작업도 출발 못 함. 바꾼 것 — mission_executor: (1) 향하던 점이 global costmap 장애물 · inscribed 칸이 되면 다가가기 전에 처리 (2) 중간 점은 빼고 다음 점으로 (3) 골은 앞 점에서의 줄이 막혔고 골 둘레 0.5 m 안에 지도에 없는 장애물이 보이면 줄이 처음 막히는 곳에서 0.25 m 앞으로 줄이고, 그런 점이 없으면 뺌 (설비 · 재측정은 줄이지 않음) (4) 다음 작업의 시작점이 막혔던 자리면 뺌 (5) 앞 점에서 길이 없으면 지금 위치에서 다시 검사 (6) 취소가 끝날 때까지 기다린 뒤 다음 goal (늦은 취소가 새 goal 을 0초에 실패시키던 것) (7) 결과 `BLOCKED skipped=[…] shortened=[…]`. 관제: 그 번호로 막힌 칸(건너뛴 점의 칸, 줄인 골은 앞 점 → 골 구간 칸)을 회차 동안 기록, 미측정이면 재측정 후보에서 뺌. Z2 경로 44 → 72점(작업 24개 그대로). 시험 39 passed. 시뮬 결과(작업 3개): 중간 점 1개 42초 · 2개 32초(골도 막힘으로 보고 버림 — 앞에서는 줄 가운데만 막은 상자와 골을 덮은 상자가 구분되지 않아 막힘 쪽으로 봄. 세 쪽이 막혀야 막힘으로 보게 하면 큰 상자에서 돌아가다 모서리에 4 cm 까지 붙어 이후 작업을 모두 잃음) · 골 작은 상자 41초 · 큰 상자 32초, 모두 갇히지 않음. 남은 것: 우회 경로가 장애물 모서리를 스칠 때 RPP 가 질러가며 붙는 문제(주행 RPP 설정, 주행 쪽과 상의), 관제까지 넣은 mock · 실기 회차 |
| N-8 | 빈 칸 메우기 (이번 범위 밖, 완주 뒤) | `inspection_manager.py` · `route_builder.py` | 장애물이나 벽 때문에 빠진 칸을 다시 방문 | |

이번에 바꾸지 않는 것: 주행 코드(`scv_nav`, 단 N-11 의 `mission_executor.py` 막힘 처리와 10/14 초기 pose 반복 고침은 예외), 메시지(`scv_msgs`), 설비 점검, 재측정 칸 선택. 지도 없이 쓰는 예전 줄 방식은 남겨 둡니다 (실기 확인 뒤 정리).

위험: 순서 규칙(가까운 줄부터)이 지도에 따라 어색한 경로를 낼 수 있음 → N-4 그림으로 먼저 확인. 구간 사이에 한 번 멈췄다 출발해 시간이 늘어남 → N-6 에서 기록. 점 위치가 달라져 주행 시험 조건과 달라짐 → 열린 방에서는 지금과 같은 지그재그가 나오게 하고 시험으로 고정.

### survey_buffer 추가 (IF-08·09) — 통합에 빠진 것

`survey_buffer` 노드는 어느 브랜치에도 없습니다. 로봇 PC 는 `robot_pc.launch.py` 가 `laptop_test.launch.py` 를 tf 모드로 불러 **시험용 `manual_tagger`** 를 그 자리에 쓰고 있습니다. `manual_tagger` 에 없는 것: run_id 가 파라미터 고정값(기본 1), 버퍼 · 디스크 저장 없음, `survey/sync` 서비스 없음, 재시작하면 seq 가 0 부터라 수신 쪽이 중복으로 버림 (`scv_survey/README.md` 「알려진 한계」). 받는 쪽(`cell_map_writer` · `network_map_engine`)은 `(robot_id, seq)` 로 중복을 걸러서 재발행 샘플을 그대로 받을 수 있습니다. `sync` 를 부를 `ros_bridge`(관제 B)는 아직 없습니다 (4-7). 웹 `scv_app` 의 `SurveyBuffer` 클래스는 웹 흐름(D2)이라 별개입니다. 주인은 `interfaces.md` 상 RSSI A (성현 · 희재) — 통합 bringup 이 `feature/control` 에 있어 관제 A 가 하되 5절에서 확인.

| # | 할 일 | 파일 | 완료 기준 | 우선 |
| --- | --- | --- | --- | --- |
| S-1 | `survey_buffer` 노드 — `ManualTagger` tf 모드를 상속해 ① run_id 를 `{ns}/status`(`RobotStatus.run_id`) 에서 읽음 ② 보낸 샘플을 메모리에 쌓고 CSV 에 append (`~/scv_survey_logs/{ns}_buffer.csv`) ③ 시작할 때 CSV 마지막 seq 다음부터 ④ `survey/sync`: run_id 같고 from_seq 이상인 샘플을 `resent=True` 로 20 Hz 이하 재발행(depth 50 안), `ok` · `last_seq` · `count` 응답 | `scv_survey/survey_buffer.py`(새) · `setup.py` | 로봇 PC 에서 샘플 `run_id` 가 작업 회차와 같음, CSV 줄이 늚, 수동 sync 호출로 재발행됨 | P2 |
| S-2 | launch 교체 — tf 모드일 때 `manual_tagger` 대신 `survey_buffer`. `manual_tagger` 는 노트북 manual 모드용으로 남김 | `laptop_test.launch.py` | `robot_pc.launch.py` 를 바꾸지 않고 뜸 | P2 |
| S-3 | seq 규칙 결정 — 회차마다 0 부터가 아니라 노드 수명 동안 이어짐 (0 부터 다시 하면 수신 쪽이 새 회차 샘플을 중복으로 버림. sync 는 run_id + from_seq 라 지장 없음). PM 에 `interfaces.md` IF-08 비고 한 줄 요청 | `docs/interfaces.md` | IF-08 비고에 적힘 | P2 |
| S-4 | 시험 — 단위: seq 복원 · sync 범위 함수를 기존 시험 파일에 더함. 실기: 관제 PC 에서 `ros2 topic echo` 받다가 Wi-Fi 30 초 차단 → 복구 → `ros2 service call /robot3/survey/sync scv_msgs/srv/SyncSamples "{run_id: N, from_seq: K}"` → 받은 seq 집합과 CSV 비교 누락 0 · 중복 0 (`docs/TEST_PLAN.md` survey_buffer 행) | `test/` · 일지 | `✅ 단위테스트 통과` 블록 | P2 |
| S-5 | sync 호출 주체 — 설계는 `ros_bridge`(관제 B). 10/14 통합은 관제 A 의 `cell_map_writer` 가 임시로 맡음 (DB-5), `ros_bridge` 가 생기면 넘김 | — | 4-7 · DB-5 에 연결 | P2 |
| S-6 | 문서 · 주인 — README · CODE_WALKTHROUGH 의 `manual_tagger` 설명 고침, 5절에 RSSI A 와 파일 주인 확인 | `scv_survey/README.md` · 이 문서 | — | P3 |

위험: `status` 가 오기 전 샘플은 run_id 0 (`cell_map_writer` 는 자기 run 을 쓰므로 무해). 재발행이 depth 50 을 넘지 않게 속도 제한.

### DB 연결 (2-10 · D4 · 4-7 · 4-4 를 한 줄로)

확인한 것 (10/14): `inspection_manager` · `cell_map_writer` 는 이미 `backend:=db` 로 `cellmap_store.DbStore`(psycopg, `SCV_DB_DSN`)를 써서 `runs` · `grid_stats` · `aps` 에 직접 씁니다 — 실제 PostgreSQL 로는 한 번도 안 돌림 (2-10). `DbStore` 는 `maps.is_active` 지도와 그 `grid_cells` 를 전제로 하는데, 격자는 웹 서버가 지도 업로드 때 만들고 **기본 격자가 1.0 m** 라 0.5 m 로 올려야 `cellmap_store` 의 (ix, iy) 와 맞습니다. 웹의 섹터는 ix 절반 규칙, 관제 A 는 `zones.yaml` 폴리곤이라 서로 다릅니다. 원본 샘플 표 `net_samples`(PK robot_id · run_id · seq) 는 ROS 경로에서 아무도 쓰지 않고 웹 경로(`client_page` → `/api/agent/samples`)만 씁니다 — 이 PK 가 그대로 「중복 0」 장치이고 `last_seq` 조회가 sync 호출 근거라 4-7 · S-5 를 여기서 한 번에 풉니다. `runs` 는 `DbStore.open_run` 과 웹 `POST /api/runs` 둘이 만듭니다 (4-4 · D2 · D4).

| # | 할 일 | 파일 · 명령 | 완료 기준 | 우선 |
| --- | --- | --- | --- | --- |
| DB-1 | 관제 PC DB 준비 — `ros2_ws/scv_db/docker/setup_db.sh --test` 로 시험 DB, `SCV_DB_DSN` 환경변수(값은 `scv_db/README.md` 의 접속 정보, 문서에 적지 않음), psycopg 설치 확인 | `ros2_ws/scv_db` | `python3 -c` 접속 · `maps` 조회 | P2 |
| DB-2 | 지도 · 격자 · AP 등록 — 웹 `/api/maps/upload` 로 `testbed_v1` 을 `grid_size_m=0.5` 로 올리고 격자 생성 · 활성, `aps` 에 핫스팟 SSID(측정 AP). `grid_cells` 의 (ix, iy) · 원점 · 칸 수가 `cellmap_store.free_cells` 와 같은지 비교 (5절 「칸 원점」 질문을 여기서 닫음). 섹터 열은 웹 표시용으로만 두고 관제 A 는 `zones.yaml` 만 씀 | `server_page.py` · `cellmap_store.py` | 칸 수 · 원점 일치. 어긋나면 정섭 님과 규칙 합의 | P2 |
| DB-3 | `backend:=db ap_ssid:=<SSID>` 한 회차 (2-10) — mock 먼저, 그다음 robot3. `runs` 열림 → 닫힘(done / partial), `grid_stats` 칸 수가 파일 백엔드와 같음, 재측정 `update_cell` 반영. DB 를 끄면 `DB_UNAVAILABLE` 알림 | `control_pc.launch.py` | 회차 1건이 DB 에 완결 | P2 |
| DB-4 | 원본 샘플 저장 — db 백엔드일 때 `cell_map_writer` 가 NetSample 을 `net_samples` 에 그대로 INSERT … ON CONFLICT DO NOTHING (`ap_id` · `cell_id` 는 `DbStore` 가 붙임, `task_id` 비움, 1초 묶음). `DbStore.write_samples` · `last_seq(robot_id, run_id)` 추가 | `cellmap_store.py` · `cell_map_writer.py` · `test_cellmap_store.py` | 로봇 CSV 줄 수 = `net_samples` 줄 수, 같은 seq 두 번 보내도 1건 | P2 |
| DB-5 | 끊김 복구 — `cell_map_writer` 가 seq 빈틈을 보거나 샘플이 N 초 끊겼다 다시 오면 `survey/sync(run_id, last_seq+1)` 호출. S-5 의 호출 주체를 당분간 관제 A 가 맡음 (관제 B `ros_bridge` 가 생기면 넘김) | `cell_map_writer.py` | Wi-Fi 30 초 차단 뒤 누락 0 · 중복 0 (TEST_PLAN survey_buffer 행, T6-3 ②③) | P2 |
| DB-6 | run 의 원본 하나 (4-4 · D2 · D4) — 권장: 통합 시연은 `inspection_manager` 의 `DbStore` 가 `runs` 를 열고 닫고, 웹은 `runs WHERE status='running'` 을 읽어 표시만, 웹 `POST /api/runs` · client 흐름은 끔. 「DB 쓰기는 관제 API 로」(v0.2.9)는 `ros_bridge` 뒤로 미룸 — PM · 정섭 확인 | — | D4 상태를 「10/14: DbStore 직접 쓰기」로 | P2 |
| DB-7 | 보고서용 나머지 표(`tasks` · `precise_measures` · `inspections`)는 4-5 · 4-6 에서. 10/14 에는 넣지 않음 | — | — | P3 |

위험: `DbStore` 가 질의마다 접속을 열므로 재발행 20 Hz 는 묶어서 넣음. DB 가 죽어도 `cell_map_writer` 는 메모리 격자로 계속 (칸 확정은 파일처럼 되지 않으므로 `DB_UNAVAILABLE` 로 알림).

### 단계 0. 인수 정리

진행 (10/13 밤): 0-3 · 0-4 끝. 병합한 `feature/control` 에서 다섯 패키지 빌드 성공, `scv_manager` + `scv_survey` 시험 87 passed. 0-1 은 D1 · D8 끝, D2 는 남음. 0-2 끝 (`docs/daily/barlide/2026-10-13.md`, `3908b72`).

| # | 할 일 | 완료 기준 | 선행 | 우선 |
| --- | --- | --- | --- | --- |
| 0-1 | PM 에게 D1(인계 시점 · 방법)·D2·D8 을 묻고 답을 3절 표에 적음 | 세 줄의 상태가 「확정」 | — | P0 |
| 0-2 | 오늘 일지 제출 | 허브 일지 화면에 제출로 표시 (21:00 전) | D8 | 끝 |
| 0-3 | `temp/bundle-test-1012` 의 `scv_manager` 변경을 줄 단위로 읽음 (`inspection_manager.py` +388, `manager_core.py` +55, `route_builder.py` +85, 시험 3개) | 코드리뷰에서 각 함수가 왜 있는지 설명할 수 있음. 이상한 곳은 5절에 추가 | — | 끝 |
| 0-4 | 임시 브랜치에서 빌드와 시험을 직접 돌림 | `colcon build` 성공, `scv_manager`·`scv_survey` 시험 통과 수 기록 (문서값 87) | 0-3 | 끝 |

### 단계 1. 임시 구현을 `feature/control` 로

진행 (10/13 밤): 1-1 – 1-4 끝. 병합이라 `package.xml` 의 `nav_msgs` · `scv_survey` 도 함께 들어왔습니다. 정섭 님 커밋(`cbeb6cc`)과 겹친 파일은 없었고, `cbeb6cc` 가 더한 `aps.is_target` · `net_samples.task_id` 는 임시 `DbStore` 의 SQL 과 부딪히지 않습니다 (실제 DB 로는 미확인 — 2-10). 주행은 임시 브랜치에 든 것 대신 최신 `feature/driving` 을 함께 병합했습니다. 빈 마스크를 실제 Nav2 가 받는지는 2-2 에서 봅니다.

| # | 할 일 | 바뀌는 곳 | 완료 기준 | 선행 | 우선 |
| --- | --- | --- | --- | --- | --- |
| 1-1 | D1 결정대로 반영 | (가)면 PM 의 병합을 `git pull` · (나)(다)면 PM 과 합의한 방식 | `feature/control` 에서 `inspection_manager` 가 켜지자마자 `/robot4/keepout_mask`·`/robot3/keepout_mask` 를 냄 | 0-1 | 끝 |
| 1-2 | 반영 뒤 정섭 님 최근 커밋(`cbeb6cc`)과 함께 빌드되는지 확인 | — | `scv_msgs`·`scv_bringup`·`scv_survey`·`scv_manager` 빌드 성공, 시험 통과 | 1-1 | 끝 |
| 1-3 | `package.xml` 의존성 확인 | `scv_manager/package.xml` 에 `nav_msgs`·`scv_survey` | 깨끗한 워크스페이스에서 `rosdep`·빌드가 됨 | 1-1 | 끝 |
| 1-4 | `docs/CONTROL_MANAGER_STATUS.md` 를 지금 상태로 고침 | 2절 단계 표, 6절 미결정 목록, 7절 제한 사항 | 문서의 「구현 상태」가 코드와 같음 | 1-1 | 끝 |

### 단계 2. 묶음 시험 (T6-3 · T6-4 · T5-8 · T2-6)

진행 (10/13 밤): 2-1 끝 — 측정 → Dock → 칸 확정(Z1 30칸 · Z2 30칸) → 재측정 3칸(`CLEARED` 1 · `REMAINS` 2) → Dock → 회피 칸 2개 → SHT-02 점검 · SHT-01 `KEEPOUT_BLOCKED` → Dock → `RUN_COMPLETED`. 2-11 · 2-12 는 주행 수정(`4f151b4` · `b52cc7a`)이 병합됐으므로 실기에서 다시 봅니다.

실행 순서는 `ros2_ws/src/scv_manager/BUNDLE_TEST.md` 를 따릅니다. 로봇은 PM 이 허락하고 다른 팀이 쓰지 않을 때만 움직입니다. 로봇 PC launch 는 켜고 10초 뒤 `mission_executor` 가 스스로 undock 하므로 주변을 비우고 켭니다.

| # | 할 일 | 완료 기준 | 선행 | 우선 |
| --- | --- | --- | --- | --- |
| 2-1 | 관제 PC 한 대에서 mock 으로 전체 흐름 | `mock_execute_task_server.py` 만으로 측정 → Dock → 칸 확정 → 재측정 → Dock → 마스크 → 설비 → Dock → `RUN_COMPLETED` | 0-4 | 끝 |
| 2-2 | 빈 마스크와 Nav2 (IF-10) | 관제를 먼저 켠 뒤 `ros2 topic info /robot4/keepout_mask -v` 에 발행 1(`inspection_manager`) · 구독 2(global·local costmap). Nav2 가 `Managed nodes are active` 까지 감 | 1-1 · 로봇 1대 | P1 |
| 2-3 | 작업 1건 왕복 (IF-11) | `/inspection/start` → robot4 가 undock 해 섹터 12점을 돌고 Dock, 관제 로그에 `작업 완료` | 2-2 | P1 |
| 2-4 | 상태 수신과 끊김 (IF-12) | 관제에서 status 1 Hz 수신. 로봇 PC 를 끄면 30초 안에 E3 알림과 실행 작업 `FAILED(OFFLINE)` | 2-3 | P1 |
| 2-5 | 비상정지 · 재개 (IF-16) | `/inspection/estop` → goal 취소 · 로봇 정지 · E7. `/inspection/resume` → 같은 task_id 가 한 번만 다시 감 | 2-3 | P1 |
| 2-6 | 칸 맵 확정과 재측정 — **핫스팟을 켠 채로** (10/12 밤 시험은 꺼져 있어 16/17칸이 미검출) | 관제 로그 `칸 맵 확정: Z1: N칸 (음영 a · 미검출 b · low c)` 가 로봇 PC 히트맵과 맞음. 재측정 칸마다 `CLEARED` 또는 `REMAINS`. 칸 수 · 재측정 전후를 기록 | 2-3 · 2-12 · `precise_server` | P2 |
| 2-7 | 회피 마스크 | 핫스팟을 끈 채 재측정까지 → 관제 로그 `회피 칸 N개`, RViz 의 global costmap 에 그 칸이 장애물로 보임, 5초 안에 재계획 | 2-6 | P2 |
| 2-8 | 설비 점검 | SHT-01·SHT-02 두 건이 robot4 에 가고 결과 알림이 옴. 관측 자리가 회피 칸이면 `KEEPOUT_BLOCKED` | 2-7 · 비전 서버(가짜 또는 실제) | P2 |
| 2-9 | 두 대 동시 | 두 로봇이 함께 출발해 각자 Dock 으로 돌아오고 회차가 `RUN_COMPLETED` 로 끝남 | 2-3 을 로봇마다 | P2 |
| 2-10 | DB 백엔드 | `backend:=db` 로 한 회차. `runs` 에 회차가 열리고 닫히며 `grid_stats` 에 칸이 들어감 (임시 문서: "db 쪽은 아직 실제 DB 로 돌려 보지 못했다") | D4 · 관제 PC 의 PostgreSQL 에 `testbed_v1` 0.5 m 격자가 활성 → 절차는 DB-1 – DB-3 | P2 |
| 2-11 | 측정이 진행률 50 % 에서 완료로 끝난 원인 (10/12 밤 시험) | 원인이 관제 · 주행 중 어느 쪽인지 찾아 고치거나 기록. 12점을 다 돈 뒤에 완료가 옴 **10/13 robot3 실기에서 찾은 원인**: 경로 점이 안쪽 벽 조각에 붙어 있었음 (Z2 0번 0.08 m, Z1 · Z2 6번 0.18 m — 로봇 반경 0.19 m 안). 그 점의 0.35 m 안에 못 들어가 waypoint 가 지워지지 않고 되돌아감. Z1 6번은 12점 중 절반이라 10/12 밤의 50 % 와 맞음 (추정). 3-3 으로 고침 | 2-3 | P1 |
| 2-12 | 측정 뒤 DOCK 작업이 로봇 PC 쪽에서 취소된 원인 (10/12 밤 시험) | 측정 뒤 DOCK 이 성공하고 재측정 작업이 나감. 주행 10/13 수정(「BLOCKED 뒤 Nav2 취소 버그」)으로 풀렸는지 확인 | 2-3 | P1 |

### 단계 3. 임시 구현에서 고칠 것

임시 브랜치의 코드를 읽으며 찾은 것입니다. 실기 시험에서 문제가 되는 것부터 합니다.

| # | 무엇이 문제인가 | 고칠 방향 | 완료 기준 | 우선 |
| --- | --- | --- | --- | --- |
| 3-1 | 단계가 회차 전체에 하나라 먼저 끝난 로봇이 기다림 (D5 · 허브 T6-5 의 「남은 차이」) | 단계를 로봇별로 두고, 마스크도 그 로봇이 점검에 들어갈 때 그 로봇 토픽에만 | 한 로봇의 측정을 늦춰도 다른 로봇이 재측정 · 점검으로 넘어감. 한 대만 재측정이면 다른 한 대는 바로 점검 | P2 |
| 3-2 | 마스크를 내고 0.5초 뒤 타이머에서 바로 점검 작업을 보냄. Nav2 가 마스크를 반영했는지 보지 않음 | 발행 뒤 짧은 대기를 두거나 costmap 갱신을 확인 | 점검 첫 경로가 회피 칸을 지나지 않음 (10회) | P2 |
| 3-3 | 벽 여유를 사각형 `wall_bounds` 로만 봄. 지도 안쪽 벽 조각 네 군데(대략 y 0.03·1.83 의 x 0.1–0.5, y 3.62 의 x 2.0–2.7, y 4.83 의 x 0.1–1.3)는 걸러지지 않음 | `testbed_v1.pgm`(16비트 P5, 69 × 138)의 점유 픽셀에서 0.45 m 안의 경로 점 · 재측정 칸을 뺌 | Z2 경로와 재측정 칸이 벽 조각 0.45 m 밖. 실기에서 `BLOCKED skipped` 없음 | 끝 (10/13) — 경로 점과 재측정 칸을 지도 점유 픽셀에서 `obstacle_clearance_m`(0.30 = 로봇 반경 0.19 + 간격 0.10) 이상 떨어뜨림. 실기 확인은 남음 |
| 3-4 | 재측정이 로봇마다 5칸(`max_remeasure_per_robot`)에서 잘림. 잘린 칸은 재측정 없이 마스크로 감 | 시연 시간과 맞춰 값을 정하고, 잘린 칸 수를 알림으로 | PM 과 정한 값, 잘림이 있으면 알림 1건 | P2 |
| 3-5 | 표본 1개(`low`) 칸은 음영이어도 마스크에 안 들어감. 「샘플 2개 이상인 칸만」은 허브 T6-3 에서 정해진 규칙 | 규칙은 그대로 두고, 측정 속도 · 줄 간격을 RSSI · 주행과 맞춰 low 를 줄임 | 한 회차에서 low 칸 수 기록 | P2 |
| 3-6 | IF-15 후보와 칸 맵 후보가 함께 들어오면 같은 칸이 두 번 (D3) | 한 경로만 받도록 파라미터로 고름 | 후보를 주입해도 칸당 작업 1건 | P2 |
| 3-7 | `robot_readiness` 가 `accepting_tasks=false` 면 DOCK 도 보내지 않음. 주행은 배터리 30 % 이하에서도 DOCK 은 받음 | DOCK 작업은 `accepting_tasks` 와 무관하게 보냄 | 배터리 값을 낮춰 주입해도 DOCK 이 감 | P2 |
| 3-8 | goal 거절 · `LOW_BATTERY` 뒤 그 로봇의 남은 작업이 PENDING 으로 남으면 회차가 끝나지 않음 | D6 대로 그 로봇의 남은 작업을 실패 처리하고 DOCK 만 남김 | 배터리 20 % 주입 시 회차가 `partial` 로 닫힘 | P2 |
| 3-9 | `zones.yaml` 의 `robot:` 을 읽지 않고 파라미터와 launch 의 `ZONE_OF` 표로 짝지음. 세 곳이 어긋날 수 있음 | `zones.yaml` 의 `robot:` 을 원본으로 | launch 에 로봇 이름이 한 번만 나옴 | P3 |
| 3-10 | 설비의 섹터를 관측 자리 polygon 으로 정함. 경계(y 3.29) 근처 설비는 뜻과 다르게 갈 수 있음 | `facilities.yaml` 에 `zone_id` 를 두자고 PM 에 요청 (공유 설정이라 팀 브랜치에서 고치지 않음) | 좌표가 채워진 설비 전부가 뜻한 로봇에 배정됨 | P3 |
| 3-11 | 관제를 다시 켜면 진행 중이던 회차 · 작업 상태가 사라짐 (메모리에만 있음). DB 에는 `running` 회차가 남음 | 시연은 `close_stale_run:=true` 로 넘기고, 복구는 풀사이클 뒤 | 재시작 절차가 문서에 있음 | P3 |
| 3-12 | `apply_remeasure` 가 `n_samples` 가 0 이면 `reason` 문자열에서 값을 다시 읽음. 허브 T6-3 · `interfaces.md` v0.2.10 은 "reason 문자열이 아니다"로 정함 | 주행이 결과 필드를 채우는 것을 확인한 뒤 문자열 읽기를 뺌 (10/13: 주행 `b839d82` 가 필드를 채우고 mock 도 같게 고침 — 로봇 PC 가 모두 새 `mission_executor` 로 바뀐 뒤 뺌) | `n_samples` 0 이면 바로 미검출 | P3 |
| 3-13 | `robot_pc.launch.py` 의 Nav2 설정 기본값이 `nav2.yaml`(MPPI 비교용)이었음. 주행의 운영 설정은 `nav2_rpp.yaml`(RPP)이고 `mission_executor` 의 허용 오차도 그 값과 맞춰져 있음 | 기본값을 `nav2_rpp.yaml` 로, `BUNDLE_TEST.md` 2절 명령도 같게. PM 에 알림 | 인자 없이 띄워도 controller 가 RPP | 끝 (10/13) |
| 3-14 | 재측정 대상에 미측정 칸이 들어 있었음 (`include_unmeasured` 기본 true). 문서의 재측정은 음영 · 미검출 후보의 30초 정밀 측정이고, 10/14 실기에서 전 칸 정상인데도 미측정 칸 1개에 30초 측정 + Dock 왕복이 돌았음 | 기본값을 false 로. 미측정 칸은 N-8 의 메우기 단계에서 | 전 칸 정상이면 재측정 작업 0건 | 끝 (10/14, 실기 확인은 다음 회차) |

### 단계 4. 관제 B 와 잇기 (T6-4 「저장·표시」 · T6-6)

D2·D4 가 정해져야 시작할 수 있습니다. 아래는 권장안 (가)일 때입니다.

| # | 할 일 | 주인 | 완료 기준 | 우선 |
| --- | --- | --- | --- | --- |
| 4-1 | 웹의 순찰 시작 · 비상정지 · 재개 버튼이 `/inspection/start`·`estop`·`resume` 을 부름 | 관제 B (`ros_bridge`) | 버튼으로 로봇이 출발 · 정지. 한 대 순찰이 웹에서 시작부터 종료까지 보임 (T6-4, P0) | P1 |
| 4-2 | `/inspection/alerts` 를 웹에 표시 | 관제 B | 후보 · 해제 · 확정 · 장애가 5초 안에 화면에 (T6-3 완료 기준 ④, T6-5 의 화면 경보) | P2 |
| 4-3 | 알림 코드 목록을 정섭 님에게 전달하고 PM 에 `interfaces.md` 반영 요청 | 관제 A | 아래 「알림 코드」 표가 `interfaces.md` 에 들어감 | P1 |
| 4-4 | run_id 를 한 곳에서만 만듦 (지금은 임시 저장소와 웹 서버가 각자 `runs` 에 넣음) | 관제 A · B | 한 회차에 `runs` 줄 하나 | P2 |
| 4-5 | 작업 상태(PENDING · ACTIVE · DONE …)를 DB `tasks` 에 남길지, 남긴다면 누가 쓰는지 | 관제 A · B | 결정과 구현 또는 "남기지 않음" 기록 | P2 |
| 4-6 | 보고서 · 점검표에 필요한 값이 DB 에 다 있는지 확인 (칸 상태, 재측정 결과, 판정 · 사진) | 관제 B, 관제 A 는 값 제공 | 한 회차 뒤 지도 · 보고서(HTML) · 점검표(CSV)가 저장되고 화면에 보임 (T6-6) | P2 |
| 4-7 | 측정 샘플 저장과 sync (IF-08·09) | 관제 B (`ros_bridge`) · RSSI A | 같은 (robot_id, run_id, seq) 는 1건만 저장, seq 누락 뒤 sync 로 누락 0 (T6-3 완료 기준 ②③). 지금 임시 구조에서는 `cell_map_writer` 가 샘플을 칸으로만 모으므로 누가 원본 샘플을 저장하는지부터 확인 → 10/14 통합은 DB-4 · DB-5 (관제 A 가 `cell_map_writer` 로 `net_samples` 저장 · sync 호출), `ros_bridge` 가 생기면 넘김 | P2 |
| 4-8 | 셔터 SHT-01·SHT-02 표시 규칙 (T6-6) | 관제 B | 저장은 두 건 그대로, 점검표 · 보고서에서는 「방화셔터」 한 줄 — 하나라도 이상이면 이상, 이상 없이 미확인이 있으면 미확인, 둘 다 정상이면 정상 | P2 |

알림 코드 (`/inspection/alerts` 의 `code`, 임시 브랜치 포함):

| 코드 | 수준 | 언제 |
| --- | --- | --- |
| `RUN_STARTED` · `RUN_COMPLETED` · `RESUMED` | INFO | 회차 시작 · 종료 · 비상정지 뒤 재개 |
| `CANDIDATE` | WARN | IF-15 네트워크 후보를 받아 대기열에 넣음 |
| `REINSPECTION_DONE` | INFO | 재측정 작업 1건 완료 |
| `CLEARED` · `REMAINS` | INFO · WARN | 재측정 뒤 칸이 정상 · 음영 또는 미검출로 남음 |
| `INSPECTION_NORMAL` · `CONFIRMED` | INFO · ERROR | 설비 판정 정상 · 이상 |
| `FACILITY_SKIPPED` · `KEEPOUT_BLOCKED` | INFO · WARN | 좌표 없는 설비를 건너뜀 · 관측 자리가 회피 칸이라 미확인 |
| `E1` · `E3` · `E6` · `E7` | WARN · ERROR · WARN · ERROR | 막힘 · 로봇 응답 없음 · 3회 관측 뒤 미확인 · 비상정지 |
| `TASK_FAILED` · `DB_UNAVAILABLE` | ERROR | 작업 실패 · 칸 맵 저장소를 열지 못함 |

### 단계 5. 예외 시험 (T6-5)

| # | 예외 | 방법 | 완료 기준 | 우선 |
| --- | --- | --- | --- | --- |
| 5-1 | E3 로봇 이탈 | 로봇 PC 한 대 종료 | 30초 안에 감지 · 알림, 다른 로봇은 계속, 회차가 닫힘 | P2 |
| 5-2 | E7 비상정지 | 측정 중 `/inspection/estop` | 두 로봇 goal 취소 · 정지, 재개 뒤 이어서 완료 | P2 |
| 5-3 | E1 막힘 | 통로에 상자 | `BLOCKED skipped=N` → E1 알림, 그 로봇이 Dock 으로 돌아옴 | P2 |
| 5-4 | E4 관제 PC 끊김 | 관제 PC Wi-Fi 끄기 | 로봇은 받은 작업을 끝냄. 다시 이은 뒤 관제가 결과를 받는지, 못 받으면 어떻게 되는지 기록 | P2 |
| 5-5 | E2 배터리 | 낮은 배터리 값 주입 | 3-7 · 3-8 의 기준 | P2 |
| 5-6 | BLOCKED 재방문 (T6-5 시험 ⑤) | 통로를 막았다가 치움 | 지금은 건너뛴 점을 다시 가지 않고 작업을 BLOCKED 로 닫음. 재방문을 구현할지, 기록만 할지 PM 에 확인한 뒤 그 기준으로 | P2 |
| 5-7 | E8 DB 쓰기 실패 (T6-5 시험 ⑥) | DB 를 끊고 한 회차 | 큐 · 재시도 · 경보. 지금 관제 A 는 칸 맵 읽기 · 쓰기 실패를 로그로만 남기고 넘어감 — 관제 A 와 `ros_bridge` 중 누가 맡는지부터 정함 | P2 |

### 단계 6. 문서 · 리뷰 · 일지

| # | 할 일 | 완료 기준 | 우선 |
| --- | --- | --- | --- |
| 6-1 | 10/14 10:30 System Monitor 코드리뷰 준비 | 단계 흐름 그림 1장, 2.1 표, 3절 미결정 목록, 시험 결과를 설명할 수 있음 | P1 |
| 6-2 | 매일 일지 (21:00 전) | 시험을 통과하면 `- ✅ 단위테스트 통과:` 블록으로 | 매일 |
| 6-3 | 시험 결과를 `docs/CONTROL_MANAGER_STATUS.md` 5절에 추가 | 실기 시험마다 run_id · 결과 한 줄 | P2 |
| 6-4 | 이 문서의 3절 · 4절 상태 갱신 | 결정 · 완료가 생길 때마다 | 매일 |

## 5. 다른 팀에 확인·요청할 것

| 누구 | 내용 |
| --- | --- |
| PM (도윤) | D1(인계 시점)·D2·D4·D6·D8. 10/12 밤 시험의 50 % 완료 · Dock 취소 로그. BLOCKED 재방문과 E8 의 범위. `facilities.yaml` 의 `zone_id` 추가 |
| 정섭 (관제 B) | D2·D4. 웹과 `inspection_manager` 를 잇는 `ros_bridge` 를 누가 언제 만드는지. 알림 코드 표 전달. DB 연결: `testbed_v1` 을 0.5 m 격자로 등록(DB-2), 웹 섹터 규칙(ix 절반)과 `zones.yaml` 의 차이, `runs` 원본을 `inspection_manager` 로(DB-6) |
| RSSI (성현 · 희재) | D3. `cellmap_store`·`cell_map_writer`·`precise_server` 를 `feature/survey` 로 가져갈지. 칸 원점이 지도 origin(−0.150, −0.250)과 같은지. low 칸을 줄일 측정 속도. `survey_buffer`(S-1 – S-3)를 관제 A 가 `feature/control` 에 만들어도 되는지, seq 규칙(S-3) |
| 비전 (동준 · 재민) | D7. 두 로봇용 `inspect_facility` 서버와 IF-19 발행이 어디에 있고 언제 로봇 PC 에 올라가는지. `facilities.yaml` 의 EXT-01·EXL-01·RCK-01 좌표 |
| 주행 (본인) | 배터리 30 % 거절 · 20 % 자동 dock 과 관제 정책(3-7 · 3-8)을 맞춤. 마스크 반영 뒤 재계획 시간 측정(3-2) |

## 6. 명령 모음

```bash
# 빌드 (가상환경을 끄고)
deactivate 2>/dev/null
cd ros2_ws
colcon build --packages-select scv_msgs scv_bringup scv_survey scv_manager --symlink-install
source install/setup.bash

# 관제 A 단위 시험 — ROS 없이 도는 것만
cd src/scv_manager && /usr/bin/python3 -m pytest -q test/test_manager_core.py test/test_route_builder.py -p no:cacheprovider

# 관제 PC — 로봇 PC 의 Nav2 보다 먼저
ros2 launch scv_bringup control_pc.launch.py robots:=robot4
ros2 topic echo /inspection/alerts

# 시작 · 멈춤
ros2 service call /inspection/start  std_srvs/srv/Trigger
ros2 service call /inspection/estop  std_srvs/srv/Trigger
ros2 service call /inspection/resume std_srvs/srv/Trigger

# 마스크 확인
ros2 topic info /robot4/keepout_mask -v
```

## 7. 위험

| 위험 | 영향 | 대응 |
| --- | --- | --- |
| 관제 A 담당이 바뀌었고 주행 담당이 겸함. 10/13 은 주행 묶음 시험 세 개와 겹침 | 단계 2 전체 | 새 구현보다 임시 브랜치 반영과 시험을 먼저. P3 는 통합 뒤로 |
| 실기는 한 대로 재측정 대상 선정까지만 갔음 (10/12 밤). 재측정 이후 · DB 경로 · 두 대 동시는 미확인 | 2-6 – 2-12 | 핫스팟을 켜고 로봇 한 대 · `backend:=file` 로 먼저 완주, DB 와 두 대는 그다음 |
| 관제 웹과 `inspection_manager` 가 따로 있음 (D2) | T6-4 · T6-6 · 시연 화면 | 오늘 안에 원본을 정함. 안 되면 시연은 서비스 호출로 시작하고 화면은 DB 표시만 |
| `scv_manager` 가 `scv_survey` 의 임시 파일 세 개에 기댐 | 빌드 · 팀 브랜치 병합 | D1·D3 로 주인을 정함 |
| 관제를 늦게 켜거나 죽으면 주행의 costmap 이 준비되지 않음 | 모든 주행 시험 | 실행 순서를 `BUNDLE_TEST.md` 대로 지킴. 관제는 저장소를 못 열어도 빈 마스크는 냄 |
| 설비 좌표가 셔터 두 건뿐 | 설비 점검 시연, robot3 의 점검 단계 | 비전 팀의 좌표를 기다리되 흐름은 SHT-01·02 로 확인 |
| 로봇 PC 가 시험용 `manual_tagger` 로 샘플을 냄 — run_id 고정 · 버퍼 없음 · 끊기면 그 구간 샘플이 사라짐 | T6-3 완료 기준 ②③ (누락 0 · 중복 0), DB 의 `net_samples` | S-1 – S-4 · DB-4 · DB-5 |
| `backend:=db` 가 실제 DB 로 검증된 적 없고 웹 격자 기본값(1.0 m)이 칸 맵(0.5 m)과 다름 | DB 회차 · 보고서 | DB-1 – DB-3 을 mock 으로 먼저, 격자는 0.5 m 로 등록 |

## 8. 저장소 메모

- 이 PC 의 git 사용자는 `shshchoi` 입니다. 여기서 커밋하면 그 이름으로 남습니다. push 는 `gh` 로 로그인한 `barlide` 계정으로 나갑니다 (10/13).
- `feature/survey` 에서 작업하던 미커밋 변경 4개(`iw_reader.py` · `manual_tagger.py` · `network_map_engine.py` · `rssi_scanner.py`)는 `stash@{0}` 에 있습니다.

## 9. 근거

직접 확인한 것 (10/13 저녁):

- `feature/control`(`ddc5f4d`)의 `scv_manager` 전체, `docs/interfaces.md`, `docs/CONTROL_MANAGER_STATUS.md`, `docs/CRITICAL_PATH.md`, `docs/TEST_PLAN.md`, 현민 · 정섭 님 일지.
- `temp/bundle-test-1012`(`faa34c4`)의 `scv_manager` 변경 전체, `BUNDLE_TEST.md`, `control_pc.launch.py`, `cellmap_store.py` · `cell_map_writer.py` · `robot_pc.launch.py` 의 개요.
- `feature/driving` 의 `mission_executor.py` 와 Nav2 keepout 설정, `feature/survey` 의 `network_map_engine.py` 개요, `ros2_ws/scv_app` 의 README 10장과 API 경로, `scv_db/sql/01_schema.sql` 의 표 구조.
- 운영 `zones.yaml`·`facilities.yaml` 로 `feature/control` 의 경로 생성을 돌려 본 결과, `manager_core`·`route_builder` 단위 시험 14개 통과.
- 노션 「COOP Hub 일정 (자동 반영)」의 T6 · T2 · T5-8 행.
- COOP Hub 스케줄 화면과 T6-3 · T6-4 · T6-5 · T6-6 · T2-6 의 세부(할 일 · 완료 기준 · 정해진 것) — 10/13 밤, 현준 계정.

확인하지 못한 것:

- COOP Hub 의 태스크 · 일지 화면, T6-1 과 T5 Step 세부, 채팅.
- (10/13 밤에 풀림) `test_inspection_manager.py` 를 포함한 시험 87개 — 병합한 `feature/control` 에서 다시 돌려 통과했습니다.
- 10/12 밤 실기 시험의 로그와 수치. 허브 T2-6 의 요약만 봤습니다 (PM 의 10/12 · 10/13 일지는 양식 그대로).
- 비전 브랜치의 액션 서버 위치, `feature/vision` 의 나머지 코드.
- `cellmap_store.py` 의 세부 동작과 `scv_app` 서버 코드 본문 (경로와 표 이름만 봤습니다).
