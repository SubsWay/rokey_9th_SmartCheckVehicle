# SCV 관제 웹 — 3-3차

통신 구조 안 C를 따르는 3차 구현의 세 번째 단계입니다. DB는 **scv_db_3차**(섹터 판)를 씁니다.
3차는 로봇 1대(= 섹터 1개)로 검토하고, 두 대 운행 · 경로 · 섹터 · 판단 규칙 교체는 4차에서 합니다. 이후 차수는 **3-4 정지 · 3-5 보고서**입니다(3-3 ~ 3-6을 3-3 작업 차수 하나로 합침).

| 차수 | 내용 |
|---|---|
| 2_C-1 | 관제 PC server page — PostgreSQL 기준 정보 7개 표 |
| 2_C-2 | 제어 PC client page · 가상 로봇 robot3 — 로그인, 로봇 활성 알림(hello), 로봇 정보·카메라 |
| **3-1** | **가상 세계 묶음**(시험 지도 · world.yaml · 등록 스크립트), **Redis 도입**(로그인 세션 · 로봇 상태 · 응답 없음 E3), client **status 1 Hz** → robot_status_log, 모니터에 **로봇 위치·상태 실시간 표시**(`/ws/ui`) |
| **3-2** | **로봇 2대 동시 운행 · 섹터 방식** — 격자를 만들 때 **섹터 1 · 2**로 나눔(임시 규칙 `split_sectors`), 모니터에 섹터 색 표시 · 정보 탭에 섹터별 칸 수, **순찰 진행 중 격자 변경 잠금**, 순찰 중 현재 지도 바꾸기 잠금. **AP · 설비가 지도에 속함**(정보 탭에서 지도를 골라 추가 — 격자와 같은 방식, 설비 ID는 지도 안 유일), **도크 접근 자세**를 정보 › 지도 › 수정에서 입력 → 관제 PC가 제어 PC에 config로 보내고 client가 `initialpose`로 로봇 시작 위치 지정, 순찰 중인 지도의 AP · 설비 · 도크 잠금, 지도 삭제 때 그 지도의 격자 · AP · 설비 함께 삭제. 모니터에 **도크 · AP · 점검 설비** 표시, 활성 로봇의 **관제 PC 메모리 상태 전체** 표시. **Redis 제거** — 로그인 세션 · 로봇 최신 상태 · 응답 없음(E3) 판정을 관제 PC(server_page) 메모리로 옮김(9장). 가상 세계는 `world.yaml` 하나(측정·판정으로 알아낼 실제 세계 값만). DB: `grid_cells.sector` · `runs.sector_robots` · 로봇마다 ACTIVE task 1개 · 섹터별 실패(`partial`) · `maps.dock_poses` · aps `(map_id, ssid)` · facilities 키 `(map_id, facility_id)` |
| **3-3** | **작업 — 측정 · 정밀측정 · 탐지** — 모니터 「로봇 상태」 머리의 「순찰 시작」(섹터 배정 — 도킹 · 정상 로봇만, 로봇 1대 = 섹터 1개) · 「순찰 중단」. 시작 조건 = 현재 지도 · 격자 + 활성 **측정 AP**(정보 탭에서 지도마다 1개 지정, `aps.is_target`) + 로봇 도킹 · 정상 · 도크 접근 자세. 결과 확인은 **이력** 탭(현재 지도의 순찰, 읽기 전용). inspection_manager가 로봇마다 **측정**(SURVEY_SEGMENT, 지그재그 임시 경로 20점씩) → **정밀측정**(후보 칸 정지 정밀측정 → CONFIRMED · CLEARED → 확정 이상) → **탐지**(설비 판정 1회 + 사진, 정밀측정 · 설비 후보 없음) → **복귀**(DOCK)를 차례로 보냄. client: Nav2 `navigate_through_poses` · `undock` · `dock`, RSSI 표본(자기 섹터 칸만) → 칸 중앙값 후보, **아웃박스**(REST /api/agent/*). 실패 = 그 섹터만 중단(scv_fail_task) + 복귀, 끝 = scv_close_run. 실시간 히트맵 · 경로 · 후보 · 판정을 지도에, 최근 이벤트(진행 중 순찰의 것만) · AP 활성(client가 받은 SSID, 정보 탭에도 실시간). client 카메라 탭: 순찰 중에만 영상 + 제어 PC 지도 파일 위에 담당 로봇 위치 · 섹터 · 설비 · 측정 AP(run_config로 1회 받음) + 이번 순찰 후보 → 결론 · 설비 판정(사진). 순찰 끝 WS `run_end`. server_page 재시작 때 진행 중 run 복원 · 순찰 중 로봇 E3 보완. 가상 로봇: Nav2 · dock · undock 액션, rssi, 가상 판정(world.yaml 정답). **3-3 보완** — 이력 탭 지도가 처음부터 전체가 보이도록(이미지가 없으면 격자 범위로, 크기가 바뀌면 다시 맞춤, 「전체 보기」 ⤢ 버튼), task 표 맨 끝 **「자세히」**(측정: 지난 칸 · 칸별 측정값 · 후보 / 정밀측정: 정밀측정 · 결론 / 탐지: 판정 · 사진 크게 / 경로 · 이벤트, `GET /api/tasks/{task_id}`, DB `net_samples.task_id`). **가상 세계 다시 구성** — `virtual_world/testbed/world.yaml`(map · register · truth 세 구역), `register_world.py`가 지도 · 도크 · AP · 설비를 자동 등록하고 측정 AP는 활성이 될 때까지(기본 120초, `--no-target`이면 안 함) 기다려 지정, 가상 로봇은 가상 세계(지도 벽까지 반영한 RSSI · 판정 사진 · 순찰 영상)로 동작. **3-3 보완 2** — `register_world.py` · `robot3_virtual.py`를 옵션 없이 실행하면 testbed(`--world`로 다른 세계, 가상 로봇 `--no-world`면 로봇만), 화면의 「재측정」을 **「정밀측정」**으로, 순찰 종류는 **순찰 하나**(시작 창의 종류 선택 없앰), 「자세히」에 실제로 지나간 길 · 배터리 |

```
scv_app/
├─ server_page.py          관제 PC — FastAPI(화면 + API) · Uvicorn. client page용 로그인 API · WebSocket /ws/agent
├─ client_page.py          제어 PC — FastAPI(화면 + API) + rclpy(별도 스레드) + 관제 PC WebSocket 클라이언트 + 아웃박스(REST)
│                          실행하면 outbox_{robot_id}/ 폴더가 생김 (관제 PC로 보낼 기록 — --outbox로 위치 지정)
├─ robot3_virtual.py       시험용 가상 로봇 — ROS 2 노드, /robot3/… 토픽 · Nav2 · dock · undock 액션.
│                          기본 가상 세계 testbed(rssi · 가상 판정 · 판정 사진 · 순찰 영상), --world로 다른 세계, --no-world면 로봇만
├─ ros_check.py            ROS 통신 점검 — client와 같은 방식으로 로봇 토픽이 보이고 값이 오는지 확인
├─ virtual_world/          가상 로봇 시험 전용 세계 (3-3 다시 구성)
│  ├─ register_world.py            world.yaml(기본 testbed)의 map · register → server API로 지도 · 현재 지도 · 도크 · AP · 설비 · 측정 AP 등록
│  ├─ world_model.py               world.yaml 읽기 · 지도 벽 · RSSI 모델 (가상 로봇 · make_assets가 씀)
│  ├─ make_assets.py               판정 사진 · 순찰 영상을 합성으로 다시 만들 때 (묶음에 이미 있음)
│  └─ testbed/                     세계 하나 = 폴더 하나 (다른 지도는 폴더를 하나 더 만들어 --world로 고름)
│     ├─ world.yaml                map(지도 · 격자) · register(관제 PC 등록 값) · truth(실제 세계의 정답 — 가상 로봇만)
│     ├─ testbed.pgm · testbed.yaml  시험 지도 — 12 × 8 m 방(칸막이 · 작은 방 · 기둥 · 선반), 1 m 격자가 벽과 맞음
│     └─ media/                    drive.mp4(지도 1인칭 순찰 영상) · facilities/<설비 ID>.jpg(판정 사진)
├─ requirements.txt        관제 PC (server_page)
├─ requirements-client.txt 제어 PC (client_page · robot3_virtual) — rclpy 등은 ROS 2 Jazzy에서, 영상은 python3-opencv
└─ static/
   ├─ server/              server page 화면 (index.html · app.js · app.css)
   ├─ client/              client page 화면 (index.html · app.js · app.css — server/app.css 위에 덧붙임)
   └─ vendor/              인터넷 없이 쓰도록 동봉한 라이브러리
                           react-htm.mjs (React 19 + htm, 빌드 없이 사용) · leaflet.js · leaflet.css · images/
```

```
 로봇(robot3_virtual) ──ROS 2──▶ 제어 PC client_page ──HTTP · WebSocket──▶ 관제 PC server_page ──▶ PostgreSQL
   /robot3/battery_state ·         로그인 확인        POST /api/client/login
   dock_status · odom · ip ·       활성 알림          WS /ws/agent  hello { robot_id }
   amcl_pose(3-1) ·                status 1 Hz(3-1)   WS /ws/agent  status { x, y, yaw, battery_pct, nav_state … }
   ◀ initialpose(3-2) ◀──────────── config(3-2) ◀──── WS /ws/agent  config { map, dock_pose } (연결 · 지도 변경 · 도크 수정 때)
   ◀ navigate_through_poses ·      MissionExecutor ◀── WS /ws/agent  run_config · task_assign · task_cancel · run_cancel (3-3)
     undock · dock (액션, 3-3)     task_progress ───▶ WS /ws/agent  task_accepted · task_rejected · task_progress
   rssi (String JSON, 3-3) ──▶     SurveyBuffer ─┐
   virtual/detect_* (시험, 3-3)    DetectionAlert ┴─ 아웃박스 ─▶ REST /api/agent/samples · events · precise-measures ·
   oakd/rgb/image_raw/compressed                                inspections · media · tasks/{id}/result
                                                                        │
                                                       관제 PC 메모리 (로봇별 최신 상태) · PostgreSQL robot_status_log
                                                       server page 모니터 ◀── WS /ws/ui (1초마다)
```

## 0. 3-3차 시험 순서 (가상 로봇)

순서: **① server → ② world 등록 → ③ client → ④ robot** (가상 세계를 쓰지 않을 때의 server → client → robot 사이에 ②만 끼움)

```bash
# ① 관제 PC — server
cd scv_db/docker && ./setup_db.sh --test --reset              # 시험 DB robotdb4_test (PostgreSQL 컨테이너 1개)
cd scv_app
SCV_DB_DSN="host=127.0.0.1 port=5432 dbname=robotdb4_test user=rokey password=rokey" python3 server_page.py
# ② 관제 PC — world 등록 (server가 켜져 있어야 함)
cd virtual_world
python3 register_world.py      # = --world testbed/world.yaml --robot-map-path <이 PC의 virtual_world/testbed/testbed.yaml 절대 경로> --wait 120
#   지도 등록 + 현재 지도(격자 생성) + 도크 접근 자세 + AP 2개 + 설비 6개 — 정보 탭에서 손으로 넣을 것이 없음
#   이어서 측정 AP(world.yaml의 target: true = SCV_VAP_A)가 활성이 될 때까지 최대 120초 기다렸다가 지정
#   (활성은 client가 그 SSID 값을 받을 때 — 기다리는 동안 아래 ③ client를 켜고 로그인, ④ 가상 로봇을 켬)
#   --no-target: 등록만(측정 AP는 정보 › 측정 대상 AP › 목록 › 「측정 AP로」) · --wait 300: 더 오래 기다림
#   시간 안에 못 하면 「측정 AP 미지정」(종료 코드 2) — 다시 실행하거나 정보 탭에서 지정
#   제어 PC가 다른 PC면 --robot-map-path <제어 PC의 virtual_world/testbed/testbed.yaml 절대 경로>
#   (client의 지도 표시도 이 경로의 yaml · pgm을 읽음 — 제어 PC에 같은 파일이 있어야 함)

# 제어 PC (ROS 2를 source한 터미널 — 8장 Discovery Server 주의) — ② register_world가 기다리는 동안
# ③ client
python3 client_page.py --server http://<관제PC>:8000            # http://<제어PC>:8080 → admin 또는 user 계정으로 로그인
# ④ robot
python3 robot3_virtual.py      # = --world virtual_world/testbed/world.yaml. 빠른 시험: --speed 1.5 --rssi-hz 6
#   client와 robot은 어느 쪽을 먼저 켜도 됨 — client는 로그인 뒤 2초마다 로봇 토픽을 확인하고, 보이면 관제 PC에 접속해
#   도크 접근 자세를 받아 initialpose를 보냄(가상 로봇은 그때까지 기다림). 세 가지(client 로그인 · robot · rssi 수신)가
#   ②의 대기 시간(120초) 안에 끝나야 측정 AP가 지정됨 — 넘기면 register_world.py 다시 실행 또는 정보 탭

# 관제 PC — 순찰: 모니터 › 로봇 상태 머리의 「순찰 시작」 → 섹터 1 = robot3 (섹터 2는 「순찰 안 함」) → 순찰 시작
#   버튼이 꺼져 있으면 바로 아래 줄에 이유(측정 AP 없음 · 도킹 아님 · 도크 접근 자세 없음 …)가 나옴
```

**3-3차 순찰 1회에서 보이는 것** (testbed · robot3 · 섹터 1 · 측정 AP SCV_VAP_A)

1. 측정 — robot3가 undock(UNDOCK 이벤트) 후 섹터 1 칸을 지그재그로 돎(task 3개: 20 · 20 · 8점). 지도에 수행 중 경로(실선) · 대기 경로(점선)와 칸별 RSSI 히트맵(측정 AP SCV_VAP_A)이 쌓입니다. client 카메라 탭에는 순찰 영상이 움직일 때만 흐릅니다(멈추면 그 장면).
2. 후보 — world.yaml truth의 섹터 1 음영 구역(x 4~6, y 0~2)의 칸이 −72 dBm 미만이면 구간이 끝날 때 CANDIDATE 이벤트 → 후보 표(주황 점). 음영 밖 칸은 기둥 · 칸막이 뒤라도 −66 dBm 이상이라 후보가 되지 않습니다.
3. 정밀측정 — 측정이 모두 끝나면 후보마다 REINSPECTION: 그 지점에 서서 `precise_duration_s` 동안 정밀측정 → CONFIRMED(확정 이상 생성).
4. 탐지 — 섹터 1 설비 4개(관측 자세 칸 순서)를 차례로 1회 판정: EXL-02 위치 이상 · EXT-01 정상(압력계 green) · SHT-01 가림 → 자세 보정 3회 후 미확인(E6) · EXT-03 미검출. 판정마다 그 설비 사진(`media/facilities/<설비 ID>.jpg`) 1장이 media로 저장됩니다.
5. 복귀 — DOCK: 도크 접근 자세로 이동 → dock(DOCK 이벤트) → run done.
6. 이력 탭 — task 표 맨 끝 「자세히」로 task마다 지난 칸 · 칸별 측정값 · 후보 / 정밀측정 · 결론 / 판정 · 사진을 봅니다.

- 시험을 빨리 하려면 정보 › 판정 기준값에서 `precise_duration_s`를 5로 줄이고 가상 로봇을 `--speed 1.5 --rssi-hz 6`으로 켭니다. 칸마다 표본이 `min_samples`(3)개 이상이어야 후보가 되므로, 기본(0.3 m/s · 1 Hz)에서는 경로 끝 칸(섹터 1 음영의 x 5~6, y 0~1)이 표본 1개라 후보가 안 될 수 있습니다(`--rssi-hz 2` 이상이면 4칸 모두).
- 섹터 2(작은 방 음영 12칸 · EXT-02 이상 · EXL-01 정상)까지 보려면 `python3 client_page.py --robot-id robot4 --port 8081`(로그인)과 `python3 robot3_virtual.py --robot-id robot4`를 하나씩 더 켜고 섹터 2 = robot4로 시작합니다.
- 모니터 「순찰 중단」 → run aborted, 남은 task CANCELED, robot3는 하던 일을 멈추고 도크로 복귀합니다.
- 순찰 중 client를 끄면 `status_timeout_s` 뒤 E3 → 그 로봇의 task FAILED(NO_RESPONSE) → 남은 task 취소 → run 종료(로봇 1대면 aborted).

**가상 세계 world.yaml** (`virtual_world/testbed/world.yaml` — 경로는 이 파일 기준, 좌표는 지도의 map 좌표 m)

| 구역 | 누가 읽음 | 내용 |
|---|---|---|
| `map` | register_world · 가상 로봇 | 지도 yaml(`testbed.yaml` → pgm) · 지도 이름 · 격자 크기 |
| `register` | register_world.py → 관제 PC(DB) | 도크 접근 자세(robot3 (1.0, 0.6, 1.5708) · robot4 (7.0, 0.6, 1.5708)), AP(SCV_VAP_A (2.0, 6.5) 측정 AP · SCV_VAP_B (9.0, 3.5)), 설비 6개(ID · 종류 · 압력계 · 위치 · 관측 자세) — 정보 탭에서 손으로 넣던 값 |
| `truth` | 가상 로봇만 — DB에 넣지 않음 | AP 실제 신호 원점 · 세기 모델 · 벽 감쇠, 음영 구역, 설비 실제 상태 · 판정 사진, 순찰 영상 · 보이는 거리 |

- **RSSI = 지도 기반** — `p1m_dbm − 10·n·log10(거리) − wall_loss_db × (AP → 로봇 직선이 지나는 pgm 벽 덩어리 수, 최대 max_walls) − 음영 loss_db + 잡음`. 같은 거리라도 칸막이 · 기둥 · 선반 · 작은 방 벽 뒤는 약하고, 작은 방은 문 쪽이 조금 셉니다. 순찰은 측정 AP 하나만 재므로, 어느 AP를 측정 AP로 골라도 음영 밖은 −66 dBm 이상 · 음영 안은 −76 dBm 이하가 되도록 맞춰 두었습니다.
- **판정** — 관측 자세에서 설비 등록 위치가 `view_range_m`(2.5 m) 안이어야 그 자리를 봅니다. 멀거나 truth에 없는 설비면 정면 관측 실패(→ 미확인). `position`은 옮겨진 위치도 보이는 거리 안일 때만 검출(POSITION_ANOMALY), 멀리 옮겨졌으면 NOT_DETECTED.
- **판정 사진 · 순찰 영상** — `make_assets.py`가 만든 합성 파일(설비 종류 × 실제 상태로 그린 640 × 360 JPEG, 지도를 1인칭으로 그린 45초 mp4). 같은 이름의 실제 사진 · 영상으로 바꿔 끼워도 됩니다. world.yaml을 고쳤으면 `python3 make_assets.py testbed/world.yaml`로 다시 만듭니다.
- **다른 지도** — `virtual_world/<이름>/`에 지도 yaml · pgm과 world.yaml을 두고 `register_world.py --world <이름>/world.yaml` · `robot3_virtual.py --world virtual_world/<이름>/world.yaml`. 가상 로봇은 관제 PC의 현재 지도를 모르므로, 순찰하는 지도와 같은 world.yaml을 주어야 합니다.
- **--no-world** `python3 robot3_virtual.py --no-world` — 가상 세계 없이 로봇만(배터리 · odom · amcl · 이동 · 도킹 · 합성 카메라). rssi가 없어 AP가 활성이 되지 않고, 판정 요청은 시간 초과 → 미확인입니다.

- **로봇 시작 위치** — 가상 로봇은 켜지면 도크에 서 있지만 지도 위 위치를 모릅니다(amcl_pose 없음). client가 관제 PC에 연결되면 config로 도크 접근 자세를 받고, 로봇이 도킹 중이면 `/robot3/initialpose`로 보내 시작 위치를 정합니다. 도크를 아직 넣지 않았다면 정보 › 지도 › 수정에서 넣는 순간 다시 보내집니다. client 로봇 정보 탭의 「도크 접근 자세 (관제 PC)」 칸에서 받은 값과 보낸 시각을 볼 수 있습니다.
- 모니터의 격자가 **섹터 1(파랑, x 0~6 m · 48칸) · 섹터 2(주황, x 6~12 m · 48칸)**로 칠해지고, 범례에 섹터별 칸 수와 기본 배정(섹터 1 = robot3, 섹터 2 = robot4)이 나옵니다. 정보 › 격자 칸 패널에도 섹터별 칸 수 · ix 범위가 나옵니다.
- 지도 위에 **도크**(robot3 (1.0, 0.6) · robot4 (7.0, 0.6), 로봇 색 네모 + 점선 = 도크 방향), **AP**(📶 SCV_VAP_A · B), **점검 설비 6개**(소 = 소화기 · 셔 = 셔터 · 유 = 유도등 · 랙 = 랙, 점선 끝 점 = 관측 자세)가 나옵니다. 음영 구역은 측정 결과로만 표시합니다(모니터 · 이력 탭 히트맵).
- 「로봇 상태」에서 활성 로봇은 관제 PC 메모리에 있는 최신 상태의 모든 필드와 E3까지 남은 초를 그대로 보여 줍니다(1초마다 갱신).
- client를 끄면 「연결 끊김 — n초 뒤 응답 없음」 → `status_timeout_s`(30초) 뒤 **E3(응답 없음)**가 robot_events에 남고 「비활성」으로 바뀝니다.
- 등록할 때의 `--robot-map-path`는 제어 PC에서 testbed.yaml이 있는 경로입니다 — client는 task의 지도 파일이 이 경로에 없으면 MAP_MISMATCH로 거부합니다. 도크 위치는 가상 로봇이 파일에서 읽지 않고 관제 PC가 보낸 initialpose로만 압니다.

## 1. 설치·실행 (관제 PC)

```bash
cd scv_app
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python3 server_page.py                 # http://<관제PC>:8000  →  /server/login
```

| 환경변수 | 기본값 | 설명 |
|---|---|---|
| `SCV_DB_DSN` | `host=127.0.0.1 port=5432 dbname=robotdb4_postgresql user=rokey password=rokey` | scv_db_3차로 만든 DB (시험은 robotdb4_test) |
| `SCV_MEDIA_DIR` | `./scv_media` | 등록한 지도 PNG 저장 위치 (`maps/`). 더미 데이터(`make_dummy.py --media`)도 같은 곳으로 |
| `SCV_HOST` / `SCV_PORT` | `0.0.0.0` / `8000` | |

- 로그인은 수퍼 계정 **admin / admin**(`02_seed.sql`)만 됩니다. 일반 계정(권한 user)은 client page용이라 server page 로그인이 거부됩니다.
- 워커는 1개(프로세스 1개)로만 실행합니다. 제어 PC WebSocket 연결(`/ws/agent`), 로그인 세션, 로봇 최신 상태를 이 프로세스 메모리가 쥐고 있기 때문입니다. 워커 1개는 단일 스레드가 아니라 이벤트 루프 + 요청 처리 스레드 풀이며, 메모리 상태는 잠금 하나로 보호합니다. server를 다시 시작하면 다시 로그인해야 합니다(9장).
- HTTPS·nginx는 쓰지 않습니다(포트폴리오 범위, 보안 고려 안 함).

## 2. 화면

| 주소 | 내용 |
|---|---|
| `/server/login` | admin 로그인. 로그인 전에는 다른 탭이 비활성 |
| `/server/monitor` | **현재 지도**를 가운데에 표시(Leaflet, 좌표 단위 m — 이미지 왼쪽 아래 = yaml origin). 그 위에 **격자 칸**(grid_cells)을 칸마다 사각형으로 그리고 **격자 번호**(큰 숫자 = cell_id, 작은 숫자 = ix,iy)를 표시. 칸이 화면에서 26 px보다 작으면 번호를 숨기고, 46 px 이상이면 ix,iy도 보임. 순찰 중에는 실시간 경로 · 히트맵 · 후보 · 판정도 지도에. 아래 세 칸 = **로봇 상태**(머리에 「순찰 시작」 · 순찰 중이면 「순찰 중단」, 시작할 수 없으면 이유 한 줄) · **순찰**(진행 중 run · 측정 AP) · **이번 순찰 이벤트**(진행 중 run의 이벤트만, 순찰이 없으면 비움) |
| `/server/history` | **이력** 탭(3-3차, 읽기 전용). 현재 지도에서 수행한 순찰(run) 목록 → 하나를 고르면 상태 · 배정 · 기준선 재료 표시 · 그 run의 이벤트, 지도(경로 · 칸별 RSSI 히트맵 · 후보 · 설비 판정 — 로봇 상태 · 실시간 로봇 마커 없음), 아래 task 표(전체 · 측정 · 정밀측정 · 탐지 · 복귀 걸러 보기, 맨 끝 **「자세히」**) · 후보 → 정밀측정 결론 · 설비 탐지(사진). 진행 중 run이면 몇 초마다 다시 읽음. 지도는 들어오면 전체가 보이게 맞추고(이미지 파일이 없으면 격자 · 도크 · AP · 설비 범위 + 「지도 이미지 파일 없음」 알림), 화면 크기가 바뀌면 다시 맞춤 — 직접 확대 · 이동한 뒤에는 「전체 보기」(⤢)로. **「자세히」** 창(`GET /api/tasks/{task_id}`): 작은 지도(계획 경로점 = 점선, 실제로 지나간 길 = 실선 — 제어 PC status 1 Hz `robot_status_log`, 결과), 보낸 · 끝난 시각 · 걸린 시간 · 지나간 거리 · 배터리 시작 → 끝 · 사유, 그 task의 이벤트, **칸 순서**(계획 = 경로점마다 칸 — 경로점 번호가 지도에도 표시, `seg:a-b`는 칸 번호가 아니라 섹터 경로의 경로점 번호 a~b / 실제 = 지난 칸 순서 — 측정은 표본, 그 밖은 status 1초 위치. 계획 밖 칸 · 다시 지난 칸 · 못 지난 계획 칸을 색으로, 칸별 측정값 표에 「계획 #」), 그리고 측정 = 지난 칸(지난 순서) · 칸별 측정 AP 값(중앙값 · 표본 수 · 최소~최대, `net_samples.task_id`) · 이 구간 후보 / 정밀측정 = 후보 · 측정 때 칸 값 · 정밀측정(AP별 중앙값 · IQR · 표본) · 결론 · 확정 이상 / 탐지 = 설비 · 판정(detail · final · 정면 · 보정 · 압력계 · 신뢰도 · bbox) · 사진(누르면 크게) |
| `/server/master` | **정보** 탭. 오른쪽 세로 탭에서 표를 누를 때마다 선택·해제(여러 개 가능), 선택한 표는 선택한 순서대로 왼쪽에 세로로 쌓여 표 이름 · 설명 · 행 수 · 현재 값을 보여 줌. 다른 탭에 다녀와도 선택 유지. 「목록」 → 목록 창에서 추가 · 삭제 · 값 수정 |

**행 출력 규칙** — 정보 탭 패널과 목록 창 모두 **최신 순 10행**씩 보이고, 10행을 넘으면 아래 쪽 번호(‹ 1 … 4 5 6 ›)로 다음 10행을 봅니다. 추가하면 1쪽으로 이동합니다.

| 표 | 최신 순 기준 |
|---|---|
| robots | registered_at ↓ |
| users | created_at ↓ |
| settings | updated_at ↓ (값이 같으면 key) |
| aps | ap_id ↓ (목록 창, 지도별) |
| maps | created_at ↓ |
| grid_cells | cell_id ↓ (목록 창, 지도별) |
| facilities | facility_id ↓ (목록 창, 지도별) — 등록 시각 컬럼이 없어 ID 내림차순 |

maps · grid_cells · facilities 패널은 행 나열이 아니라 요약(칸 수 · 종류별 개수)이라 쪽이 없습니다. aps · facilities 요약 패널은 현재 지도의 것만 보이며 「현재 지도」 줄은 두지 않습니다(3-3차). aps 패널 · 목록 창의 활성 표시는 `/ws/ui`로 실시간 갱신됩니다.

## 3. 표별 동작 규칙

| 표 | 추가 | 수정 | 삭제 | 비고 |
|---|---|---|---|---|
| robots | ○ | ○ (주소) | ○ | robot_id 형식은 DB CHECK(영문 소문자 시작, 하이픈 없음). **활성은 client(제어 PC)가 값을 보내 줄 때만** — server page에서 바꿀 수 없고, 추가하면 비활성 |
| users | ○ (아이디·비밀번호) | ○ | ○ | 추가하면 **권한 user**(client page 계정). 수정은 아이디·비밀번호(비밀번호는 입력했을 때만 바뀜). **수퍼 계정 admin**은 비밀번호만 수정, 삭제 버튼 없음 — DB CHECK·트리거도 같은 규칙. 이름 `admin`은 다른 계정이 쓸 수 없음. 비밀번호는 DB의 pgcrypto `crypt(…, gen_salt('bf'))`로 해시 |
| settings | ○ | ○ | ○ | **설명(description)** 표시·입력·수정. 바꾸면 updated_by(로그인 사용자)·updated_at이 자동 기록. 정보 탭 패널에도 key 아래 설명 표시 |
| aps | ○ | ○ (SSID·메모·설치 x, y) | ○ | **지도에 속함** — 목록 창 위에서 지도를 고르고(처음은 현재 지도) 그 지도에 추가. 지도마다 SSID는 하나(다른 지도에는 같은 SSID 가능). 설치 위치 x · y는 함께 넣거나 둘 다 비움. **활성은 client·로봇이 그 AP 값을 보내 줄 때만** — 추가하면 비활성. **측정 AP**(3-3차): 목록 창 「측정 AP로」 — 지도마다 정확히 1개(`aps.is_target`, DB 부분 유일 인덱스), **활성 AP만** 고를 수 있음(비활성이면 버튼 꺼짐 · 400). 다른 AP를 고르면 이전 측정 AP는 풀림. 순찰은 이 AP의 표본만 모음. 순찰 중인 지도는 잠금(409) |
| maps | 업로드 | ○ (도크 접근 자세만) | ○ | 「지도 등록」: 이름 · 제어 PC 지도 경로 · 격자 크기 + **yaml · pgm** → resolution · origin 자동 추출(origin yaw = 0, negate = 0만 허용), pgm을 크기 그대로 PNG로 저장. **수정은 로봇별 도크 접근 자세**(도크 바로 앞에서 도크를 바라보는 x, y, yaw — 비우면 그 로봇 도크 없음)만, 바꾸면 현재 지도일 때 제어 PC에 바로 다시 보냄. 순찰(run) 진행 중에는 그 지도의 도크 수정 · 「현재 지도로」 다른 지도 고르기 잠금(409). **삭제하면 그 지도의 격자 칸 · AP · 설비도 함께 지움** — 순찰 · 측정 · 판정 기록이 있으면 「사용 중」으로 거부 |
| grid_cells | ✕ | ✕ | 지도 단위 | 지도를 **「현재 지도로」 선택할 때 그 지도만** 격자가 없으면 생성(섹터도 이때 정함). 목록 창에서 지도별 「격자 다시 생성」·「격자 삭제」 — **그 지도로 순찰(run)이 진행 중이면 409로 거부** |
| facilities | ○ | ○ | ○ | **지도에 속함** — AP와 같이 목록 위 지도에 추가. 설비 ID는 지도 안에서 유일(다른 지도에는 같은 ID 가능). 관측 자세는 x · y · yaw 세 칸 → JSON. 순찰 중인 지도는 잠금(409) |

- **삭제 규칙** — 다른 표가 참조하는 행은 DB 외래키 때문에 지워지지 않습니다. 이때 「사용 중 — 비활성(is_active = false)으로 바꾸세요」라고 안내합니다. 지도는 그 지도의 격자까지 함께 지우되, 설비·측정 기록이 참조하면 지우지 않습니다.
- **격자 생성 규칙** — 지도의 **흰 부분(빈 공간)이 조금이라도 걸치는 칸은 모두** 만듭니다. 픽셀 occupancy = (255 − 값)/255가 0.25 미만이면 흰 부분이고, 값 205(map_saver의 미탐색, 회색)는 뺍니다. 벽 여유(`settings.wall_margin_m`)로 칸을 빼지 않으므로 벽 가까이 좁게 남은 흰 띠에도 칸이 생깁니다. 칸 번호 기준점은 지도 원점입니다. 이전 규칙으로 만든 격자는 목록 창의 「격자 다시 생성」으로 바꿉니다.
- **섹터 규칙(3-2, 임시)** — 칸을 ix 순으로 세어 **칸 수가 절반에 가장 가까운 열**에서 나눕니다. 그 열까지(왼쪽)가 섹터 1, 오른쪽이 섹터 2입니다(같은 차이면 왼쪽 열, 열이 하나면 모두 섹터 1). `server_page.split_sectors()` 하나만 바꾸면 다른 분할 규칙으로 바꿔 끼울 수 있습니다. DB 업그레이드 구간과 더미 생성도 같은 규칙입니다. 설비의 섹터는 view_pose가 속한 칸의 섹터입니다(저장하지 않음).
- **섹터 ↔ 로봇** — 기본 배정은 섹터 1 = robot3, 섹터 2 = robot4(`DEFAULT_SECTOR_ROBOTS`)입니다. 순찰 시작 때 바꿀 수 있고, 활성인 로봇의 섹터만 그 회차에 넣어 `runs.sector_robots`에 저장합니다(3-3차).
- **활성 여부(robots · aps)** — client·로봇이 보내 준 값으로만 정합니다. robots는 제어 PC 연결(hello) · 끊김, aps는 client가 최근 5초 안에 받은 SSID(status의 `ssids`, 3-3차 — 측정 AP가 아니어도 받은 SSID는 모두) — 끊기면 `status_timeout_s` 뒤 비활성. 바뀌면 정보 탭이 `/ws/ui`로 바로 다시 읽습니다. **server_page를 시작할 때 모두 비활성으로 맞춥니다.**

## 4. DB에 대해

- scv_db_3차 스키마를 씁니다(settings.description, users 권한 admin | user, grid_cells.sector, runs.sector_robots · status partial, maps.dock_poses, aps · facilities가 지도에 속함, inspections.map_id, aps.is_target 측정 AP). 이전 판 DB는 새 `01_schema.sql` → `02_seed.sql`을 다시 실행하면 맞춰집니다.
- 트리거가 거부한 경우(예: admin 삭제 시도)는 DB 메시지를 그대로 400으로 보여 줍니다.


## 5. 제어 PC — client page (2_C-2차)

### 5.1 실행

```bash
source /opt/ros/jazzy/setup.bash
cd scv_app
python3 -m venv --system-site-packages .venv && source .venv/bin/activate   # rclpy를 쓰려면 --system-site-packages
pip install -r requirements-client.txt
python3 client_page.py --server http://<관제PC>:8000     # http://<제어PC>:8080 → /client/login
```

| 옵션 | 환경변수 | 기본값 | 설명 |
|---|---|---|---|
| `--robot-id` | `SCV_ROBOT_ID` | `robot3` | 이 제어 PC와 짝인 로봇 |
| `--server` | `SCV_SERVER_URL` | `http://127.0.0.1:8000` | 관제 PC server page 주소 |
| `--camera-topic` | `SCV_CAMERA_TOPIC` | `/{robot_id}/oakd/rgb/image_raw/compressed` | 카메라 탭에서 보여 줄 토픽 — 기본은 짝인 로봇(robot3)의 카메라 |
| `--host` / `--port` | `SCV_CLIENT_HOST` / `SCV_CLIENT_PORT` | `0.0.0.0` / `8080` | |
| `--check-s` | `SCV_ACTIVE_CHECK_S` | `2` | 로봇이 비활성일 때 토픽 목록 확인 주기(초) |


### 5.2 동작

| 순서 | 내용 |
|---|---|
| 로그인 | 첫 화면은 `/client/login`. 아이디·비밀번호를 관제 PC `POST /api/client/login`으로 확인합니다(제어 PC는 DB에 접속하지 않음). DB users의 **user**와 **admin** 모두 로그인할 수 있습니다. 로그인 전에는 탭이 비활성 |
| 로봇 활성 확인 | 로그인하면 바로 ROS 토픽 목록(`get_topic_names_and_types`)을 보고, `/robot3/…` 토픽 중 **발행자가 있는 토픽**이 하나라도 있으면 활성입니다(이 노드의 구독만 있는 토픽은 세지 않음). 비활성이면 `--check-s`마다 다시 확인하고, 활성이 되면 확인을 멈춥니다 |
| 관제 PC에 알림 | 활성이 되면 관제 PC `WebSocket /ws/agent`에 접속해 **hello**(`robot_id`, `agent_version`, `run_id`, `active_task_id`, `outbox_pending` — 구현방안 6.2절)를 **한 번** 보냅니다. 연결은 계속 유지하며, 끊기면 다시 접속해 hello를 보냅니다(구현방안 6.3절) |
| 로봇 정보 탭 | `/robot3/battery_state` · `dock_status` · `odom` · `ip`를 구독(best effort)해 1초마다 표시. 5초 넘게 값이 없으면 「끊김」 |
| 로봇이 꺼지면 | 활성인 동안 위 네 토픽의 값이 **5초 넘게 하나도 오지 않으면** 로봇이 꺼진 것으로 보고 비활성으로 바꿉니다. 관제 PC WebSocket 연결을 닫아 **관제 PC의 robots.is_active도 false**가 되고, 토픽 목록 확인을 다시 시작합니다. 다시 활성이 되려면 토픽 목록의 발행자와 **꺼진 뒤 새로 받은 값**이 함께 있어야 합니다(DDS 목록에 잠시 남는 옛 발행자로 다시 켜지지 않게). 활성이 되면 다시 접속해 hello를 보냅니다 |
| 카메라 탭 | 카메라 토픽(`sensor_msgs/CompressedImage`)을 **탭을 보는 동안 · 순찰 중에만** 구독하고(3-3차 — 측정 · 정밀측정 · 탐지 · 복귀 모두), JPEG를 그대로 MJPEG(`/api/camera/stream`)으로 화면에 보냅니다. 다른 탭으로 가거나 순찰이 끝나면 구독 해제. 아래 지도 · 순찰 정보는 10.5절 |

### 5.3 관제 PC 쪽 처리 (server_page)

- `POST /api/client/login` — users에서 아이디·비밀번호 확인(admin · user 모두). 결과만 돌려주고 server page 세션은 만들지 않습니다.
- `WebSocket /ws/agent` — 첫 메시지가 hello가 아니면 닫습니다. hello의 `robot_id`가 robots에 있으면 **is_active = true**로 바꾸고 welcome을 보냅니다. 등록되지 않은 로봇이면 error 후 닫습니다. **연결이 끊기면 is_active = false**(값이 오지 않으면 비활성 규칙). 같은 로봇이 다시 접속하면 이전 연결을 닫습니다.
- server page를 다시 시작하면 시작할 때 모두 비활성이 되고, client가 다시 접속해 hello를 보내면 활성으로 돌아옵니다.

## 6. 가상 로봇 — robot3_virtual.py

```bash
source /opt/ros/jazzy/setup.bash
python3 robot3_virtual.py                                  # = --world virtual_world/testbed/world.yaml (이동 0.3 m/s, RSSI 1 Hz)
python3 robot3_virtual.py --speed 1.5 --rssi-hz 6          # 빠른 시험
python3 robot3_virtual.py --robot-id robot4                # /robot4/… (섹터 2)
python3 robot3_virtual.py --world 다른세계/world.yaml       # 다른 가상 세계
python3 robot3_virtual.py --no-world                       # 가상 세계 없이 로봇만
python3 robot3_virtual.py --camera-hz 5            # 카메라 주기 (0이면 카메라 끔)
```

구현방안 4.1절의 로봇 통신과 제어 PC 표준 노드(AMCL · Nav2)의 이름을 흉내 냅니다. client가 쓰는 것만 구현합니다. **[가상 세계]** 표시는 `--no-world`가 아닐 때만 있습니다.

| 이름 (`/robot3/…`) | 타입 | 내용 |
|---|---|---|
| `battery_state` | sensor_msgs/BatteryState · 1 Hz | 도킹 중 충전, 주행 중 방전 |
| `dock_status` | irobot_create_msgs/DockStatus · 1 Hz | 도킹 여부 |
| `odom` | nav_msgs/Odometry · 10 Hz | 시작 자세(도크) 기준, 속도 포함 |
| `amcl_pose` | geometry_msgs/PoseWithCovarianceStamped · 2 Hz | 지도 좌표(σ 5 cm) — initialpose를 받은 뒤부터 |
| `ip` | std_msgs/String · 1 Hz | 이 PC의 IP |
| `oakd/rgb/image_raw/compressed` | sensor_msgs/CompressedImage · 10 Hz | 640 × 360 JPEG. [가상 세계] 판정 사진(`photo_hold_s` 동안) → 순찰 영상(움직일 때 다음 장면, 멈추면 그 장면, 위 띠에 로봇 · 상태 · 시각). 가상 세계가 없거나 영상 · OpenCV가 없으면 합성 화면 |
| `rssi` [가상 세계] | std_msgs/String (JSON) · 1 Hz | `{stamp, scans: [{ssid, rssi_dbm}]}` — truth `ap_models` · 지도 벽 · `holes`로 계산 (rssi_scanner 대신, initialpose 뒤부터) |
| `virtual/detect_request` → `virtual/detect_result` [가상 세계, 시험 전용] | std_msgs/String (JSON) | 가상 판정 — 설비 자리가 `view_range_m` 안이면 truth `facilities` 정답으로 검출 결과(YOLO 대신) + 판정 사진을 카메라로 |
| 구독 `initialpose` | geometry_msgs/PoseWithCovarianceStamped | 시작 위치 = 도크 |
| 액션 `navigate_through_poses` (3-3) | nav2_msgs/NavigateThroughPoses | 점을 차례로 회전 후 직선 이동(장애물 무시) · 남은 점 수 피드백 · 취소 |
| 액션 `undock` · `dock` (3-3) | irobot_create_msgs/Undock · Dock | 도크에서 25 cm 뒤로 / 도크 접근 자세 1 m 안이면 도킹 |

## 7. client에 로봇 값이 안 뜰 때

client_page.py를 실행하는 **같은 터미널**에서 `python3 ros_check.py`를 실행하면 환경 변수, `/robot3/…` 토픽 목록(발행자 수), 5초 동안 받은 값 개수와 판단을 보여 줍니다.

| 결과 | 원인 | 조치 |
|---|---|---|
| 발행자가 안 보임 | 로봇과 client의 ROS 환경이 다름 | 두 터미널의 `ROS_DOMAIN_ID` · `ROS_LOCALHOST_ONLY`/`ROS_AUTOMATIC_DISCOVERY_RANGE` · `ROS_DISCOVERY_SERVER` · `RMW_IMPLEMENTATION`을 맞춤 |
| 토픽은 보이는데 값이 0개 | 발견은 되지만 데이터 전달이 막힘(같은 PC의 공유 메모리 등) | 양쪽에 `export FASTDDS_BUILTIN_TRANSPORTS=UDPv4` 후 다시 실행 |
| 값이 옴 | ROS 통신 정상 | client 터미널 로그 확인 — 토픽 목록 확인 결과, 「첫 값 받음 …」, 콜백 오류(`[ros] … 오류`)가 찍힘 |

## 8. 주의사항 — Discovery Server와 가상 로봇 시험

TurtleBot4를 쓰는 PC는 `~/.bashrc`에 Fast DDS **Discovery Server** 설정이 들어 있습니다(예: `ROS_DISCOVERY_SERVER=;;;;;192.168.109.105:11811;`, `ROS_SUPER_CLIENT=True`). 이 설정이 켜져 있으면 노드는 같은 PC에 있어도 **서로 직접 찾지 않고 그 서버를 거쳐서만** 찾습니다. 서버는 실제 로봇의 Raspberry Pi에서 돌아가므로 그 로봇이 꺼져 있거나 주소에 닿지 않으면, robot3_virtual과 client_page가 정상으로 떠도 서로 보이지 않습니다. client 상태줄에는 「로봇 확인 중 — 아직 /robot3/… 발행자 없음」이 계속 보이고 로봇 정보·영상이 오지 않습니다.

### 8.1 가상 로봇으로 시험할 때 — Discovery Server 끄기

로봇(robot3_virtual) 터미널과 client(client_page) 터미널 **양쪽 모두**에서 실행한 뒤 각각 다시 실행합니다.

```bash
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
ros2 daemon stop        # ros2 topic list가 쓰는 데몬이 예전 설정을 기억하지 않게
```

`ROS_DOMAIN_ID`는 양쪽이 같으면 그대로 둡니다. `unset`은 **그 터미널에서만** 설정을 지우며 `~/.bashrc`는 바뀌지 않습니다.

Discovery Server를 그대로 쓰고 싶으면 이 PC에 서버를 직접 띄우는 방법도 있습니다.

```bash
fastdds discovery -i 0 -l 127.0.0.1 -p 11811        # 별도 터미널
export ROS_DISCOVERY_SERVER=127.0.0.1:11811          # 로봇·client 터미널 양쪽
```

### 8.2 시험이 끝난 뒤 — Discovery Server 다시 켜기

셋 중 하나로 되돌린 뒤, 실행 중이던 로봇·client 프로그램도 다시 실행합니다.

| 방법 | 명령 | 비고 |
|---|---|---|
| A. 새 터미널 | `unset`한 터미널을 닫고 새 터미널을 연 뒤 `ros2 daemon stop` | 가장 간단. `~/.bashrc`가 다시 읽힘 |
| B. 같은 터미널에서 다시 읽기 | `source ~/.bashrc` → `ros2 daemon stop` | |
| C. 값을 직접 넣기 | `export ROS_DISCOVERY_SERVER=";;;;;192.168.109.105:11811;"` · `export ROS_SUPER_CLIENT=True` → `ros2 daemon stop` | 값은 이 PC의 `~/.bashrc`에 있던 것. `;` 때문에 따옴표 필요 |

되돌아왔는지 확인:

```bash
printenv | grep -E 'ROS_DISCOVERY_SERVER|ROS_SUPER_CLIENT'   # 원래 값이 나오면 복구
ros2 topic list                                               # 실제 로봇이 켜져 있으면 그 로봇 토픽이 보임
```

### 8.3 공통

- 로봇 · client · `ros2` 명령을 쓰는 **모든 터미널을 같은 상태**(모두 끔 또는 모두 켬)로 맞춥니다. 한쪽만 바꾸면 서로 보이지 않습니다.
- 설정을 바꾼 뒤에는 `ros2 daemon stop`으로 데몬을 재시작합니다(다음 `ros2` 명령 때 새 설정으로 다시 켜짐).
- 실제 로봇과 함께 쓰는 최종 구성에서는 `~/.bashrc`의 Discovery Server 설정을 그대로 씁니다.
- 원인이 분명하지 않으면 client 터미널 환경에서 `python3 ros_check.py`로 확인합니다(7장).

## 9. 관제 PC 메모리 상태 · 로봇 상태 (3-1 도입, 3-2에서 Redis → 메모리)

3-1차에는 Redis에 두었던 실시간 상태를 3-2차부터 server_page 프로세스 메모리에 둡니다. client가 로봇 토픽을 받아 1 Hz status로 줄여 보내므로 관제 PC가 처리할 양이 적고, 메모리에만 있는 기록은 만들지 않는다는 원칙(영구 기록은 모두 PostgreSQL)을 지키면 별도 저장소가 필요 없기 때문입니다.

| 항목 | 형식 | 쓰는 곳 | 내용 | server 재시작 때 |
|---|---|---|---|---|
| 로그인 세션 | token → 사전, 8 h | server 로그인 | user_id · username · role · created_at. 요청마다 8 h 연장, 로그아웃·계정 삭제 때 지움, 계정 이름 변경 때 함께 바꿈 | 다시 로그인 |
| 로봇 최신 상태 | robot_id → 사전 + 마지막 수신 시각 | `/ws/agent` status | ts · run_id · task_id · nav_state · progress · x · y · yaw · battery_pct · is_docked · estop · received_at | 다음 status(1초 안)로 다시 채움 |

- **status** — client의 mission_executor가 1초마다 만들고(위치 = `{ns}/amcl_pose`, 배터리·도킹 = 로봇 토픽) scv_agent가 `/ws/agent`로 보냅니다(구현방안 6.2절 status). 관제 PC는 메모리 최신값을 덮어쓰고 수신 시각을 갱신하며, 위치·배터리가 있으면 `robot_status_log`에도 남깁니다(같은 (robot_id, ts)는 무시).
- **E3 응답 없음** — 1초마다 도는 점검(watchdog)이 마지막 수신 뒤 `settings.status_timeout_s`(30 s)가 지난 로봇을 찾아 `robot_events`에 `E3 · source = control · level = warn`을 남기고 메모리 상태를 지웁니다. 같은 점검이 만료된 로그인 세션도 정리합니다.
- **3-3차 추가** — 진행 중 run(CURRENT_RUN, 시작 때 runs에서 복원) · 실시간 칸 값(LIVE_CELLS, 처음 볼 때 net_samples에서 다시 집계) · 최근 이벤트 1,000건(RECENT_EVENTS, 시작 때 robot_events에서) · AP 활성(AP_SEEN — status의 `ssids`, 끊기면 1초 점검이 비활성). 순찰 진행률은 tasks를 그대로 조회합니다(별도 메모리 없음). e-stop 상태(3-4 정지)도 같은 방식으로 둡니다.
- **재시작 때 E3 보완(3-3차)** — server_page가 시작할 때 진행 중 run에서 PENDING · ACTIVE task가 남은 로봇을 「방금 받은 것」으로 메모리에 넣습니다. `status_timeout_s` 안에 다시 연결하면 그대로 이어지고, 연결하지 않으면 평소처럼 E3 → task 실패 → run 종료로 정리됩니다.
- **모니터 실시간** — 브라우저가 `/ws/ui`에 접속하면 1초마다 로봇별 상태를 받습니다(로그인 세션 쿠키로 확인). 지도 위 마커(방향 표시 포함)와 「로봇 상태」 칸이 갱신되며, 상태는 정상 / 연결 끊김(만료 대기) / 응답 없음 / 비활성으로 나눕니다.
- **가상 로봇 3-1** — 도크(= 시작 위치)에 서서 충전합니다(battery_state 충전 중, dock_status 도킹). `{ns}/amcl_pose`(map 좌표)를 2 Hz로 내보내 실제 구성의 제어 PC AMCL을 대신합니다. 이동(Nav2 · dock · undock)은 3-3차입니다. 3-2차부터 시작 위치는 관제 PC의 도크 접근 자세(→ client initialpose)로 정하고, 3-3차 DOCK task는 waypoints = [도크 접근 자세]로 그 자리까지 이동한 뒤 dock 액션을 부릅니다(robot4 가상 로봇 · 두 대 시험 환경은 이후 차수).

## 10. 3-3차 — 작업 (측정 · 정밀측정 · 탐지)

### 10.1 흐름 (로봇마다, 서로 기다리지 않음)

| 단계 | 만드는 때 (inspection_manager) | 제어 PC (client) | 남는 기록 |
|---|---|---|---|
| 측정 SURVEY_SEGMENT | 순찰 시작 — 그 로봇 섹터의 임시 경로를 `segment_points`(20)점씩 | 도킹 중이면 undock → Nav2 주행. 그동안 `{ns}/rssi` + AMCL 위치 → 자기 섹터 칸 표본만(밖이면 버림), 1초마다 묶어 POST samples. 구간이 끝나면 새 후보 칸 → CANDIDATE 이벤트 | net_samples · robot_events(UNDOCK · CANDIDATE) · candidates(OPEN) |
| 정밀측정 REINSPECTION | 그 로봇의 측정이 모두 끝났을 때 — 열린 후보마다(같은 로봇) | 후보 지점(그 칸에서 마지막으로 잰 위치)으로 이동 → `precise_duration_s` 정지 측정 → POST precise-measures → 측정 AP 중앙값 < 기준이면 CONFIRMED, 아니면 CLEARED | precise_measures · candidates(REINSPECTING → CLOSED) · findings(CONFIRMED) |
| 탐지 FACILITY_CHECK | 정밀측정이 끝났을 때(후보가 없으면 바로) — 섹터 안 활성 설비마다(관측 자세 칸의 지그재그 순서) | 관측 자세로 이동 → **판정 1회**(다시 판정 · 설비 후보 없음). 가림 · 정면 아님이면 자세 보정 재관측 `max_pose_retry`(3)번까지 → 넘으면 OCCLUDED · 미확인 + E6. 사진 1장 | inspections · media · robot_events(E6) |
| 복귀 DOCK | 탐지가 끝났을 때, 또는 task가 실패했을 때 | 도크 접근 자세로 이동 → dock | robot_events(DOCK) |

- task는 로봇마다 하나씩(ACTIVE 하나, task_id 순서) WS `task_assign`으로 보냅니다. client는 `task_accepted` · `task_progress`(1초마다)를 WS로, 결과는 아웃박스로 `POST /api/agent/tasks/{id}/result`를 보냅니다.
- **실패** — 막힘(Nav2 실패 → E1, BLOCKED) · 배터리 `battery_return_pct` 이하(E2, BATTERY_LOW) · 지도 파일 없음(MAP_MISMATCH → `task_rejected`) · 응답 없음(E3, NO_RESPONSE) → `scv_fail_task`(그 로봇의 남은 task만 취소) → 연결된 로봇이면 복귀 DOCK. 남은 task가 없으면 `scv_close_run` → done · partial · aborted.
- **순찰 종류** — 순찰(`run_type = patrol`) 하나입니다(3-3 보완 — 순찰 시작 창에 종류 선택 없음, server는 다른 값 거부). task는 측정 → (후보가 있으면) 정밀측정 → 탐지 → 복귀. DB CHECK는 이전 값(remeasure · demo)도 허용한 채 두었고, 이력 탭에는 그 값이 그대로 보입니다.
- **기준선 재료 표시** — run이 done으로 끝날 때 순찰(`run_type = patrol`)이고 RSSI 확정 이상(네트워크 후보에서 나온 findings)이 없으면 `runs.is_baseline_ok = true`. partial · aborted는 false입니다(순찰 종류는 3-3 보완부터 순찰 patrol 하나 — DB에 남은 이전 demo · remeasure 회차도 false). 이 값으로 기준선을 만드는 일은 보고서 차수(3-5)입니다.
- **시작** — 모니터 「순찰 시작」 → 섹터 배정 창. 막는 조건(전체): 현재 지도 없음 · 격자 없음 · 측정 AP 없음 · 측정 AP 비활성. 로봇마다: 비정상(제어 PC 연결 · status 없음) · 도킹 아님 · 도크 접근 자세 없음 — 배정 창 · 로봇 상태 칸에 로봇별 이유가 나오고, 조건을 만족하는 로봇만 고를 수 있습니다(서버도 다시 확인해 400). 시작하면 로봇마다 `run_config`(격자 · 자기 섹터 칸 · `ssids` = [측정 AP] · 도크 · 섹터 설비)를 1회 보냅니다.
- **끝** — run이 done · partial · aborted가 되면 그 run의 로봇에 WS `run_end { run_id, status }` → client는 run_config를 지우고 카메라 · 지도 겹침을 내립니다.
- **중단** — 모니터 「순찰 중단」 → `scv_abort_run`(ADMIN_STOP) → WS `run_cancel` → client는 하던 일을 멈추고 도크로 복귀(task 없이).
- **재접속** — client가 다시 연결하면 관제 PC가 진행 중 run의 `run_config`를 다시 보내고, ACTIVE task가 client와 다르면 다시 보내거나(`task_assign`) 취소합니다(`task_cancel`, 진행 중 task가 없으면 도크 복귀).

### 10.2 임시 규칙 (함수 하나씩 — 4차에 교체)

| 규칙 | 위치 | 지금 |
|---|---|---|
| 섹터 분할 | server_page `split_sectors` | ix 순 칸 수 절반 (3-2차와 같음) |
| 측정 경로 | server_page `plan_survey` | 섹터 칸을 ix 열마다 iy 방향을 번갈아(지그재그). 칸 안에서 `wall_margin_m` 벽 여유를 만족하는 점 중 칸 중심에 가장 가까운 점, 없으면 그 칸을 건너뛰고 첫 task의 reason에 `SKIPPED_CELLS:n`. `path_spacing_m`은 쓰지 않음 |
| 네트워크 후보 | client `candidate_rule` | 같은 칸 표본(pose_ok만) `min_samples`개 이상인 AP 중 **가장 센 AP의 중앙값** < `rssi_suspect_dbm` → 후보(같은 run · 칸은 한 번). 3-3차는 측정 AP 하나의 표본만 모으므로 = 측정 AP 중앙값(함수는 여러 AP도 받도록 둠) |
| 정밀측정 결론 | client MissionExecutor | 정밀측정 측정 AP 중앙값 < `rssi_suspect_dbm` → CONFIRMED, 아니면 CLEARED |
| 설비 판정 | client `judge` | 가림 · 정면 아님 → OCCLUDED(미확인) · 안 보임 → NOT_DETECTED · 위치 차이 > 0.5 m → POSITION_ANOMALY · 상태 · 압력계 이상 → ABNORMAL · 그 밖 NORMAL (가상 판정 — 실제 YOLO는 이후 차수) |
| 다른 섹터 표본 | client SurveyBuffer | 자기 섹터 칸이 아니면 버림(개수만 화면에) |

### 10.3 제어 PC 아웃박스 (client)

- 폴더: `--outbox`(환경 변수 `SCV_OUTBOX_DIR`), 기본 `outbox_{robot_id}/`. 파일 하나 = 요청 하나(번호 순), 사진은 `files/`, run별 표본 seq · 이벤트 번호는 `state.dat`.
- 관제 PC가 받으면(2xx) 지우고, 연결 실패 · 5xx · 409(사진이 판정보다 먼저 온 경우)는 잠시 뒤 다시, 그 밖의 4xx는 `failed/`로 옮기고 다음으로 넘어갑니다.
- 관제 PC는 seq · event_uid · inspection_uid · (task_id, ap_id) · object_key로 중복을 무시하므로 다시 보내도 됩니다. client를 다시 켜면 남은 기록부터 이어서 보내고, 표본 seq는 `GET /api/agent/last-seq`와 `state.dat` 중 큰 값 다음부터 씁니다.
- client page 로봇 정보 탭의 「작업 (관제 PC task)」 칸에 실행 중 task · 단계 · 진행률 · 대기 task · 표본 수 · 아웃박스(보낼 것 · 보냄)가 나옵니다.

### 10.4 사진 · 파일

판정 사진은 관제 PC `SCV_MEDIA_DIR/inspections/YYYY/MM/DD/r{run}/{설비}_{uid}_{n}.jpg`에 저장하고 media 행에 경로 · 해시 · 크기를 남깁니다. 모니터(진행 중) · 이력 탭 설비 탐지 표와 client 카메라 탭에서 바로 볼 수 있습니다.

### 10.5 client 카메라 탭 (3-3차)

| 부분 | 내용 |
|---|---|
| 영상 | 순찰 중(run_config를 받은 뒤 ~ run_end · run_cancel 복귀까지)에만. 순찰이 아니면 「순찰 중에만 영상을 보여 줍니다」 |
| 지도 | 제어 PC의 지도 파일(`robot_map_path`의 yaml + pgm, `/api/map` · `/api/map/image`)을 읽어 그림 — 관제 PC에서 받지 않음. 위에 **담당 로봇만** 위치 · 방향(1초) · 실행 중 task 경로. 순찰 중에는 run_config로 1회 받은 자기 섹터 칸(파랑) · 섹터 설비 · 측정 AP · 도크를 순찰이 끝날 때까지 겹침 |
| 아래 표 | 로봇 정보 탭에 없는 것만: 이번 순찰 **후보 → 정밀측정 결론**(칸 · 점수 · CONFIRMED · CLEARED) · **설비 판정**(판정 · 이유 · 사진 — `/api/patrol/photo/{uid}`, 최근 12장). 새 순찰이면 비움 |

- 진행 상황은 `GET /api/patrol`(순찰 여부 · run · 단계 · 진행률 · 위치 · task 경로 · 기록 · 지도 키)로 1초마다 읽습니다.
- 지도 파일이 없으면 지도 칸에 「제어 PC에 지도 파일이 없거나 읽지 못했습니다 — 경로」가 나옵니다(순찰 task도 MAP_MISMATCH로 거부됨). 지도 경로는 관제 PC config · run_config의 `robot_map_path`입니다.
