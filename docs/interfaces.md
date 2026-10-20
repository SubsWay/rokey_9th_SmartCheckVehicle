# SCV 인터페이스 — 통신 계약 (v0.3, 10/15 실기 통합 기준)


> **v0.3 (2026-10-15 최종 통합 `temp/integration-1014`, 실기 두 대 순찰로 확인한 계약)** — 아래 표가 지금 코드와 같습니다. 옛 버전 기록(v0.2.x)은 경위로만 남깁니다.
> - IF-05 설비 점검은 **서비스**이고 서버는 비전 노드(`scv_detection_manager`, 노드 이름 `detection_alert`)입니다. 옛 action 정의는 삭제했습니다.
> - RSSI 재측정(IF-04 · IF-11 결과)에 `rssi_mean · n_total · n_missed · duration_s · measurement_valid · measurement_reason` 추가 (RSSI 팀 수정). **판정은 1차 · 재측정 모두 검출 RSSI 중앙값 < −72 dBm 또는 미검출 비율 ≥ 0.5, 전체 표본 최소 3개** (10/15 PM 결정 — RSSI 분기의 평균 판정은 적용하지 않음). 정지 측정 10~15초(기본 15초). **재측정은 1차 측정 경로에서 미검출된 칸만** 갑니다.
> - 미검출 표본도 DB 에 남깁니다 (`net_samples.detected=false`, `rssi_dbm` NULL). 평균 · 히트맵은 검출 값만.
> - 새 토픽 IF-20 `{ns}/camera/preview` (블랙박스), 새 서비스 `/inspection/abort` (순찰 종료).
> - `scv_msgs` 를 바꾸면 관제 PC · 로봇 PC 를 **함께** 다시 빌드해야 서로 통신됩니다.

2026-10-10 · 작성 도윤 (PM) · **팀 검토 전 초안입니다. 피드백을 받아 고칩니다.**

v0.2: 회차 전달·근거 사진·후보 발견 로봇·재개/재배정·각도 rad·scv_survey 패키지
v0.2.1: 로봇 이름공간 `/robot4`·`/robot3`, 모든 기기 turtle08, 빌드 전 가상환경 끄기, 자세 보정 "뒤로" 처리 제안
v0.2.2: 고정 웹캠은 쓰지 않습니다 (10/10 PM 결정) — `webcam_node` 없음, CAM 탭은 로봇 카메라(OAK-D)만
v0.2.4: 설정 파일 형식(`facilities.yaml`, `zones.yaml`)과 기기별 launch 뼈대 추가
v0.2.5: 시나리오 변경 (10/11 PM 결정) — 로봇 A 는 통신 측정·재측정, 로봇 B 는 물리 인프라 점검. keepout 마스크(IF-10)는 로봇 B 에만 발행하고, REINSPECTION 은 음영 후보를 로봇 A 가 그 구역 안에서 다시 재는 작업입니다. 순서는 A → B → A 이고 로봇 A 는 RSSI 만, 로봇 B 는 물리 점검만 합니다. 음영 구역 안에는 설비를 두지 않습니다. 메시지 정의(`scv_msgs`)는 그대로입니다
v0.2.6: 기준 지도 확정 (10/11 PM 결정) — 주행 팀의 `testbed_v1`, `scv_bringup/maps/` 에 한 벌. 아래 「기준 지도」 절. 측정 노드는 로봇 A 쪽에만, 판정 노드는 로봇 B 쪽에만 올립니다
v0.2.7: 시나리오 변경 (10/12 PM 결정, v0.2.5·v0.2.6 의 역할 분리를 대신함) — 두 로봇이 동시에 주행하고 섹터 2개를 하나씩 맡아 같은 일(측정 → 재측정 → 점검)을 합니다. 측정·판정 노드는 두 로봇 쪽에 모두 올립니다. keepout 마스크(IF-10)는 지도 전체 크기 한 장을 **두 로봇에** 발행하되, 측정·재측정 동안은 빈 마스크이고 재측정 뒤 남은 칸으로 점검 단계 전에 발행합니다. REINSPECTION 은 그 섹터의 로봇이 음영 칸 안에서 다시 재는 작업입니다. 섹터와 맡는 로봇은 `zones.yaml` 의 `robot:`. DOCK 작업은 inspection_manager 가 만듭니다. 랙 적재물 돌출(`KIND_RACK=4`) 추가. 관제 PC 안의 DB·웹은 PostgreSQL + Redis + 웹 API (관제 팀 `scv_database`), 로봇 PC ↔ 관제 PC 는 이 문서의 ROS 인터페이스 그대로입니다.
v0.2.8: keepout 마스크 발행 규칙 (10/12 오후 PM 결정) — 발행자는 시작부터 **빈 마스크**를 `/robot4/keepout_mask`·`/robot3/keepout_mask` 에 transient_local 로 발행합니다(지도 `testbed_v1` 과 같은 크기·원점·해상도). 발행자가 없으면 Nav2 의 `keepout_layer` 가 준비되지 않아 경로 계획이 실패하므로, 주행은 이 발행을 확인한 뒤 `keepout_layer` 를 켭니다. 두 로봇은 서로 기다리지 않고, 마스크는 **로봇마다 그 로봇이 점검 단계에 들어갈 때** 그 로봇 토픽으로 보냅니다 — 내용은 재측정이 끝난 섹터의 남은 음영·미검출·미측정 칸(Dock 반경 0.6 m 제외). 측정·재측정 중인 로봇은 빈 마스크 그대로입니다. 재측정 뒤에도 남은 칸은 관제에 알림을 띄워 사람이 확인합니다.
v0.2.9: 맵은 DB 로, IF-10 은 관제가 (10/12 회의 뒤 PM 결정) — network_map_engine(RSSI)은 1차 측정의 칸 맵(칸 중심 좌표 · 중앙값 · 상태 미측정/미검출/음영/정상)을 **관제 API 로 DB 에 쓰고**, 재측정 대상 칸을 후보(IF-15)로 냅니다. 재측정 결과(IF-04 → IF-11 result)는 inspection_manager 가 DB 의 그 칸에 적습니다. **IF-10 은 inspection_manager(관제)가 냅니다** — 시작할 때 두 로봇에 빈 마스크, 로봇마다 점검 작업을 보내기 전에 DB 맵의 남은 칸으로 그 로봇 토픽에 발행합니다. 노드 사이에 요청을 주고받지 않고 DB 를 봅니다. DB 쓰기는 모두 관제 API 로 합니다. 칸 0.5 m · 칸 값은 검출된 RSSI 의 중앙값 · 검출 0개이거나 미검출 비율 ≥ 0.5 이면 미검출 · 중앙값 < −72 dBm 이면 음영 (`SHADOW_DBM`, −75 dBm 은 지도 색 기준) · 회피 마스크는 샘플 2개 이상 칸만 (`min_samples_for_mask`) (`scv_survey` 코드값, 9d3a4cc). 미측정은 갈 수 있는 칸(섹터 안, 벽·장애물에서 0.45 m 밖) 중 샘플이 없는 칸이고, 재측정 대상입니다(로직에만, 시연에는 없음). 왕복 경로 간격(`spacing`)은 0.5 m 입니다.
v0.2.10: `scv_msgs` 필드 셋·하나 추가 (10/12 저녁 PM 결정) — `ExecuteTask` 결과에 `rssi_median`·`rssi_iqr`·`n_samples` (REINSPECTION 만, PreciseMeasure 결과를 그대로 옮김 · 다른 종류는 0). inspection_manager 가 이 값으로 DB 의 그 칸을 고칩니다 (`reason` 문자열을 읽지 않음). `InspectFacility` 목표에 `run_id` — mission_executor 가 받은 작업의 `run_id` 를 그대로 넣고 detection_alert 가 `InspectionResult.run_id` 로 씁니다. 기존 필드는 그대로라 지금 코드는 다시 빌드만 하면 됩니다 (`colcon build --packages-select scv_msgs`).
v0.2.11: RSSI 측정은 로봇 PC 에서 (10/12 저녁 PM 결정, 강사 조언) — 로봇 PC(MSI)를 두 로봇 위에 하나씩 얹고 `rssi_scanner` 를 그 PC 에서 돌립니다. RPi 에는 드라이버만 남습니다. IF-02·IF-04 는 같은 로봇 PC 안의 토픽·액션이 되고, `rssi_scanner` 는 `robot_pc.launch.py` 로 띄웁니다. 스캔은 로봇 PC 내장 무선랜으로 하고, 스캔 중 turtle08 통신이 흔들리면 USB 무선랜으로 나눕니다 (D2).
v0.2.12: IF-05 설비 점검을 액션에서 **서비스**로 (10/14 PM 결정, 비전 팀 구현 기준) — `{ns}/inspect_facility` · `scv_msgs/srv/InspectFacility`. 판정 노드는 OAK-D 영상을 처음부터 계속 받고, 요청이 오면 그때 판정합니다. **비전용 자세 보정 주행은 없습니다** — 관측 자세에서 한 번 판정하고, 판정할 수 없으면 UNCONFIRMED 로 돌려줍니다 (피드백·보정값 필드를 뺌). 그래서 아래 「자세 보정 "뒤로"」 제안은 폐기합니다. 옛 `action/InspectFacility.action` 은 T2-6 통합 때 mission_executor 를 바꾼 뒤 지웁니다.
v0.2.3: 미검출을 RSSI −100 으로 적지 않습니다 — `NetRaw`·`NetSample` 에 `detected` 추가, 미검출 샘플의 `rssi_dbm` 은 쓰지 않음 (10/10 PM·RSSI 팀)

팀 사이에 오가는 토픽·서비스·액션과 그 타입을 한곳에 모았습니다. 타입 정의는 `ros2_ws/src/scv_msgs/` 에 있습니다. 고칠 것이 있으면 그 약속의 **주인**에게 말하고, 주인이 `scv_msgs` 와 이 문서를 함께 고칩니다.

## 이름 규칙

- `{ns}` 는 로봇 이름공간입니다: 로봇 A = 4번 `/robot4`, 로봇 B = 3번 `/robot3`. `robot_id` 필드에는 앞의 `/` 없이 `robot4`, `robot3` 을 씁니다.
- 코드에 로봇 번호를 직접 쓰지 않습니다. 노드는 이름공간을 launch 인자(`robot_ns`)로 받아, 로봇이 바뀌어도 launch만 고치면 되게 합니다.
- 좌표는 저장 지도(`map`) 기준이고 단위는 m, 각도는 rad 입니다.

## 기준 지도

모든 팀이 같은 지도 한 벌을 씁니다: `ros2_ws/src/scv_bringup/maps/testbed_v1.yaml` · `testbed_v1.pgm` (주행 팀이 10/10 에 만든 지도, 10/11 PM 확정).

| 항목 | 값 |
| --- | --- |
| 해상도 | 0.05 m/px |
| origin | `[-0.150, -0.250, 0]` — 지도 그림 왼쪽 아래 모서리의 map 좌표 |
| map 원점 (0, 0) | 주행 공간 안쪽 왼쪽 아래 벽 모서리 |
| 크기 | 69 × 138 px (주행 공간 약 3.2 × 6.5 m) |

- 격자 지도·keepout 마스크·`facilities.yaml`·`zones.yaml` 의 좌표는 모두 이 지도의 `map` 좌표입니다.
- 지도는 바꾸지 않습니다. 바꿔야 하면 PM 에게 말하고, PM 이 이 폴더의 파일과 이 절을 함께 고칩니다.
- 각 팀 패키지 안의 지도 사본(`scv_nav/maps`, `scv_survey/maps`)은 기준이 아닙니다.
- `/inspection/...` 은 로봇과 무관한 관제 PC 쪽 이름입니다.

## 기기와 패키지

| 패키지 | 실행 위치 | 들어갈 노드 | 팀 |
| --- | --- | --- | --- |
| `scv_msgs` | 5대 전부 | 메시지·서비스·액션 | PM |
| `scv_survey` | 로봇 PC · 관제 PC | `rssi_scanner`, `survey_buffer`, `network_map_engine` | 통신 인프라·RSSI (A·B) |
| `scv_nav` | 로봇 PC | `mission_executor`, Nav2 설정 | 주행 |
| `scv_detection` | 로봇 PC | `detection_alert` | 비전·물리 인프라 |
| `scv_manager` | 관제 PC | `inspection_manager` | 관제·작업 관리 (A) |
| `scv_monitor` | 관제 PC | `ros_bridge`, 관제 웹·API | 관제·작업 관리 (B) |
| `scv_database` | 관제 PC | PostgreSQL·Redis 스키마·설치 (노드 없음, 10/12) | 관제·작업 관리 (B) |
| `scv_bringup` | 5대 전부 | launch, params, 지도, `facilities.yaml` | PM |
| `scv_testbed` | PC | 가짜 데이터 도구, 시험 스크립트 | PM |

5대 = 로봇 RPi 2대(4번·3번) · 로봇 PC 2대(MSI GPU, 로봇 위에 얹음 · 10/12) · 관제 PC 1대(Victus). **모두 turtle08**에 붙습니다. Discovery Server는 RPi 4번 · RPi 3번 · 관제 PC 세 곳이고, 로봇 PC는 자기 로봇 RPi 서버의 클라이언트입니다.

## 인터페이스

IF 번호는 시스템 다이어그램과 같습니다 (IF-03·13 은 삭제된 번호). IF-01(핫스팟 RF) · 17(Socket.IO·SQL) · 18(HTTP)은 ROS 밖이라 이 표에 없다 (다이어그램 참고).

| IF | 이름 | 종류 · 타입 | 보내는 쪽 → 받는 쪽 | QoS · 주기 | 주인 |
| --- | --- | --- | --- | --- | --- |
| 02 | `{ns}/survey/raw` | topic · `scv_msgs/NetRaw` | rssi_scanner → manual_tagger · precise_server (같은 로봇 PC). 측정 채널 고정 `rssi_freq` (전체 대역 스캔은 통신을 끊음), 스캔하지 않은 tick 은 보내지 않음 | reliable · transient_local · KEEP_LAST 600 · 1 Hz | RSSI A |
| 04 | `{ns}/survey/precise` | action · `scv_msgs/PreciseMeasure` | mission_executor → precise_server (같은 로봇 PC). 요청 10~15 s, 측정 전 정지 · 칸 · 위치 확인(measurement_guard) | 재측정 칸마다 1회 | RSSI A |
| 05 | `{ns}/inspect_facility` | service · `scv_msgs/InspectFacility` | mission_executor → detection_alert (`scv_detection_manager`, 같은 로봇 PC). 종류별 검사(소화기 · 유도등 · 랙 YOLO, 셔터 RGB-D)를 켜고, 판정이 나도 `facility_min_s`(3 s)는 계속 본 뒤 마지막 확정 판정으로 응답 · 확정이 없으면 `facility_window_s`(6 s)까지 → 끄고 IF-19 발행 (10/16). 로봇은 응답을 받을 때까지 관측 자리에 서 있다. 관측 자리가 막히면 둘레 0.15–0.45 m 대체 자리에서 같은 방향으로 (reason `ALT_VIEW`) | 설비마다 1회 · 응답 최대 30 s · 자세 보정 주행 없음 | 비전 B |
| 06 | `{ns}/dock` · `{ns}/undock` | action · `irobot_create_msgs` | mission_executor (로봇 PC) → Create 3 | — | 주행 |
| 07 | `{ns}/scan` · `odom` · `tf` · `tf_static` / `{ns}/cmd_vel` | topic · 표준 타입 | RPi ↔ Nav2 (로봇 PC) | 센서 best effort · cmd_vel reliable | 주행 |
| 08 | `{ns}/survey/sample` | topic · `scv_msgs/NetSample` | manual_tagger (로봇 PC, 위치는 TF map→base_link) → ros_bridge · cell_map_writer (관제 PC). 미검출도 보냄, 관제는 진행 중 회차 · 도크 밖 표본만 저장 | reliable · depth 50 · 1 Hz (sync 재발행은 depth를 넘지 않게 천천히, 예: 20 Hz 이하) | RSSI A |
| 09 | `{ns}/survey/sync` | service · `scv_msgs/SyncSamples` | ros_bridge (관제 PC) → survey_buffer (로봇 PC) — **미구현** (survey_buffer 없음, 정의만 남김) | 재연결 때 | RSSI A |
| 10 | `{ns}/keepout_mask` | topic · `nav_msgs/OccupancyGrid` | inspection_manager (관제 PC) → Nav2 (두 로봇 PC — 시작부터 빈 마스크, 점검 작업 전에 **재측정 뒤에도 미검출로 저장된 칸**만, Dock 0.6 m 제외) | reliable · transient_local | 관제 A |
| 11 | `{ns}/execute_task` | action · `scv_msgs/ExecuteTask` | inspection_manager (관제 PC) → mission_executor (로봇 PC) | — | 관제 A |
| 12 | `{ns}/status` | topic · `scv_msgs/RobotStatus` | mission_executor (로봇 PC) → inspection_manager · ros_bridge (관제 PC) | reliable · 1 Hz | 관제 A |
| 14 | `{ns}/oakd/rgb/image_raw/compressed` · `oakd/stereo/image_raw/compressedDepth` | topic · `sensor_msgs/CompressedImage` | OAK-D (RPi) → detection_alert · camera_preview (로봇 PC). 관제 PC 는 원본을 받지 않음 (Wi-Fi 대역) | best effort · 구독할 때만 | 비전 B |
| 15 | `/inspection/candidates` | topic · `scv_msgs/Candidate` | network_map_engine → inspection_manager (관제 PC 안) | reliable · transient_local · KEEP_LAST 100 | RSSI B |
| 16 | `/inspection/alerts` | topic · `scv_msgs/Alert` | inspection_manager → 관제 웹 (관제 PC 안) | reliable | 관제 A |
| 16 | `/inspection/start` · `/inspection/estop` · `/inspection/resume` · `/inspection/abort` | service · `std_srvs/Trigger` | 관제 웹(ros_bridge) → inspection_manager. abort = 순찰 종료(작업 취소 → 로봇마다 DOCK → 회차 aborted), estop = 멈추고 재개 대기(재개 때 남은 waypoint 만) | — | 관제 A |
| 16 | `/inspection/reassign` | service · `scv_msgs/Reassign` | 관제 웹 → inspection_manager | — | 관제 A |
| 19 | `{ns}/inspect/result` | topic · `scv_msgs/InspectionResult` | detection_alert (로봇 PC) → inspection_manager · ros_bridge (관제 PC). IF-05 판정마다 1건(회차 · 작업 · 설비 id · 근거 사진 640 px). 소화기면 `detail` 이 JSON `{text, aruco:[{marker_id, manufacture_date, manufacture_number, expiry_date, expiry_status}]}` → DB `inspections.aruco`. (비전 원래의 검사 중 순간 불량 이벤트는 10/16 부터 별도 토픽 `{ns}/inspect/defects`(VOLATILE · id 없음, 관제는 구독 안 함) — 같은 토픽에 QoS 가 다른 발행자가 있어 나던 「incompatible QoS」 경고를 없앰) | reliable · transient_local · KEEP_LAST 50 | 비전 B |
| 20 | `{ns}/camera/preview` | topic · `sensor_msgs/CompressedImage` | camera_preview (로봇 PC, 주행 중만 · 640 px 1 Hz, 비전이 있으면 검출을 그린 화면) → ros_bridge (관제 PC) → 블랙박스 파일 `scv_media/recordings/r{run}/{robot}/{ms}.jpg` (이력 탭 재생) | best effort · depth 1~5 | 관제 B |

## 타입 한눈에

| 타입 | 담는 것 |
| --- | --- |
| `NetRaw` | 측정 시각, 로봇, 검출 여부, 핫스팟 RSSI |
| `NetSample` | `NetRaw` + seq, 회차, 위치(x·y·yaw), `pose_ok`, `resent` (검출 여부 포함) |
| `RobotStatus` | 상태, 배터리, 새 작업을 받는지, 수행 중인 작업·진행률, 위치, 경고, 회차 |
| `Candidate` | 이상 후보: 출처(네트워크·비전), 종류, 대상, 갈 위치, 점수, 근거, 발견 로봇·회차 |
| `InspectionResult` | 설비 판정 한 건: 설비, 종류, 판정, 사진, 관측 횟수, 사진 축소본(CompressedImage) |
| `Alert` | 알림: 수준, 코드, 로봇, 대상, 문구, 늦게 전달됐는지 |
| `SyncSamples` (srv) | 회차와 `from_seq` 를 주면 그 뒤 샘플을 `survey/sample` 로 다시 발행 |
| `Reassign` (srv) | 한 로봇의 남은 작업을 다른 로봇으로 (받는 로봇 비우면 manager가 고름) |
| `ExecuteTask` (action) | 작업 한 건: 종류(SURVEY_SEGMENT·FACILITY_CHECK·REINSPECTION·DOCK), 웨이포인트, 대상, 회차 → 결과 ok·사유, REINSPECTION 이면 `rssi_median · rssi_iqr · n_samples(검출 수) · n_total · n_missed · rssi_mean · duration_s · measurement_valid · measurement_reason` |
| `PreciseMeasure` (action) | 정지 측정: 시간(10~15 s) → 위 9개 결과 필드 (중앙값으로 판정, 무표본 · 3개 미만 · 조건 실패는 판정 불가) |
| `InspectFacility` (srv) | 설비 점검: 작업 · 회차 · 설비 · 종류 → 판정 · 설명 · 원본 경로 (자세 보정 없음, 옛 action 은 10/15 삭제) |

필드와 상수는 각 `.msg` · `.srv` · `.action` 파일의 주석을 보세요.

## 관제 PC 안의 계약 — DB · 웹 API (ros 모드, 10/15)

순찰의 주인은 `inspection_manager`(ROS)이고, 웹(`server_page`)은 같은 프로세스 안 `ros_bridge` 로 ROS 와 이어져 보여 주고 버튼으로 부르기만 합니다. DB 는 PostgreSQL 16 (`scv_db/sql/01_schema.sql`, 여러 번 적용해도 되는 업그레이드 포함).

| 데이터 | 쓰는 쪽 | 표 · 칼럼 | 비고 |
| --- | --- | --- | --- |
| 회차 · 작업 | inspection_manager | `runs` · `tasks` (1초마다 sync) | 관제 재시작 때 남은 running 회차는 aborted |
| 로봇 상태 | ros_bridge ← IF-12 | `robot_status_log` (1 Hz) | 30초 무수신이면 E3 |
| RSSI 표본 | ros_bridge ← IF-08 | `net_samples` (`detected`, 미검출이면 `rssi_dbm` NULL) | 진행 중 회차 · 도크 밖만, 측정 AP(`aps.is_target`)로 등록된 SSID 만 |
| 칸 통계 | cell_map_writer · inspection_manager | `grid_stats` (`median` 판정, `mean`, `n_detected`, `n_missed`) | 재측정 결과로 갱신 |
| 설비 점검 | ros_bridge ← IF-19 | `inspections` (`aruco` JSONB) · `media` (근거 사진) | 설비는 `facilities` 에 등록돼 있어야 저장 (외래키) |
| 알림 | ros_bridge ← IF-16 alerts | `robot_events` | E1 막힘 · E3 무응답 · E6 판정 못 함 · E7 비상정지 · CLEARED/REMAINS/UNCONFIRMED · SAVE_PENDING |
| 블랙박스 | ros_bridge ← IF-20 | 파일 `scv_media/recordings/r{run}/{robot}/{ms}.jpg` | DB 표 없음 |

| 웹 API (로그인 쿠키) | 하는 일 |
| --- | --- |
| `POST /api/runs` · `/api/runs/{id}/abort` · `/api/runs/{id}/stop` · `/api/runs/resume` | 순찰 시작 · 비상정지 · 순찰 종료 · 재개 → IF-16 서비스 |
| `GET /api/runs/{id}` (`?lite=1`) | 회차 · 작업 · 후보 · 점검(사진 · ArUco) · 히트맵 · 방문 궤적 |
| `GET /api/runs/{id}/recording` | 블랙박스 프레임 목록 (로봇별 촬영 시각 · 경로) |
| `WS /ws/ui` | 로봇 상태 · 알림 · 계획 경로 · 순찰 시작 가능 여부 (1초) |

## 설계 문서에 없어서 PM 이 채운 부분 — 검토해 주세요

아래는 설계 초안에 필드 정의가 없거나 서로 달라서 임시로 정한 것입니다. 주인이 확인하고 고쳐 주세요.

| 무엇 | 임시로 정한 내용 | 확인할 사람 |
| --- | --- | --- |
| `NetRaw` 필드 | 시각, `robot_id`, `detected`, `rssi_dbm` | RSSI |
| 미검출 처리 | 미검출 샘플은 RSSI 통계에 넣지 않고, 지도 칸마다 미검출 비율을 따로 센다 (주행과 묶은 시험에서 바뀔 수 있음) | RSSI · 주행 |
| `SyncSamples` 응답 | 샘플을 응답에 담지 않고 `survey/sample` 로 다시 발행, 응답은 개수만 | RSSI A · 관제 B |
| `RobotStatus` 필드 전체 | 상태 7가지, `accepting_tasks`, `warning` 등 | 관제 A · 주행 |
| `InspectionResult` 필드 전체 | 판정·종류 상수를 여기에 두고 `InspectFacility` 가 함께 씀 | 비전 B · 관제 |
| `Alert` 필드 전체 | `code` 는 문자열 (CANDIDATE, CONFIRMED, E1–E8 …) | 관제 A |
| `Candidate` 위치 | 초안의 `geometry_msgs/Pose2D` 는 폐기 예정 타입이라 `x`·`y`·`yaw` 로 바꿈 | RSSI B · 관제 A |
| `Candidate` 종류 | 혼잡·로밍 의심은 범위에서 빠져 상수에서 뺌 (1 과 4 만 남김) | RSSI B |
| `ExecuteTask` 종류 | 다이어그램에 있는 `DOCK` 을 추가. DOCK 작업은 inspection_manager 가 만든다 (측정 → DOCK → 재측정 → DOCK → 점검 → DOCK, 10/12) | 관제 A · 주행 |
| 회차 전달 | manager가 `ExecuteTask.run_id`에 넣고, mission_executor가 `RobotStatus.run_id`로 내보내고, survey_buffer가 그것을 읽음 · 설비 점검은 mission_executor가 `InspectFacility` 요청의 `run_id`로 넘기고 detection_alert가 `InspectionResult.run_id`에 넣음 (10/12) | 관제 A · 주행 · RSSI A |
| 재측정 결과 (10/15 갱신) | mission_executor가 `PreciseMeasure` 결과 9개 필드를 `ExecuteTask` 결과에 그대로 넣음. inspection_manager가 유효성(measurement_valid · 10~16 s · n_total = 검출 + 미검출) 확인 뒤 중앙값 · 미검출 비율로 판정하고, **저장에 성공한 뒤에만** CLEARED/REMAINS 알림. 저장 실패면 SAVE_PENDING 알림 · 순찰은 계속 · 같은 결과로 재시도 | 주행 · 관제 A · RSSI |
| 근거 사진 축소본 | JPEG, 긴 변 640 px 이하. 원본은 로봇 PC `image_ref` | 비전 B · 관제 B |
| 네트워크 후보의 `robot_id` | 측정한 로봇의 이름 (10/12: 두 로봇이 각자 섹터를 측정) | RSSI B · 관제 A |
| `Reassign` 형식 | 보낼 로봇 · 받을 로봇(비우면 manager) → 옮긴 작업 수 | 관제 A |
| `resent` | sync 요청으로 다시 보낸 샘플만 true | RSSI A · 관제 B |
| ~~자세 보정 "뒤로" (`adjust_forward_m` < 0)~~ — **10/14 폐기: 자세 보정 주행 없음** | 로봇이 후진 명령을 무시한 적이 있고 후진은 지양. mission_executor는 보정값을 cmd_vel로 보내지 않고 **Nav2 목표 위치로 바꿔** 보낸다 (뒤 지점이면 돌아서 갔다가 방향을 다시 맞춤). Nav2 행동 트리의 BackUp 복구 동작은 뺀다. 설비 관측 위치는 1.0–1.2 m로 등록해 뒤로 물러날 일을 줄인다 (OAK-D depth는 약 0.7 m부터 유효) (제안) | 비전 B · 주행 |

## 설정 파일 (형식 초안)

`ros2_ws/src/scv_bringup/config/` 에 있습니다. 값은 테스트베드에서 잰 뒤 PM 이 채웁니다. 좌표는 저장 지도(`map`) 기준 [m], 각도 [rad].

```yaml
# facilities.yaml — 설비 등록. kind 는 InspectionResult 의 KIND_* 와 같다
facilities:
  - id: EXT-01
    kind: extinguisher                  # extinguisher · shutter · exit_light · rack
    expected: {x: 0.0, y: 0.0}          # 설비가 있어야 할 위치 (shutter 는 band: 하강 띠의 네 꼭짓점)
    view: {x: 0.0, y: 0.0, yaw: 0.0}    # 로봇이 서서 볼 자세 (설비에서 1.0–1.2 m)

# zones.yaml — 측정 구역 = 섹터 (10/12: 2개, 로봇 하나가 섹터 하나)
zones:
  - id: Z1
    robot: robot4                       # 이 섹터를 맡는 로봇 (robot4 · robot3)
    dock: {x: 0.0, y: 0.0}               # Dock 위치 (초기 위치·Dock 접근 기준, 주행 목표 아님)
    polygon: [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
    spacing: 0.5                        # 왕복 경로 간격 [m] = 칸 크기 (10/12)
  - id: Z2
    robot: robot3
    dock: {x: 0.0, y: 0.0}
    polygon: [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
    spacing: 0.5
```

기기별 실행: `ros2 launch scv_bringup robot_rpi.launch.py robot_ns:=robot4` · `robot_pc.launch.py robot_ns:=robot4` · `control_pc.launch.py` — 노드가 준비되면 PM 이 launch 에 넣습니다.

## 빌드

```bash
deactivate 2>/dev/null   # 가상환경이 켜져 있으면 끄고 빌드
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 interface list | grep scv_msgs
# 11개 (msg 6 · srv 3 · action 2)
```

`scv_msgs` 를 고치면 그것을 쓰는 패키지도 다시 빌드해야 합니다.

가상환경(venv)이 켜진 채 빌드하면 `scv_msgs` 가 `ModuleNotFoundError: No module named 'em'` 으로 실패합니다. cmake 가 가상환경의 python 을 잡아서, 메시지 코드 생성에 필요한 도구(empy 등)를 못 찾기 때문입니다. `deactivate` 로 끄고 새 터미널에서 빌드하세요.
