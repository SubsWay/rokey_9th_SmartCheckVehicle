# SCV System Design Document (제출용 초안 v0.1)

2026-10-10 · 작성 도윤 (PM) · 출처: 팀 노션 「SCV 설계」, 요구사항·아키텍처 v1.0, 시스템 다이어그램 v2.1, 저장소. **저장소(`scv_msgs`, `docs/interfaces.md`)와 다르면 저장소가 맞습니다.**

## 1. 개요

TurtleBot4 두 대를 이동형 인프라 점검 플랫폼으로 써서, 물리 인프라(소화기·방화셔터 하강구간·비상구 유도등·랙 적재물 돌출)와 통신 인프라(핫스팟 RSSI)를 같은 순찰에서 위치 기반으로 점검한다. 두 로봇이 섹터를 하나씩 맡아 동시에 측정 → 재측정 → 설비 점검을 하고, 측정으로 만든 통신 지도(히트맵)에서 재측정 뒤에도 남은 음영을 점검 때 피한다 (10/12 시나리오). 요구사항은 `requirements.md`, 흐름은 `process_flow.md`, 시험은 `../TEST_PLAN.md`.

과제의 세 모듈:

| 모듈 | 노드 |
| --- | --- |
| Detection Alert | `detection_alert` (설비 점검 서비스 서버, 소화기·유도등·랙은 yolo26n · 셔터 띠는 depth) |
| AMR Controller | Nav2·AMCL, `mission_executor`, `rssi_scanner`, `survey_buffer`, `inspection_manager` |
| System Monitor | `ros_bridge`, `network_map_engine`, 관제 웹·API, PostgreSQL·Redis (관제 팀 설계, 10/12) |

## 2. 기기와 배치

| 기기 | 대수 | 돌리는 것 |
| --- | --- | --- |
| 로봇 RPi (TurtleBot4 탑재) | 2 (4번 `/robot4`, 3번 `/robot3`) | 드라이버(Create 3 · LiDAR · OAK-D), Discovery Server (10/12 저녁: `rssi_scanner` 는 로봇 PC 로) |
| 로봇 PC (MSI GPU 노트북, 로봇마다 1대 · 로봇 위에 얹음) | 2 | Nav2·AMCL·keepout 필터, `mission_executor`, `rssi_scanner`, `survey_buffer`, `detection_alert` — 두 로봇이 같은 구성 (10/12) |
| 관제 PC (Victus 노트북) | 1 | `inspection_manager`, `ros_bridge`, `network_map_engine`, 관제 웹·API, PostgreSQL·Redis (Docker), Discovery Server |
| 휴대폰 핫스팟 | 1 | 측정 대상. 로봇 통신에는 쓰지 않는다 |

- 공통: Ubuntu 24.04 · ROS 2 Jazzy · FastDDS. 모든 기기가 Wi-Fi `turtle08`.
- Discovery Server 는 RPi 4번 · RPi 3번 · 관제 PC 세 곳. 로봇 PC 는 자기 로봇 RPi 서버의 클라이언트.
- Nav2·AMCL 과 YOLO 추론, RSSI 측정(10/12 저녁)은 RPi 가 아니라 로봇 위에 얹은 로봇 PC 에서 돈다(RPi4 성능). 설비 점검 루프가 로봇 PC 안에서 닫혀, 관제 PC 가 끊겨도 판정이 계속된다.
- 음영은 측정 대상(핫스팟) 신호가 약한 곳일 뿐이고, 로봇 통신(turtle08)은 끊기지 않는다. 그래서 재측정은 음영 칸 안에 들어가서 하고, 재측정 뒤에도 남은 칸만 설비 점검 때 피한다 (10/12).
- 로봇 PC ↔ 관제 PC 는 ROS 2 로 주고받는다 (아래 IF). 관제 PC 안의 DB·웹(PostgreSQL + Redis + 웹 API)은 관제 팀 설계를 따르고, 로봇 PC 는 DB 에 직접 접속하지 않는다 (10/12).

## 3. 패키지와 팀

| 패키지 | 실행 위치 | 노드 | 팀 · 브랜치 |
| --- | --- | --- | --- |
| `scv_msgs` | 전부 | 메시지 6 · 서비스 2 · 액션 3 | PM |
| `scv_survey` | 로봇 PC · 관제 PC | `rssi_scanner`, `survey_buffer`, `network_map_engine` | RSSI · `feature/survey` |
| `scv_nav` | 로봇 PC | `mission_executor`, Nav2 설정 | 주행 · `feature/driving` |
| `scv_detection` | 로봇 PC | `detection_alert` | 비전 · `feature/vision` |
| `scv_manager` | 관제 PC | `inspection_manager` | 관제 · `feature/control` |
| `scv_monitor` | 관제 PC | `ros_bridge`, 관제 웹·API | 관제 · `feature/control` |
| `scv_database` | 관제 PC | PostgreSQL·Redis 스키마와 설치 스크립트 (노드 없음) | 관제 · `feature/control` |
| `scv_bringup` | 전부 | 기기별 launch, 설정(`facilities.yaml`, `zones.yaml`), 지도 | PM · `feature/pm` |
| `scv_testbed` | PC | 시험 스크립트 | PM · `feature/pm` |

모든 브랜치에 패키지 전체가 있고 팀은 자기 패키지만 고친다. `main` 이 통합본이다. 로봇 PC·관제 PC·RPi 는 `main` 으로 돌린다.

실행:

```bash
ros2 launch scv_bringup robot_rpi.launch.py robot_ns:=robot4     # 로봇 RPi (우리 노드 없음 · 드라이버는 turtlebot4 bringup)
ros2 launch scv_bringup robot_pc.launch.py  robot_ns:=robot4     # 로봇 PC
ros2 launch scv_bringup control_pc.launch.py                     # 관제 PC
```

## 4. 노드가 하는 일

| 노드 | 하는 일 |
| --- | --- |
| `rssi_scanner` | 핫스팟을 스캔해 RSSI 와 검출 여부를 1 Hz 로 발행(위치는 붙이지 않음). 요청이 오면 정지 30초 반복 스캔 → 중앙값·IQR·표본 수 |
| `survey_buffer` | `rssi_scanner` 샘플에 같은 시각의 AMCL 위치와 seq 를 붙여 로봇 PC 디스크에 먼저 저장하고 관제 PC 로 발행. 요청이 오면 지정한 seq 부터 다시 발행 |
| Nav2·AMCL | 저장 지도 위 위치 추정·주행. 관제 PC 가 보낸 keepout 마스크의 칸을 경로에서 뺀다 (측정·재측정 동안은 빈 마스크, 점검 단계에 들어갈 때 남은 칸 · `keepout_layer` 는 빈 마스크 발행을 확인한 뒤 켠다) |
| `mission_executor` | 받은 작업(측정 구간·설비 점검·재검증·Dock)을 Nav2 로 수행하고 결과를 보고. 상태를 1 Hz 로 발행. 관제 PC 가 끊겨도 받은 작업은 끝낸다 |
| `detection_alert` | OAK-D 영상은 계속 받고, 점검 요청(IF-05 서비스)이 오면 YOLO(yolo26n) → 등록 위치와 대조 → 정면 판별(아니면 UNCONFIRMED — 자세 보정 주행 없음, 10/14) → 판정. 셔터 띠 위 물체는 학습 없이 depth 로(띠 영역에 바닥보다 10 cm 이상 솟은 것), 랙 적재물 돌출도 yolo26n 으로 (10/14, 처음엔 OpenCV). 사진은 로봇 PC 에 저장하고 축소본을 결과와 함께 보낸다 |
| `inspection_manager` | 작업·후보 상태의 주인. 섹터마다 경로 생성과 작업 배정(측정 → Dock → 재측정 → Dock → 점검 → Dock, Dock 작업도 manager 가 만든다), 후보 → 재측정 작업(그 섹터의 로봇), 재측정 결과를 DB 의 그 칸에 기록, **keepout 마스크 발행(IF-10)** — 시작할 때 빈 마스크, 로봇마다 점검 작업 전에 DB 맵의 남은 칸으로, 음영 때문에 못 가는 설비의 예외 처리, 측정 경로의 웨이포인트를 지도와 벽 여유 0.45 m 로 걸러냄(지도 안 벽 조각), 이탈 감지·경보, 최종 판정, 비상정지 (이탈 시 재배정은 풀사이클 뒤 추가 기능) |
| `network_map_engine` | 두 로봇의 샘플로 칸 맵(0.5 m 칸 · 중앙값 · 칸 중심 좌표 · 상태 미측정/미검출/음영/정상) → 관제 API 로 DB 에 저장 → 재측정 대상 칸(음영·미검출·미측정)을 후보(IF-15)로 · 기준선 비교 · 보고서 레이어. keepout 마스크는 내지 않는다 (DB 맵을 읽어 inspection_manager 가 낸다, 10/12 회의 뒤) |
| `ros_bridge` | ROS 로 받은 샘플·상태·판정·알림을 관제 API 로 넘겨 PostgreSQL 에 저장(중복은 DB 유일키가 거름), 실시간 상태는 Redis 로 화면에 전달 |
| 관제 웹 | 순찰 시작·비상정지 (재배정 버튼은 추가 기능), 네트워크 지도·작업/로봇·점검 이력, 보고서·점검표 |

## 5. 인터페이스

이름·타입·방향·QoS·주인은 [`docs/interfaces.md`](../interfaces.md), 필드는 `ros2_ws/src/scv_msgs/` 의 정의 파일이 원본이다.

| 구간 | 인터페이스 |
| --- | --- |
| RPi ↔ 로봇 PC | IF-06 Dock/Undock · IF-07 센서·`cmd_vel` · IF-14 OAK-D 영상 |
| 로봇 PC 안 | IF-02 측정 raw · IF-04 정지 측정 · IF-05 설비 점검 |
| 로봇 PC ↔ 관제 PC | IF-08 측정 샘플 · IF-09 sync · IF-10 keepout 마스크 · IF-11 작업 · IF-12 상태 · IF-19 판정 결과 |
| 관제 PC 안 | IF-15 후보 · IF-16 알림·시작·비상정지·재개·재배정 · ROS ↔ 웹·DB 는 `ros_bridge` |

로봇 PC ↔ 관제 PC 의 IF 는 모두 ROS 2 그대로다. 10/12 에 바뀐 것은 관제 PC 안의 저장·화면(SQLite·Flask → PostgreSQL·Redis·웹 API)뿐이다.

## 6. 데이터

- 측정 샘플: 시각, 로봇, 회차, 위치(x·y·yaw), 위치 확실 여부, 검출 여부, RSSI. 미검출은 RSSI 값으로 적지 않고 검출 여부로 표시한다.
- 지도: 칸 0.5 m · 칸 값은 검출된 RSSI 의 중앙값 · 검출 0개이거나 미검출 비율 ≥ 0.5 이면 미검출 · 중앙값 < −72 dBm 이면 음영 (`SHADOW_DBM`, −75 dBm 은 지도 색 기준) · 회피 마스크는 샘플 2개 이상 칸만 (`min_samples_for_mask`) (`scv_survey` 코드값, 9d3a4cc). 칸은 네 가지 — 미측정 · 미검출 · 음영 · 정상. 미측정은 갈 수 있는 칸(섹터 안, 벽·장애물에서 0.45 m 밖) 중 샘플이 없는 칸이다. 측정 맵은 network_map_engine 이 관제 API 로 DB 에 올리고, 재측정과 점검은 DB 의 그 맵을 쓴다. 보간 칸은 따로 표시하고 판정에 쓰지 않는다.
- 설비 판정: 종류는 소화기 · 셔터 · 유도등 · 랙(`KIND_RACK`, 10/12). NORMAL · NOT_DETECTED · OCCLUDED · POSITION_ANOMALY · ABNORMAL(압력·소등·적치·돌출) · UNCONFIRMED → 최종 정상 / 이상 / 미확인.
- 후보: 출처(네트워크·비전), 종류, 대상(격자 칸 또는 설비), 발견 로봇, 회차 → 재검증 뒤 확정 · 해제 · 미확인. 음영 후보의 재측정은 그 섹터를 맡은 로봇(발견 로봇과 같다)이 한다.
- DB: PostgreSQL 16 + Redis (관제 팀 설계 `scv_database`, 10/12). runs · net_samples · grid_stats · baselines · facilities · candidates · tasks · inspections · alerts 등 표의 원본은 그 패키지의 `sql/01_schema.sql`. 관제 PC 의 API 만 DB 에 쓴다.
- 설정: `scv_bringup/config/facilities.yaml`(설비 번호·종류·예상 위치·관측 자세), `zones.yaml`(섹터 2개 — 구역 꼭짓점·맡는 로봇 `robot:`·Dock 위치 `dock:`·경로 간격), 기준 지도 `scv_bringup/maps/testbed_v1.yaml`(주행 팀이 만든 지도, 10/11 PM 확정).

## 7. 다중 로봇 운영

두 로봇이 동시에 주행한다 (10/12 PM 결정). 구역을 섹터 2개로 나눠 로봇 A(`/robot4`)·로봇 B(`/robot3`)가 하나씩 맡고 같은 일을 한다. 어느 로봇이 어느 섹터인지는 `zones.yaml` 의 `robot:` 에 적는다.

1. **측정**: Undock → 자기 섹터를 왕복하며 RSSI 측정 → Dock.
2. **재측정** (있을 때만): 자기 섹터에 음영·미검출·미측정 칸이 있으면 Undock → 그 칸 안에서 정지 30초 측정 → Dock. 없으면 건너뛴다. 정지 측정 중앙값이 −72 dBm 이상이면 해제, 아니면(또는 여전히 잡히지 않으면) 회피 구역이 된다.
3. **점검**: Undock → 자기 섹터의 설비를 점검 → Dock. 재측정 뒤에도 남은 음영·미검출·미측정 칸은 keepout 마스크로 피해 간다 (inspection_manager 가 점검 작업 전에 DB 맵으로 만들어 그 로봇에 보낸다). 그 때문에 갈 수 없는 설비는 관제가 예외로 처리한다(미확인). 재측정 뒤에도 남은 칸은 관제에 알림을 띄워 사람이 확인한다.

- 로봇마다 최대 3바퀴, 재측정이 없으면 2바퀴. 두 로봇은 서로 기다리지 않는다 — 한 로봇만 재측정이 필요하면 다른 로봇은 바로 점검으로 넘어간다.
- 마스크는 시작부터 빈 것을 두 로봇에 발행해 둔다(발행자가 없으면 Nav2 경로 계획이 실패). 로봇마다 점검 단계에 들어갈 때 재측정이 끝난 섹터의 남은 칸으로 그 로봇에 보낸다(지도 전체 크기 · Dock 반경 0.6 m 제외). 측정·재측정 중인 로봇은 빈 마스크 그대로다.
- 두 Dock 은 각자 섹터 안에 떨어져 있어(약 3.6 m) 두 로봇이 동시에 출발하고 각자 Dock 으로 돌아온다 (10/12 실측: Dock A (3.18, 1.00) · Dock B (3.18, 4.60)).
- 설비 후보는 점검한 로봇이 자세를 바꿔 스스로 다시 보고(최대 3회), 안 되면 미확인으로 남긴다.
- 작업의 주인은 로봇이 아니라 `inspection_manager` 의 작업 목록이다. 두 로봇은 서로 직접 통신하지 않는다.

## 8. 설계 결정

| # | 결정 |
| --- | --- |
| D1 | YOLO 는 로봇 PC(MSI GPU)에서 추론 |
| D2 | RSSI 는 로봇 위 로봇 PC 의 내장 무선랜으로 핫스팟 채널만 스캔해 읽는다 (10/12 저녁: RPi → 로봇 PC · 스캔 중 turtle08 통신이 흔들리면 USB 무선랜으로 분리) |
| D3 | 측정 경로는 구역별 왕복 + 설비 지점 |
| D4 | 섹터 2개를 두 로봇이 하나씩 맡아 같은 일을 동시에 한다 (10/12 변경 — 처음엔 스카우트 + 검증자, 10/11 은 역할 분리·번갈아 구동) |
| D5 | 웹·DB 와 ROS 는 관제 PC 의 `ros_bridge` 로 잇는다 (웹이 죽어도 수집 계속). 웹·DB 는 관제 팀 설계 PostgreSQL + Redis + 웹 API (10/12, 처음엔 Flask + SQLite) |
| D6 | 칸 0.5 m · 중앙값 · 미검출 비율 0.5 · 음영 < −72 dBm (색 기준 −75) · 마스크는 샘플 2개 이상 칸 (10/12, `scv_survey` 코드값) · 변화 판정 수치는 첫 측정을 보고 정한다 |
| D7 | 측정 버퍼는 로봇 PC 에 둔다 (위치 결합도 로봇 PC) |
| D8 | 관제 PC 와 끊기면 받은 작업은 끝까지, 새 작업만 대기 |
| D9 | 시험 AP 는 휴대폰 핫스팟 (로봇과 통신하지 않음) |
| D10 | 측정 지표는 RSSI 만 (BSSID·채널·로밍·지연·손실·처리량 제외) |
| D11 | 상태 추정은 커버리지 저하 의심 · 판단 불가 두 가지, 회피 기준과 해제 규칙 포함 |
| 10/14 | 설비 점검 요청(IF-05)은 액션이 아니라 서비스로 한다 (비전 팀 구현 기준) — 판정 노드는 영상을 계속 받고 요청 때 한 번 판정한다. 비전용 자세 보정 주행은 없다 — 판정 못 하면 UNCONFIRMED. 소화기·유도등·랙 판정은 모두 yolo26n 으로 바꿨다 (랙도 OpenCV 대신). mission_executor 쪽 변경은 PM 최종 통합(T2-6) 때 |
| 10/12 저녁 | RSSI 측정은 로봇 PC 에서 한다 (강사 조언) — 로봇 PC(MSI)를 두 로봇 위에 하나씩 얹고 `rssi_scanner` 를 그 PC 에서 돌린다 · RPi 에는 드라이버만 · IF-02·IF-04 는 로봇 PC 안 · 시연도 이 구성 · 노트북을 얹어 바뀌는 footprint·무게·Dock 진입·센서 시야는 주행 팀이 확인 · 재측정 결과는 `ExecuteTask` 결과 필드로, 설비 점검 목표에 `run_id` (interfaces v0.2.10) |
| 10/12 회의 뒤 | IF-10 은 inspection_manager 가 DB 맵을 읽어 낸다 (network_map_engine 은 맵을 DB 에 쓰고 후보를 낸다) · DB 쓰기는 모두 관제 API · 왕복 경로 간격 0.5 m · 미측정은 갈 수 있는 칸(섹터 안, 벽·장애물에서 0.45 m 밖) 중 샘플이 없는 칸 · 칸 값과 기준은 `scv_survey` 코드값 (미검출 비율 0.5 · 음영 < −72 dBm · 회피 마스크는 샘플 2개 이상 칸 — 10/12 저녁 정정, 코드 9d3a4cc) |
| 10/12 회의 | 전체 회의에서 정한 것 — 측정 칸은 0.5 m 정사각형이고, 칸의 값은 그 칸에서 검출된 RSSI 의 중앙값이다 (`scv_survey` 코드 기준) · 칸은 세 가지로 나눈다: 미측정(지나가지 않음) · 미검출(지나갔으나 잡히지 않음) · 음영(잡혔으나 약함) · 재측정 대상은 음영·미검출·미측정 칸이다 (미측정은 로직에만 넣고 시연에는 없다) · RSSI 팀이 측정이 끝난 맵을 DB 에 올려 두고, 재측정과 점검은 그 맵을 꺼내 쓴다 — 요청을 주고받지 않는다 (재측정은 대상 칸을 찾는 데, 점검은 회피에) · 재측정은 그 섹터를 맡은 로봇 · 설비(소화기·유도등·랙) 배치와 좌표는 비전 팀, 핫스팟 연출은 RSSI 팀이 시험하며 정한다 · 시연 시간 목표는 두지 않고 풀사이클 시험에서 잰다 · 관제 DB 인터페이스·스키마는 10/13 까지 확정 |
| 10/12 | 두 로봇이 동시에 주행한다 · 섹터 2개를 하나씩 맡아 같은 일을 한다 (RSSI 측정 → 재측정(필요할 때만) → 물리 인프라 점검, 사이마다 Dock) · 섹터와 맡는 로봇은 `zones.yaml` 의 `robot:` · 두 Dock 은 각자 섹터 안에 떨어져 있어(약 3.6 m) 두 로봇이 동시에 출발하고 각자 Dock 으로 돌아온다 · 점검 때는 재측정 뒤에도 남은 음영·미검출 칸을 피한다 (마스크는 지도 전체 한 장을 두 로봇에, 측정·재측정 동안은 빈 마스크) · 로봇마다 최대 3바퀴 · 측정·판정 노드는 두 로봇 쪽에 모두 · 음영 때문에 못 가는 설비는 관제가 예외 처리 · 랙 적재물 돌출 점검 추가 (OpenCV, `KIND_RACK`) · 셔터 띠 위 물체는 학습 없이 depth · 관제 PC 의 DB·웹은 관제 팀 설계(PostgreSQL + Redis + 웹 API), 로봇 PC ↔ 관제 PC 는 ROS 그대로 |
| 10/12 오후 | 두 로봇은 서로 기다리지 않는다 · 마스크는 시작부터 빈 것을 발행하고 로봇마다 점검 단계에 들어갈 때 남은 칸으로 · 재측정 뒤에도 남은 칸은 관제 알림(사람 확인) · 시연은 한 섹터에만 음영, 재측정에서 사라지는 흐름 (핫스팟 연출은 RSSI 팀과 확정) · 측정 경로 웨이포인트는 지도와 벽 여유 0.45 m 로 거른다 · 설비 좌표는 비전 팀이 10/12 저녁까지 초안 |
| 10/11 | 음영 재측정은 다른 로봇이 아니라 발견한 로봇이 한다 · 이탈 재배정과 배터리 인계는 풀사이클 뒤 추가 기능 · 기준 지도는 주행 팀의 `testbed_v1` · 설계 문서의 원본은 저장소 (A → B → A·역할 분리·번갈아 구동·마스크는 로봇 B 만·음영 구역 안에는 설비 없음은 10/12 에 바뀜) |
| 10/10 | 고정 웹캠은 쓰지 않는다 · 로봇은 4번·3번 · 미검출은 검출 여부로 표시 · 지도는 테스트베드에서 한 번 만든 것을 고정해 쓴다 |

## 9. 한계

- 강의장 축소 테스트베드의 결과이고 실제 창고를 재현한 것이 아니다 (BR-R07).
- 휴대폰 핫스팟으로 만든 음영은 실제 설비 AP 의 음영과 같지 않다.
- 로봇 높이에서 잰 신호다. 단말 높이의 품질과 다를 수 있다.
- 로봇과 로봇 PC 사이의 Wi-Fi 가 끊기지 않는 것을 전제로 한다.
