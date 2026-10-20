# ROS 관제 통합 실행 방법 (temp/integration-1014)

2026-10-14 · PM 도윤. 순찰의 주인은 `inspection_manager`(ROS)이고, 관제 웹(`server_page`)은 `ros_bridge` 로 상태 · 알림 · 판정 · 표본을 받아 보여 주며 버튼으로 `/inspection/start · estop · resume` 를 부른다. 웹이 작업을 만들거나 로봇에 직접 보내지 않는다. 로봇 PC 에서 `client_page` 는 띄우지 않는다.

```
브라우저 → server_page(SCV_RUN_OWNER=ros) ─ ros_bridge ─ /inspection/* ─▶ inspection_manager ─ExecuteTask─▶ mission_executor
                                   ◀ status · alerts · inspect/result · survey/sample (ROS)
inspection_manager → DB runs · tasks (1초마다) · cell_map_writer → DB grid_stats
```

DB 스키마는 바꾸지 않았다 (01_schema.sql 그대로).

## 1. 관제 PC 준비 (한 번)

```bash
cd ~/B_0102_Final_Project && git fetch origin && git switch temp/integration-1014
cd ros2_ws
deactivate 2>/dev/null                      # 빌드는 가상환경을 끄고
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to scv_bringup scv_manager scv_survey
```

웹 파이썬: `server_page` 는 rclpy(ROS)와 fastapi · psycopg 가 같은 파이썬에 있어야 한다.
- 지금까지 server_page 를 돌리던 파이썬(가상환경)이 파이썬 3.12 이면 그대로 쓴다 — ROS 를 source 한 셸에서 그 파이썬을 `web_python:=` 으로 넘긴다.
- 없으면 `python3 -m pip install --user -r ros2_ws/src/scv_app/requirements.txt`.

## 2. DB · 지도 준비 (한 번, 순찰이 없을 때)

1. DB 는 지금 쓰는 컨테이너 그대로 (`scv_db/docker/setup_db.sh`). 시험만 하려면 `./setup_db.sh --test` 로 시험 DB 를 만들고 `SCV_DB_DSN` 을 그 DB 로.
2. 아래 3절처럼 웹까지 띄운 뒤 브라우저에서 (관리자 계정):
   - 정보 › 지도: 기준 지도 `testbed_v1` (제어 PC 경로 = 로봇 PC 의 `scv_bringup/maps/testbed_v1.yaml`), **격자 0.5 m**, 현재 지도로.
   - **격자 다시 생성** 한 번 — ros 모드에서는 섹터가 `zones.yaml` 로 나뉜다 (섹터 1 = 아래 = robot4, 섹터 2 = 위 = robot3). 예전 격자(왼쪽 · 오른쪽)는 manager 와 맞지 않는다.
   - 도크 접근 자세(robot4 · robot3), 측정 AP = 실제 핫스팟 SSID (아래 `ap_ssid` 와 같은 값) 를 측정 대상으로.
   - 설비는 `facilities.yaml` 과 같은 ID 로 (지금 좌표가 있는 것은 SHT-01 · SHT-02).

## 3. 실행

**관제 PC** (터미널 하나, `SCV_DB_DSN` 은 평소 값):
```bash
source /opt/ros/jazzy/setup.bash && source ~/B_0102_Final_Project/ros2_ws/install/setup.bash
ros2 launch scv_bringup control_pc.launch.py backend:=db ap_ssid:=<핫스팟SSID> web:=true close_stale_run:=true
#   한 대만: robots:=robot4   ·   웹 파이썬 지정: web_python:=<가상환경>/bin/python3   ·   웹 폴더 지정: scv_app_dir:=<경로>
```
뜨는 것: `cell_map_writer` · `inspection_manager` · `server_page`(로그에 `[ros_bridge] 켜짐`). 웹은 `http://<관제PC>:8000`.

**로봇 PC** (로봇마다, 평소와 같음 — client_page 는 띄우지 않음):
```bash
ros2 launch scv_bringup robot_pc.launch.py robot_ns:=robot4 iface:=<무선랜> ssid:=<핫스팟SSID>     # 설비 판정은 fake_vision(기본)
```

**브라우저**: 모니터 탭 → 두 로봇 상태가 보이면 「순찰 시작」 → 진행 중에는 「비상정지」(두 로봇 정지) · 「재개」.

## 4. 확인할 것

| 무엇 | 기대 |
| --- | --- |
| 모니터 로봇 상태 | 두 로봇 위치 · 배터리 · 상태가 1–2초마다 바뀜, 「준비됨」이면 시작 버튼 켜짐 |
| 순찰 시작 | 응답 `run_id=N, 작업 M건 생성`, 이력 탭에 작업 줄이 생기고 ACTIVE → DONE |
| 히트맵 | 측정 중 칸 색이 찍힘 (net_samples) |
| 비상정지 → 재개 | 두 로봇 정지, 하던 작업은 대기로 → 재개하면 그 작업부터 |
| 최근 이벤트 | 시작 · 재측정 결과 · 회피 · E1 · E3 · E6 · 종료가 5초 안에 |
| 끝 | 회차 done (한 로봇 실패면 partial) |

## 5. 막힐 때

| 증상 | 확인 |
| --- | --- |
| 로봇이 「비정상 (상태 없음)」 | 관제 PC 에서 `ros2 topic echo /robot4/status` 가 오는지 (도메인 · Discovery Server · SUPER_CLIENT) |
| 시작 → 「/inspection/start 서비스가 없습니다」 | `inspection_manager` 가 떴는지, 같은 도메인인지 |
| 시작 → 「회차를 열 수 없습니다」 | manager 로그: DB 활성 지도 · 격자 0.5 m · origin 이 `testbed_v1` 과 같아야 함, 남은 running 회차는 `close_stale_run:=true` |
| `web:=true 는 backend:=db 와 함께` | backend:=db 를 같이 준다 |
| `server_page.py 가 없습니다` | `scv_app_dir:=~/B_0102_Final_Project/ros2_ws/src/scv_app` |
| 웹이 `No module named fastapi / rclpy` | 1절 웹 파이썬 — ROS 를 source 한 셸 + fastapi 있는 파이썬 |

## 6. 로봇 없이 시험 (노트북에서 해 본 방법)

```bash
python3 ros2_ws/src/scv_manager/test/mock_execute_task_server.py      # 가짜 로봇 두 대 (상태 · 표본 · 작업 수행)
ros2 launch scv_bringup control_pc.launch.py backend:=db ap_ssid:=<지도에 지정한 측정 AP> web:=true close_stale_run:=true
```
10/14 밤 결과: 웹 시작 → 작업 53건 모두 DONE → 회차 done, 비상정지 · 재개, 판정 · 사진 저장, E3 로봇마다 1건, 단위 시험 98 passed.

## 7. 아직 아닌 것

- 비전 실제 연결 · IF-05 서비스 전환 (T2-6, 비전 작업이 끝나면 이 브랜치에 묶는다).
- 영구 중단(회차를 aborted 로 끝내기) 버튼 — manager 에 `/inspection/abort` 가 없어 지금은 비상정지 · 재개만.
- 알림 중 DB 규칙(robot_events.code CHECK) 밖의 것은 화면에만 (RUN_STARTED · REMAINS · KEEPOUT_BLOCKED · INSPECTION_NORMAL · E7 등).
