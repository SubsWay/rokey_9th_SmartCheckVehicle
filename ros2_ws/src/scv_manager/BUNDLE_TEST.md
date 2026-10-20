# 묶음 시험 실행 순서 (RSSI + 주행 + 관제) — `feature/control` (10/13 `temp/bundle-test-1012` 에서 병합)

시연과 같은 배치: 로봇마다 로봇 PC(MSI) 한 대, 관제 PC(VICTUS) 한 대.
로봇은 PM 이 허락하고 다른 팀이 쓰지 않을 때만 움직인다.

## 흐름

```
두 로봇 측정(SURVEY) → Dock → 칸 맵 확정 → 자기 섹터의 미검출·음영 칸 재측정(30초 정밀 측정) → Dock
→ 남은 칸을 회피 마스크로 → 설비 점검 → Dock → 회차 종료
```

- robot4 = Z1(아래, 셔터 띠), robot3 = Z2(위). `robot_ids`·`zone_ids` 파라미터의 순서로 짝짓는다.
- 설비 좌표가 (0, 0) 인 EXT-01·EXL-01·RCK-01 은 건너뛴다. 지금은 robot4 의 SHT-01·SHT-02 만 한다.
- 단계는 회차 전체에 하나라서, 한 로봇이 먼저 끝나도 다른 로봇이 Dock 에 올 때까지 다음 단계를 기다린다.

## 한 번에 띄우기 (PC 마다 명령 하나)

빌드(0절)를 한 뒤, **관제 PC 를 먼저** 띄운다.

```bash
# 관제 PC — robot4 한 대, 칸 맵은 파일
ros2 launch scv_bringup control_pc.launch.py robots:=robot4
# 관제 PC — 두 대, 칸 맵은 DB (같은 터미널에 SCV_DB_DSN 을 직접 export 해 둔다)
ros2 launch scv_bringup control_pc.launch.py backend:=db ap_ssid:=<핫스팟>

# 로봇 PC — 저장소 뿌리에서. localization · Nav2 · mission_executor · RSSI · 정밀 측정 서버 · 가짜 비전이 함께 뜬다
ros2 launch scv_bringup robot_pc.launch.py robot_ns:=robot4 iface:=<무선랜> ssid:=<핫스팟>

# 시작 (관제 PC)
ros2 service call /inspection/start std_srvs/srv/Trigger
```

- **로봇 PC launch 는 켜고 10초 뒤 `mission_executor` 가 스스로 undock 한다. 로봇이 움직인다.** 주변을 비우고 켠다.
- 순서는 localization → mission_executor(undock · 초기 pose) → Nav2(30초 뒤) 다. Dock 에 있는 채로 Nav2 를 먼저 띄우면
  bringup 이 중단된다 (10/12 주행 일지). `Managed nodes are active` 가 나올 때까지 15초쯤 더 기다린다.
- 로봇 PC launch 는 아직 실제 로봇에서 돌려 보지 못했다. 한 묶음으로 안 뜨면 2절처럼 터미널을 나눠 하나씩 띄운다.
- RSSI 스캔은 로봇 PC 마다 한 번 `sudo setcap cap_net_admin+ep /usr/sbin/iw` 가 필요하다 (scv_survey/README 「스캔 권한」).
- Discovery Server 를 쓰는 PC 에서 `ros2 topic list` 가 비어 보이면 `ROS_SUPER_CLIENT=True` 와 `--spin-time 8` 을 준다.
- 시험은 `python3 -m pytest -p no:launch_testing -p no:launch_ros src/scv_manager/test src/scv_survey/test -q` (87 passed).

아래 1–2절은 같은 것을 하나씩 띄우는 방법이다.

## 0. 모든 PC

```bash
git fetch && git checkout feature/control && git pull
cd ros2_ws && colcon build --packages-select scv_msgs scv_bringup scv_survey scv_manager scv_nav
source install/setup.bash
```

## 1. 관제 PC (VICTUS) — 로봇 PC 의 Nav2 보다 먼저

```bash
ros2 run scv_survey cell_map_writer          # 한 대만 시험: --ros-args -p sample_topics:="['/robot4/survey/sample']"
ros2 run scv_manager inspection_manager      # 한 대만 시험: --ros-args -p robot_ids:="['robot4']" -p zone_ids:="['Z1']"
ros2 topic echo /inspection/alerts           # 알림 보기
```

- `inspection_manager` 는 켜지자마자 `/{robot}/keepout_mask` 에 빈 마스크를 낸다. 주행의 keepout_layer 는
  이 마스크를 받아야 costmap 이 준비되므로 **관제를 먼저 켠다.**
- 칸 맵은 기본으로 `~/scv_cellmap/Z1.json`·`Z2.json` 에 쓴다. DB(관제 PC 의 PostgreSQL)에 쓰려면
  두 노드에 `-p backend:=db -p ap_ssid:=<핫스팟>`(writer) · `-p cellmap_backend:=db -p ap_ssid:=<핫스팟>`(manager)
  를 주고, 같은 터미널에 `SCV_DB_DSN` 이 있어야 한다. `ap_ssid` 는 rssi_scanner 의 대상 SSID 와 같은 값이다.
  DB 에는 testbed_v1 지도가 격자 0.5 m 로 올라가 활성이어야 한다.
  DB 를 열지 못해도 `inspection_manager` 는 살아서 빈 마스크를 내고, 순찰 시작만 거절한다 (알림 `DB_UNAVAILABLE`).
  db 쪽은 아직 실제 DB 로 돌려 보지 못했다.

## 2. 로봇 PC (MSI) — 로봇마다. 아래는 robot4, 다른 로봇은 robot3 으로 바꾼다

```bash
# 순서: (1) 의 localization → (2) mission_executor → (1) 의 Nav2.  Dock 에서 Nav2 를 먼저 띄우면 bringup 이 중단된다
# (1) localization · Nav2 — 저장소 뿌리에서, 반드시 이 launch 로 (params 안의 $(var namespace) 를 이 launch 가 바꿔 준다)
ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot4 map:=ros2_ws/src/scv_nav/maps/testbed_v1.yaml params:=ros2_ws/src/scv_nav/config/localization.yaml
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot4 params_file:=ros2_ws/src/scv_nav/config/nav2_rpp.yaml
# (2) 작업 실행
python3 ros2_ws/src/scv_nav/scripts/mission_executor.py --ros-args -r __ns:=/robot4
# (3) RSSI 측정 (rssi_scanner · manual_tagger · 미리보기 히트맵)
ros2 launch scv_survey laptop_test.launch.py pose_source:=tf robot_ns:=robot4 rviz:=false iface:=<무선랜> ssid:=<핫스팟>
# (4) 재측정용 30초 정밀 측정 서버 (임시)
ros2 run scv_survey precise_server --ros-args -r __ns:=/robot4
# (5) 가짜 비전 (설비 점검 요청에 2초 뒤 NORMAL)
python3 ros2_ws/src/scv_manager/test/mock_inspect_facility_server.py --ros-args -r __ns:=/robot4
```

- 주행의 `fake_servers.py` 는 같이 켜지 않는다. 가짜 `survey/precise` 가 (4) 와 이름이 겹친다.
- (4) 가 없으면 재측정 작업이 `NO_SERVER` 로 실패하고, (5) 가 없으면 설비 점검이 실패한다. 회차는 끝까지 간다.

## 3. 시작 · 멈춤 (관제 PC)

```bash
ros2 service call /inspection/start  std_srvs/srv/Trigger
ros2 service call /inspection/estop  std_srvs/srv/Trigger    # 두 로봇 작업 모두 취소
ros2 service call /inspection/resume std_srvs/srv/Trigger
```

## 4. 볼 것

| 단계 | 확인 |
| --- | --- |
| Nav2 를 띄운 뒤 | `ros2 topic info /robot4/keepout_mask -v` — 발행은 `inspection_manager` 하나, 구독은 global·local costmap 둘. costmap 이 `/robot4/global_costmap/keepout_mask` 에 붙어 있으면 설정이 틀린 것 |
| 시작 | 관제 로그에 두 로봇 `작업 전송` 이 함께 나온다 |
| 측정 | 칸(0.5 m) 한 줄을 1/4 · 3/4 높이로 두 번 (줄 간격 `survey_spacing_m` 0.25, 한 줄 건너 돌기). 지도의 벽에서 0.30 m(`obstacle_clearance_m`) 안에는 점을 두지 않는다. 모서리마다 작업이 끝나 로봇이 서고 방향을 맞춘 뒤 다음 줄로 간다(`corner_mode` angle — 주행 BT 지움 반경 0.15 와 짝). Z2 기준 점 45 · 작업 24 · 약 6분 |
| 칸 확정 | 관제 로그 `칸 맵 확정: Z1: N칸 (음영 a · 미검출 b · low c)` 가 로봇 PC 히트맵과 맞는지 |
| 재측정 | 로봇마다 최대 5칸(`max_remeasure_per_robot`). 알림 `CLEARED`(해제) · `REMAINS`(남음) |
| 마스크 | 관제 로그 `회피 칸 N개 [...]`. RViz 의 `/{robot}/global_costmap/costmap` 에 그 칸이 장애물로 보인다 |
| 설비 | SHT-01·SHT-02. 관측 자리가 회피 칸 안이면 `KEEPOUT_BLOCKED` 알림을 내고 건너뛴다 |
| 끝 | 알림 `RUN_COMPLETED`, 마스크가 다시 빈 것으로 |
| 끝난 뒤 | `python3 ros2_ws/src/scv_manager/scripts/cellmap_heatmap.py` — 관제가 확정한 칸 맵(`~/scv_cellmap/Z*.json`)을 지도 위 히트맵 PNG 로. 주행 중 실시간은 RViz 에 `/survey/heatmap`(MarkerArray, Durability Transient Local) 추가 |

## 알아 둘 것

- 샘플이 1개뿐인 칸(low)은 음영이어도 마스크에 넣지 않는다. 측정 속도가 빨라 low 가 많으면 음영 칸이 빠질 수 있다.
- Nav2 설정은 주행 팀의 운영 파일 `nav2_rpp.yaml`(RPP)이다. `robot_pc.launch.py` 의 기본값도 이것이고, `nav2.yaml`(MPPI)은 비교용이다 (`nav2_params:=nav2.yaml`).
- 마스크 설정(`nav2*.yaml` 의 keepout_layer)은 주행 팀의 것을 그대로 쓴다 (`map_topic: $(var namespace)/keepout_mask`, costmap 단위 `use_maximum: true`).
- keepout_layer 는 inflation 앞에 있어서 회피 칸 둘레 0.45 m 도 비용이 붙는다. 좁은 곳에서는 길이 막힐 수 있다.
- 로봇 없이 관제 PC 한 대로 흐름만 보려면 2절 대신
  `python3 src/scv_manager/test/mock_execute_task_server.py` 를 켠다 (가짜 샘플도 함께 낸다).
