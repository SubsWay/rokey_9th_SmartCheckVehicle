# 주행 설계 노트

2026-10-12 · 담당 hhj · 기준: `feature/driving` 브랜치 작업 트리 (`ros2_ws/src/scv_nav`)

주행 코드를 이해하고 설명하기 위해 정리한 내용이다. 각 코드가 **무엇을 하는지**, **왜 그 값인지(실험 근거)**, **리뷰에서 나올 질문과 답**, **스스로 알고 있는 약점** 순서로 정리했다.
측정값의 원본은 `MEASUREMENTS.md`, `PLAN.md`, `docs/daily/barlide/2026-10-1x.md`, `data/`에 있다.

> ⚠ 2026-10-12 기준으로 커밋하지 않은 변경이 있다(keepout 설정, `DOCK_PCT`, fake_manager 웨이포인트). 이 노트는 그 변경까지 반영한 상태를 설명한다. 리뷰 전에 커밋됐는지 확인할 것.

---

## 1. 전체 구조 한눈에 보기

```
관제 PC (inspection_manager)                      로봇 PC (/robot3, /robot4)
──────────────────────────                      ────────────────────────────────────────────
 {ns}/execute_task  (IF-11 액션)  ───────────▶  mission_executor.py ──▶ TurtleBot4Navigator ──▶ Nav2
 {ns}/status        (IF-12, 1 Hz) ◀───────────        │                  (nav2_simple_commander)   │
 {ns}/keepout_mask  (IF-10)       ───────────────────────────────────────────────▶ costmap keepout_layer
                                                      │
                     비전팀  {ns}/inspect_facility ◀──┤ (FACILITY_CHECK)
                     RSSI팀  {ns}/survey/precise   ◀──┘ (REINSPECTION)

Nav2 안쪽: bt_navigator(navigate_through_poses_scv.xml) → planner → controller_server(RPP)
           → velocity_smoother(0.2 m/s) → collision_monitor → cmd_vel → Create 3
위치 추정: AMCL (localization.yaml) + 지도 maps/testbed_v1
```

- **mission_executor 는 속도를 직접 내지 않는다.** 경로 추종은 전부 Nav2 가 한다. mission_executor 는 "어디로 갈지, 언제 멈출지, 실패를 어떻게 보고할지"만 정한다. (예외: 취소 시 정지 명령 `stop()`, 자세 보정용 `spin`/`driveOnHeading`)
- 실행 순서: **localization → mission_executor(undock·초기 pose) → nav2**. nav2 를 초기 pose 없이 먼저 띄우면 map TF 를 60초 기다리다 bringup 이 실패한다(10/11 시행착오).

## 2. 파일 지도

| 파일 | 역할 | 운영/측정 |
| --- | --- | --- |
| `scripts/mission_executor.py` (318줄) | IF-11 액션 서버, IF-12 상태 발행, 작업 4종 실행, 예외 E1·E2 | **운영 코드** |
| `config/nav2_rpp.yaml` | 실제로 쓰는 Nav2 설정 (컨트롤러 RPP) | **운영 설정** |
| `config/nav2.yaml` | 비교용 Nav2 설정 (컨트롤러 MPPI, TurtleBot4 기본) | 3-2 비교 기준선 |
| `config/localization.yaml` | AMCL 설정 (튜닝 3개) | **운영 설정** |
| `config/navigate_through_poses_scv.xml` | Nav2 기본 BT 복사본, `RemovePassedGoals radius` 만 0.7 → 0.35 | **운영 설정** |
| `scripts/course_run.py` | 고정 코스(약 7.2 m, 코너 3개) 한 번 주행 | 3-2 측정 |
| `scripts/track_check.py` | 경로 이탈(cross-track) 평균·최대 기록 | 3-2 측정 |
| `scripts/tol_check.py` | goal 허용 오차 단계 시험 | 3-4 측정 |
| `scripts/pose_check.py` | AMCL 위치·표준편차 출력/기록 | 2-1~2-3 측정 |
| `scripts/fake_manager.py` | 가짜 관제 (execute_task 한 건 보내기·취소) | 5-1 시험 |
| `scripts/fake_servers.py` | 가짜 비전·RSSI 서버, 시험용 keepout 마스크 | 5-2·4-1 시험 |
| `scripts/latency_check.py`, `odom_check.py`, `plot_latency.py` | Wi-Fi 지연, odom 오차 | 1단계 측정 |

스크립트는 `setup.py` 의 `console_scripts` 에 등록하지 않고 `python3 scripts/xxx.py --ros-args -r __ns:=/robot3` 로 직접 실행한다. (설치 대상은 BT xml 하나뿐.) → "왜 entry point 가 없나?" 질문이 나오면: 개발 중 빌드 없이 바로 고쳐 돌리기 위해서, 통합 단계에서 등록 예정.

---

## 3. mission_executor.py 따라 읽기

### 3-1. 시작 (`main()`)

1. `TurtleBot4Navigator()` 생성. 이 객체는 **자기 스스로 spin** 하므로 executor 에 넣지 않는다.
2. 시작 위치(home) 결정: `-p x y yaw` 셋 다 주면 그 값, 아니면 namespace 별 표 `HOME`.
   - `HOME` 은 2-1 에서 undock 직후 위치를 3회 측정한 평균 (robot3 차이 2.1 cm/1.8 cm/0.35°).
   - 이유: **Dock 에서는 라이다가 꺼져** AMCL 이 위치를 잡을 수 없다(1-1 측정, scan 0 Hz). 그래서 undock 직후 자리를 "알려진 초기 pose"로 쓴다.
3. Dock 에 있으면 `undock()` → `setInitialPose(home)` → `waitUntilNav2Active()`.
   - undock 상태로 시작해도 `setInitialPose` 를 한다. 안 하면 `waitUntilNav2Active()` 가 amcl_pose 를 받을 때까지 **(0,0) 을 초기 pose 로 보낸다**(10/12 버그 수정).
4. 노드는 `MultiThreadedExecutor` + 별도 스레드로 spin, 메인 스레드는 0.5초마다 **놀고 있을 때 배터리 ≤ `DOCK_PCT`(20 %) 면 스스로 dock**.
5. `SignalHandlerOptions.NO`: Ctrl+C 뒤에도 `cancelTask()`·`stop()` 을 보낼 수 있게 rclpy 기본 시그널 처리를 끔.

### 3-2. 작업 수락 (`on_goal`)

- `busy` 면 거절 → **한 번에 한 작업**. 관제가 이전 작업을 취소하고 새로 보내는 규칙(IF-11).
- 배터리 ≤ 30 % 면 DOCK 외 거절 (E2).
- `busy = True` 를 **수락 콜백에서** 잡는 이유: 수락과 `execute` 시작 사이에 다른 goal 이 들어오는 틈을 막음.
- 콜백 그룹이 `ReentrantCallbackGroup` 인 이유: `execute` 가 오래 도는 동안에도 취소 요청·다른 액션 피드백·파라미터 서비스 응답을 받아야 해서.

### 3-3. 작업 종류별 흐름 (`execute`)

| type | 흐름 |
| --- | --- |
| `SURVEY_SEGMENT` | `follow(waypoints)` — navigate_through_poses 로 통과 |
| `FACILITY_CHECK` | `follow(precise=True)` → `inspect()` (비전 액션, 자세 보정 피드백을 회전·전진으로 따름) |
| `REINSPECTION` | `follow(precise=True)` → `measure()` (RSSI precise 30초) |
| `DOCK` | `go_dock(waypoints)` — 웨이포인트(없으면 home)로 가서 `dock()` |

- Dock 상태에서 DOCK 외 작업을 받으면 먼저 undock + 초기 pose.
- 결과는 `succeed / abort / canceled` 중 하나와 `Result(ok, reason)`.
- `reason == 'LOW_BATTERY'` 면 그 자리에서 `go_dock` 으로 돌아감.
- `finally` 에서 `busy=False`, 상태를 IDLE 로 (DOCKED 면 유지).

### 3-4. 핵심: `follow_loop` (경로 통과 + 막힘 E1)

```
goThroughPoses(남은 waypoints)
while 작업이 안 끝남:
    취소 요청 or 배터리 ≤ DOCK_PCT → cancelTask, 정지, 반환
    feedback 으로 진행률(i/n) 계산·발행
    5 cm 이상 움직였으면 → 시각 갱신 (BLOCKED 였으면 EXECUTING 으로 복귀)
    10초 동안 5 cm 못 움직임 → break (지금 향하던 waypoint 건너뜀)
    5초 동안 5 cm 못 움직임 → 상태 BLOCKED (Nav2 가 재계획하는 중)
    0.2초 쉼
(break 로 나오면) cancelTask → 향하던 waypoint 다음부터 다시 goThroughPoses
```

- **5초/10초 규칙의 근거**: 요구사항 BR-T10 "막힘 시 10초 이내 우회·건너뛰기". 5초는 Nav2 가 재계획·회복하는 동안 관제에 "막혔다"를 먼저 알리는 경고 단계.
- 지금 향하던 waypoint 위치 `k = len(waypoints) - number_of_poses_remaining`.
- 건너뛴 게 하나라도 있으면 끝까지 가도 `ok=False, reason='BLOCKED skipped=n'` → 관제가 그 칸을 재측정 대상으로 판단할 수 있게.
- `while … else:` 문법: 루프가 `break` 없이 끝났을 때(= Nav2 작업 완료)만 `else` 가 실행된다. **리뷰에서 낯설다는 말이 나올 수 있으니 설명할 준비.**

### 3-5. 정밀 도착 (`set_tolerance`)

- 정밀 지점은 **BT 를 새로 만들지 않고 goal checker 파라미터만 바꾼다**: `general_goal_checker.xy_goal_tolerance / yaw_goal_tolerance` 를 `PRECISE_TOL (0.03 m, 0.052 rad=3°)` 로 → 도착 후 `finally` 에서 `NORMAL_TOL (0.05 m, 0.087 rad=5°)` 로 원복.
- `SetParameters` 서비스로 `controller_server` 에 보냄 (`ros2 param set` 과 같음). SimpleGoalChecker 는 실행 중 파라미터 변경을 받는다.
- 장점: BT·플러그인 추가 없음, 노드 수 그대로. 단점은 6장 약점 참고.

### 3-6. 다른 팀 액션 호출 (`call_action`, `inspect`, `do_adjust`)

- `call_action`: 서버 5초 대기 → goal 전송 → 결과 대기. 그 사이 **작업이 취소되면 하위 액션도 취소**.
- 비전 피드백(자세 보정)은 executor 스레드의 콜백에서 오므로 **리스트에 쌓기만 하고**, 실제 `nav.spin`/`driveOnHeading` 은 작업 스레드(`each=do_adjust`)에서 실행. 이유: nav 는 자기 스스로 spin 하는 별도 노드라 콜백 안에서 블로킹 호출하면 꼬임.
- 전진만 하고 **후진은 하지 않는다** — Create 3 는 뒤쪽 낭떠러지 센서가 없어 후진 명령을 막음(10/11 MPPI 후진 문제와 같은 이유).
- 시설 종류는 `target_ref` 앞 세 글자(`EXT`/`SHT`/`EXL`)로 정함 → 코드에 `shortcut:` 주석으로 남긴 임시 방식. 관제가 kind 를 보내 주면 바꿈.

### 3-7. 상태 발행 (`publish_status`, IF-12)

- 1 Hz. `state, battery_percent, accepting_tasks(배터리 > 30 %), progress, warning, task_id, x/y/yaw, pose_ok`.
- `pose_ok = max(std_x, std_y) ≤ pose_std_max(0.15 m)` — RSSI 팀이 false 샘플을 버림.
- 0.15 m 근거(2-3): 정상 9.2–11.4 cm, 일부러 틀린 위치 52–53 cm. 기존 0.10 이면 정상 5개 중 4개가 false.
- 공분산 `max(…, 0.0)`: `setInitialPose` 가 공분산 0 을 보내 AMCL 이 −1e-13 같은 음수를 내고 `sqrt` 에서 스레드가 죽었던 버그(10/11) 방지.

---

## 4. 설정값과 근거 (리뷰에서 "이 값 왜?" 나올 때)

### 4-1. 컨트롤러: RPP 선택 (3-2)

| 설정 | 시간 | 이탈 평균 | 이탈 최대 |
| --- | --- | --- | --- |
| MPPI, 지움 거리 0.7 (robot4 5회) | 43.9초 | 12.2 cm | 38.1 cm |
| RPP, 지움 거리 0.7 (robot4 5회) | 39.2초 | 10.0 cm | 35.2 cm |
| MPPI, 지움 거리 0.35 (robot3 3회) | 47.5초 | 10.3 cm | 27.2 cm |
| RPP lookahead 0.6, 0.35 (robot3 3회) | 41.7초 | 6.6 cm | 19.7 cm |
| **RPP lookahead 0.4, 0.35 (robot3 3회)** | **44.2초** | **4.5 cm** | **16.6 cm** |

데이터 `data/2026-10-12_track_check.csv`. 그래프 `docs/daily/barlide/img/2026-10-12_course_trajectory.png`, `2026-10-12_deviation_compare.png`.

- **왜 MPPI 와 RPP 둘만?** MPPI 는 TurtleBot4 Jazzy 기본(기준선), RPP 는 경로 추종 전용·파라미터 적음. DWB 는 이전 세대·critic 튜닝 많음, Graceful 은 정밀 도착용, Rotation Shim 은 래퍼라 제외.
- **MPPI 약점**: 목표가 가까이 등 뒤에 있으면 후진 금지 때문에 못 돌아서고 머뭇거림. RPP 는 제자리 회전 후 진입.
- 목표치(평균 ≤ 5 cm)는 달성, **최대 ≤ 10 cm 는 아직 미달(16.6 cm, 코너)** → 3-3 튜닝에서 계속. 리뷰에서 솔직히 말할 것.

### 4-2. `nav2_rpp.yaml` 에서 바꾼 값

| 키 | 원본 → 지금 | 이유 |
| --- | --- | --- |
| `desired_linear_vel` | 0.5 → **0.2** | 측정 속도 0-3 (RSSI 측정용 저속) |
| `velocity_smoother.max_velocity` | 0.26 → **0.2** | 컨트롤러 계획 속도와 smoother 상한을 맞춰야 추종 오차가 안 커짐 |
| `lookahead_dist` | 0.6 → **0.4** | 코너 질러가기 줄임 (최대 이탈 19.7 → 16.6 cm) |
| `rotate_to_heading_angular_vel` | 1.0 → **0.5** | 도착 회전 오버슈트. 방향 오차 평균 3.9° → 2.9° |
| `xy_goal_tolerance` / `yaw_goal_tolerance` | 0.25 / 0.25 → **0.05 / 0.087** | 3-4 시험 (아래) |
| `progress_checker.plugin` | Simple → **PoseProgressChecker** | 제자리 회전도 진척으로 인정. 도착 후 큰 회전 중 "Failed to make progress" 로 끊기던 문제 |
| `required_movement_radius` | 0.5 → **0.15** | 0.2 m/s 저속·짧은 이동에서 "10초에 0.5 m" 를 못 맞춤 |
| `default_nav_through_poses_bt_xml` | 기본 → **scv xml (지움 거리 0.35)** | 아래 4-3 |
| `keepout_layer` (global·local) | 추가, `enabled: true` | IF-10 RSSI 음영 마스크. 점유(100) 칸 = 통과 불가 |
| `use_maximum: true` | 플러그인 안 → **costmap 단위로 이동** | StaticLayer 는 costmap 단위 값을 읽음. 플러그인 안에 두면 무시되어 마스크의 빈 칸이 지도 벽을 지움 (10/12) |
| `keepout map_topic` | `keepout_mask` → `$(var namespace)/keepout_mask` | 상대 이름이면 `{ns}/global_costmap/keepout_mask` 로 붙음 |

- **keepout 은 KeepoutFilter 가 아니라 StaticLayer 한 층**: 노드(map_server·costmap_filter_info) 추가 없이 토픽 하나만 구독. 조건: 마스크가 지도와 원점·해상도·크기가 같아야 함.
- `enabled: true` 일 때 **마스크 발행자가 없으면 costmap 이 준비 안 됨 → 모든 경로 계획 실패**(10/11). 그래서 관제가 시작부터 빈 마스크를 내도록 interfaces v0.2.8 에 합의.

### 4-3. BT: waypoint 지움 거리 0.35 m

- Nav2 기본 BT 는 waypoint **0.7 m 안**에 들어오면 지우고 3초마다 재계획 → 코너를 0.7 m 앞에서 버리고 안쪽으로 질러감(최대 이탈 35–38 cm 의 원인).
- 0.2 m 로 줄이면 RPP 가 코너에 0.21 m 까지만 접근해 **영영 못 지움** → 첫 코너로 되돌아가 왕복 반복(실패).
- 그래서 0.35 m. 파일은 기본 BT 복사본에서 그 한 줄만 바꿈(주석으로 표시).

### 4-4. goal 허용 오차 (3-4)

`tol_check.py` 로 A↔B 왕복, 단계마다 6회. 도착 yaw 는 진행 방향 + 90° (매번 "전진 → 제자리 90° 회전" 이 정상).

| 허용 오차 (회전 1.0 rad/s) | 위치 오차 | 방향 오차 | 수정 0회 |
| --- | --- | --- | --- |
| 0.03 m / 3° | 0.6–2.0 cm | 0.5–6.3° | 6/6 |
| 0.05 m / 5° | 0.7–3.6 cm | 3.8–9.5° | 5/6 |
| 0.08 m / 8° | 5.2–6.8 cm | 7.0–12.9° | 6/6 |
| 0.10 m / 10° | 6.8–8.8 cm | 8.9–16.4° | 6/6 |
| 0.03 m / 3°, **회전 0.5 rad/s** | 0.2–3.0 cm | 1.1–4.5° (평균 2.9°) | 5/6 |

- **로봇은 허용 오차 경계에 들어오는 순간 멈춘다** → 0.10 이면 매번 7–9 cm 앞에서 섬. 줄여도 수정 횟수가 안 늘고 1초 정도만 더 걸려서 조임.
- 확정: 일반 0.05 m/5°, 정밀 0.03 m/3°, 회전 0.5 rad/s. 목표치(일반 ≤ 10 cm/10°, 정밀 ≤ 5 cm/5°) 안쪽.
- 한계: 오차는 **AMCL 기준**(Nav2 feedback). 바닥 테이프 실측은 안 함.
- 정지 흔들림(robot4, 30회): x std 0.97 cm, y 1.83 cm, yaw 0.50° → 정밀 3 cm 는 흔들림보다 충분히 큼.

### 4-5. AMCL (`localization.yaml`, 2-2)

| 키 | 원본 → 지금 | 이유 |
| --- | --- | --- |
| `update_min_d` | 0.25 → **0.10** | 0.2 m/s 저속이라 갱신이 너무 드묾 |
| `update_min_a` | 0.2 → **0.1** | 같은 이유 |
| `max_beams` | 60 → **120** | scan 한 장에서 벽 정합 단서 증가 |

재방문 오차 평균: 기본 5.4 cm → ① 3.5 cm → ①+② **2.2 cm** (목표 ≤ 10 cm). 한계: 점당 1회, 줄자 오차 ±1 cm 라 ①·② 차이는 잡음 범위.

### 4-6. 그 밖에 리뷰에서 물을 수 있는 값

| 값 | 근거 |
| --- | --- |
| 속도 0.2 m/s | 0-3 측정 속도(RSSI 측정용). 3-5 에서 RSSI 팀과 최종 확정 예정 |
| 배터리 30 % / 20 % | E2. 30 % 이하 새 작업 거절, `DOCK_PCT` 20 % 이하 스스로 dock (요구서는 15 %, 10/12 hhj 요청으로 20) |
| `pose_std_max` 0.15 m | 2-3, 위 3-7 |
| MPPI `vx_min` 0.0 (nav2.yaml) | Create 3 후진 차단 → MPPI 가 후진을 고르면 명령은 나가도 안 움직여 2분 대기 (10/11) |
| cmd_vel 끊김 | Create 3 가 약 0.9초 뒤 스스로 정지, 그동안 약 13.6 cm 이동 (1-3) |
| Wi-Fi 지연 | odom 수신 약 50 ms, cmd_vel→odom p95 218 ms → 기준 p95 ≤ 250 ms (1-2) |

---

## 5. 측정 스크립트 요점

- **course_run.py**: `waitUntilNav2Active(localizer='robot_localization')` 로 부름. 이름과 달리 AMCL 을 쓰지만, `'amcl'` 로 주면 amcl_pose 받을 때까지 (0,0) 초기 pose 를 보내 이미 잡힌 위치를 덮어써서 일부러 이렇게 함. Ctrl+C 시 `cancelTask()` 로 goal 취소(전에는 스크립트만 꺼지고 로봇은 계속 감).
- **track_check.py**: Nav2 **첫 전역 경로**를 기준선으로 로봇(map→base_link)과의 최단 거리를 10 Hz 로 기록. AMCL 기준이라 "컨트롤러가 경로를 얼마나 잘 따랐나"를 보는 값이지 바닥 절대 오차가 아님.
- **tol_check.py**: 목표 0.3 m 안에서 움직임 종류(F/L/R) 구간 수 − 2 = 수정 횟수 ("F→회전" 한 번은 정상). 끝나면 `finally` 에서 허용 오차를 `NORMAL` 로 되돌림.
- **fake_manager.py / fake_servers.py**: 다른 팀 노드 없이 5단계 흐름 시험. 4-2 막힘 시험은 가운데 웨이포인트 칸을 keepout 마스크로 막아서 함.

---

## 6. 리뷰에서 지적될 수 있는 약점 (먼저 알고 가기)

코드를 고치지 않고 정리만 했다. 리뷰에서 나오면 "알고 있고, 이렇게 할 계획"으로 답하면 된다.

| # | 위치 | 내용 | 영향 / 답변 |
| --- | --- | --- | --- |
| 1 | `go_dock` | ~~`nav.dock()` 결과를 확인하지 않고 바로 `STATE_DOCKED` 로 바꿈~~ **10/12 수정**: dock 뒤 `getDockedStatus()` 로 확인, 실패면 IDLE + `DOCK_FAILED` | dock 상태 토픽이 늦게 바뀌면 성공을 실패로 볼 수 있음 → 실기 확인 필요 |
| 2 | `set_tolerance` | ~~응답을 안 봐서 실패해도 일반 허용 오차로 정밀 도착을 계속함~~ **10/12 수정**: 서비스 없음·`successful=False` 면 출발하지 않고 `TOLERANCE_FAILED` | 응답만 확인하고 값을 다시 읽지는 않음 |
| 3 | `go_home`, `do_adjust` → `wait_nav` | 이 대기 중에는 취소·배터리 확인을 안 함 | home 복귀·자세 보정(짧은 동작) 중 취소가 늦게 먹힘 |
| 4 | `busy` 플래그 | 메인 스레드(자동 dock)와 액션 콜백이 lock 없이 같이 씀 | 이론상 경쟁 조건. 다만 자동 dock 은 배터리 ≤ 20 % 일 때뿐이고 그때 on_goal 은 DOCK 만 받음 → 실제로 겹칠 일은 드묾 |
| 5 | `publish_status` | `accepting_tasks` 가 배터리만 보고 `busy` 는 안 봄 | 작업 중에도 true 로 보임. 관제가 state 로 구분하면 문제 없음 |
| 6 | `NORMAL_TOL` | `nav2.yaml`(MPPI)의 허용 오차는 0.10/0.17 인데 정밀 도착 후 0.05/0.087 로 "원복" | MPPI 설정으로 돌릴 때만 값이 바뀜. RPP 가 운영 설정이라 영향 없음 |
| 7 | `KIND` | `target_ref` 앞 세 글자로 시설 종류 추정 | `shortcut:` 주석으로 남김. 관제가 kind 를 보내면 교체 |
| 8 | 경로 이탈 최대 | 16.6 cm, 목표 10 cm 미달 (코너) | 3-3 튜닝 진행 중 |
| 9 | 측정 기준 | 3-2·3-4 오차는 AMCL 기준, 바닥 실측 아님 | 6-3 정확도 회귀 시험에서 테이프 실측 예정 |
| 10 | 측정 횟수 | 3-2 robot3 3회씩(계획 5회), 2-2 점당 1회 | 시간 부족. 추가 측정 계획 있음 |
| 11 | 테스트 | 단위 테스트 없음, 실기 측정으로만 검증 | ROS 액션·Nav2 의존이 커서 fake_manager·fake_servers 로 흐름 시험 |

## 7. 예상 질문과 짧은 답

- **Q. 왜 BT 를 고치지 않고 파라미터로 정밀 도착을 하나?** → 바뀌는 건 허용 오차 두 값뿐이라 BT·플러그인 추가 없이 `SetParameters` 한 번이면 됨. 도착 후 `finally` 로 반드시 원복.
- **Q. 막힘 판단을 Nav2 progress checker 에 맡기지 않고 직접 하는 이유?** → progress checker 는 컨트롤러 단위 실패만 알려 주고, 요구사항(BR-T10)은 "10초 안에 건너뛰기 + 관제에 BLOCKED 알림"이라 작업 단위 판단이 필요. progress checker 는 회전 중에도 진척으로 보게 완화했고(PoseProgressChecker), 회전만 하며 갇힌 경우는 E1(5 cm/10초)이 잡음.
- **Q. 왜 `navigate_through_poses` 인가 (`followWaypoints` 아님)?** → waypoint 마다 멈추지 않고 이어서 지나가야 측정 속도가 일정함(BR-T02).
- **Q. undock 위치를 하드코딩해도 되나?** → Dock 에서는 라이다가 꺼져 AMCL 이 위치를 못 잡음. 3회 측정 평균이고 차이 3 cm 이내. 다른 자리에서 켤 땐 `-p x y yaw`.
- **Q. 스레드를 왜 이렇게 나눴나?** → nav(TurtleBot4Navigator)는 자기 spin, 노드는 MultiThreadedExecutor 스레드, 작업 실행은 액션 execute 스레드. 콜백 안에서 nav 블로킹 호출을 하지 않기 위해 자세 보정은 리스트로 넘김.
- **Q. 0.2 m/s 로 느린데 성능 문제는?** → 속도는 RSSI 측정 조건이 정함. 코스 7.2 m 를 44초.
- **Q. 후진을 왜 막았나?** → Create 3 는 뒤쪽 낭떠러지 센서가 없어 후진 명령을 막음. 후진을 고르면 명령만 나가고 안 움직여 2분씩 대기했음.

## 8. 남은 확인 사항

- [ ] 커밋 안 된 변경(keepout `use_maximum`·`map_topic`, `DOCK_PCT`, fake_manager 웨이포인트)이 커밋됐는지
- [ ] `nav2_rpp.yaml` 맨 위 주석 "값은 Nav2 문서 기본값, 속도만 0.2" 는 지금 사실과 다름(lookahead·회전·허용 오차·progress checker 도 바뀜) → 고칠지 결정
- [x] 6장 약점 1·2번 수정 (10/12, 실기 확인 전)
