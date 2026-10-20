# mission_executor 주행 흐름

2026-10-12 · 기준 코드 `feature/driving` fd23581 `ros2_ws/src/scv_nav/scripts/mission_executor.py` · 인터페이스 `docs/interfaces.md` v0.2.11

주행에 관한 부분만 그렸습니다. 설비 점검(IF-05)·재측정(IF-04)은 "도착 뒤 다른 노드 액션 호출"까지만 나옵니다. `{ns}` 는 `/robot4`·`/robot3` 입니다.

## 1. 외부 노드 ↔ mission_executor

```mermaid
flowchart LR
    subgraph CTRL["관제 PC"]
        IM["inspection_manager"]
        RB["ros_bridge"]
    end
    subgraph RPC["로봇 PC"]
        ME(["mission_executor"])
        NAV["Nav2<br/>bt_navigator · planner_server<br/>controller_server · behavior_server"]
        AMCL["amcl"]
        DA["detection_alert"]
        RS["rssi_scanner"]
    end
    subgraph RPI["RPi · Create 3"]
        C3["Create 3"]
    end

    IM -- "IF-11 action execute_task<br/>Goal: task_id · run_id · type · waypoints · target_ref" --> ME
    ME -- "Feedback: waypoint_index · progress<br/>Result: ok · reason" --> IM
    ME -- "IF-12 topic status 1 Hz<br/>state · battery_percent · accepting_tasks<br/>task_id · run_id · task_type · progress<br/>x · y · yaw · pose_ok · warning" --> IM
    ME -- "IF-12 status" --> RB
    IM -. "IF-10 topic keepout_mask<br/>(OccupancyGrid, transient_local)" .-> NAV

    C3 -- "topic battery_state<br/>percentage 0–1" --> ME
    C3 -- "topic dock_status<br/>is_docked" --> ME
    ME -- "IF-06 action dock · undock" --> C3
    ME -- "topic cmd_vel (TwistStamped 0)<br/>취소 때 정지" --> C3

    AMCL -- "topic amcl_pose<br/>position · orientation · covariance" --> ME
    ME -- "topic initialpose<br/>HOME 자세 · 공분산 0.15 m, 10°<br/>(amcl_pose 올 때까지 반복)" --> AMCL

    ME -- "action navigate_through_poses (waypoints)<br/>navigate_to_pose (HOME)<br/>compute_path_to_pose (길 확인)<br/>spin (시작 ±15° · 자세 보정) · drive_on_heading (자세 보정)" --> NAV
    NAV -- "Feedback: number_of_poses_remaining · current_pose<br/>Result: SUCCEEDED · FAILED · CANCELED" --> ME
    ME -- "service controller_server/set_parameters<br/>general_goal_checker.xy · yaw_goal_tolerance" --> NAV

    ME -- "IF-05 action inspect_facility<br/>task_id · facility_id · kind" --> DA
    DA -- "Feedback: adjust_yaw · adjust_forward_m<br/>Result: verdict · detail" --> ME
    ME -- "IF-04 action survey/precise<br/>duration_s=30" --> RS
    RS -- "Result: rssi_median · rssi_iqr · n_samples" --> ME
```

- keepout_mask 는 mission_executor 를 거치지 않고 Nav2 costmap 의 keepout_layer 로 바로 들어갑니다 (점선).
- `dock`·`undock`·`dock_status`·`initialpose` 와 Nav2 액션은 `TurtleBot4Navigator`(`nav`)가 대신 주고받습니다. initialpose 는 `localize()` 가 `nav` 의 publisher 로 직접 보냅니다 (setInitialPose 는 공분산 0 이라 AMCL 이 몇 도 틀린 것을 못 고침).

## 2. 시작 (main)

```mermaid
flowchart TD
    A["시작"] --> B{"-p x y yaw 있음?"}
    B -- 예 --> D["그 값을 HOME 으로"]
    B -- 아니오 --> C{"HOME 표에 ns 있음?"}
    C -- 아니오 --> X["종료: -p x y yaw 를 주세요"]
    C -- 예 --> D2["HOME 표 값<br/>robot3 2.806, 4.610, 179.9°<br/>robot4 2.863, 0.948, 175.8°"]
    D --> E{"dock_status.is_docked?"}
    D2 --> E
    E -- 예 --> F["undock 액션"]
    E -- 아니오 --> G
    F --> G["localize: initialpose ← HOME<br/>공분산 0.15 m · 10°, amcl_pose 올 때까지 반복"]
    G --> H["Nav2 활성화 대기<br/>(localizer=robot_localization: (0,0) 다시 안 보냄)"]
    H --> H2["settle: 제자리 +15° → −15° spin<br/>AMCL 이 라이다로 방향 맞춤"]
    H2 --> I["IF-11 액션 서버 · IF-12 1 Hz 타이머 시작"]
    I --> J{"0.5초마다: 놀고 있고<br/>battery ≤ 20 %이고 docked 아님?"}
    J -- 예 --> K["go_dock (waypoints 없음 → HOME)"]
    K --> J
    J -- 아니오 --> J
```

## 3. 작업 수락·실행 (on_goal · execute)

```mermaid
flowchart TD
    G0["IF-11 Goal 도착<br/>(inspection_manager)"] --> A{"busy?<br/>또는 battery ≤ 30 %이고 type ≠ DOCK?"}
    A -- 예 --> R["REJECT"]
    A -- 아니오 --> B["ACCEPT · busy=True"]
    B --> C{"state = DOCKED이고 type ≠ DOCK?"}
    C -- 예 --> C1["undock → localize(HOME) → settle(±15°)"]
    C -- 아니오 --> D
    C1 --> D["state = EXECUTING"]
    D --> T{"type"}
    T -- "0 SURVEY_SEGMENT" --> S0["follow(waypoints)"]
    T -- "1 FACILITY_CHECK" --> S1["follow(waypoints, 정밀)"] --> S1b{"ok?"}
    S1b -- 예 --> S1c["IF-05 inspect_facility<br/>피드백 오면 spin · drive_on_heading(앞으로만)<br/>verdict ≠ UNCONFIRMED 면 ok"]
    T -- "2 REINSPECTION" --> S2["follow(waypoints, 정밀)"] --> S2b{"ok?"}
    S2b -- 예 --> S2c["IF-04 survey/precise 30초<br/>결과는 reason 문자열로만"]
    T -- "3 DOCK" --> S3["go_dock(waypoints)"]
    T -- 그 밖 --> S4["UNKNOWN_TYPE"]
    S0 & S1b & S1c & S2b & S2c & S3 & S4 --> L{"reason = LOW_BATTERY?"}
    L -- 예 --> L1["go_dock (HOME)"]
    L -- 아니오 --> F
    L1 --> F["busy=False · task 비움<br/>DOCKED 아니면 state = IDLE"]
    F --> Z{"취소 요청?"}
    Z -- 예 --> Z1["canceled"]
    Z -- 아니오 --> Z2{"ok?"}
    Z2 -- 예 --> Z3["succeed"]
    Z2 -- 아니오 --> Z4["abort"]
    Z1 & Z3 & Z4 --> RES["Result: ok · reason<br/>(rssi_* 는 0 그대로)"]
```

- 정밀 도착 `follow(..., 정밀)`: 출발 전 set_parameters 로 허용 오차를 0.03 m · 0.052 rad 로 바꾸고, 끝나면 0.05 m · 0.087 rad 로 되돌립니다. 바꾸지 못하면 출발하지 않고 `TOLERANCE_FAILED`.
- waypoints 가 비면 `NO_WAYPOINTS` (DOCK 은 예외: HOME 으로 감).

## 4. 주행·막힘 (follow_loop, E1·E2)

```mermaid
flowchart TD
    A["left = 길 있는 waypoint 만<br/>(compute_path_to_pose: 지금 위치→첫 점, 앞 점→다음 점)"] --> B{"left 남음?"}
    B -- 아니오 --> Z0["실패: BLOCKED skipped=n"]
    B -- 예 --> C["navigate_through_poses(left)"]
    C --> D{"Nav2 끝남?"}
    D -- 아니오 --> E{"IF-11 취소 요청<br/>또는 battery ≤ 20 %?"}
    E -- 예 --> E1["cancelTask · cmd_vel 0 ×5<br/>CANCELED / LOW_BATTERY"]
    E -- 아니오 --> F["Nav2 feedback → IF-11 feedback<br/>waypoint_index · progress"]
    F --> G{"5 cm 또는 0.3 rad 움직임?"}
    G -- 예 --> G1["타이머 리셋<br/>BLOCKED 였으면 EXECUTING"] --> W
    G -- 아니오 --> H{"멈춘 지 10초?"}
    H -- 예 --> K
    H -- 아니오 --> I{"5초이고 아직 BLOCKED 아님?"}
    I -- 아니오 --> W["0.2초 대기"] --> D
    I -- 예 --> I1["state = BLOCKED · warning = BLOCKED<br/>(IF-12 로 관제에 보임)"] --> I2{"남은 점 중 길 없는 점?"}
    I2 -- 예 --> K
    I2 -- 아니오 --> W
    D -- 예 --> R{"결과"}
    R -- SUCCEEDED --> R1{"건너뛴 점 있음?"}
    R1 -- 아니오 --> OK["성공"]
    R1 -- 예 --> Z1["실패: BLOCKED skipped=n"]
    R -- FAILED --> K
    R -- 그 밖 --> Z2["실패: 결과 문자열"]
    K["cancelTask · 지난 점 버림<br/>길 없는 점 빼기, 없으면 지금 향하던 점 하나 빼기<br/>state = EXECUTING"] --> B
```

## 5. 복귀 (go_dock)

```mermaid
flowchart TD
    A{"dock_status.is_docked?"} -- 예 --> A1["state = DOCKED · 성공"]
    A -- 아니오 --> B{"waypoints 있음?"}
    B -- 예 --> C["follow(waypoints)"]
    B -- 아니오 --> D["navigate_to_pose<br/>HOME 에서 dock 반대쪽으로 0.3 m (DOCK_BACKOFF)"]
    C & D --> E{"ok?"}
    E -- 아니오 --> E1["실패 (그 사유)"]
    E -- 예 --> F["state = DOCKING · dock 액션"]
    F --> G{"is_docked?"}
    G -- 예 --> G1["state = DOCKED · 성공"]
    G -- 아니오 --> G2["state = IDLE · DOCK_FAILED"]
```

## 6. artifact 「1 AMR · mission_executor」 와 비교

artifact 는 807b7e9 기준이라 그 뒤(8034dfd·fd23581) 바뀐 곳이 틀립니다.

| 항목 | artifact | 지금 코드 (fd23581) |
| --- | --- | --- |
| 스스로 dock 하는 배터리 | 15 % | **20 %** (`DOCK_PCT`) |
| 허용 오차 | 정밀 0.05 m/0.087 rad, 평소 0.10/0.17 | **정밀 0.03/0.052, 평소 0.05/0.087**. 못 바꾸면 `TOLERANCE_FAILED` |
| 막힘 E1 | 5초에 그 점 건너뜀, 10초에 남은 점 모두 | 5초 BLOCKED(회전 0.3 rad 도 움직임으로 셈). 길 없는 점은 바로 뺌. 10초·Nav2 실패면 길 없는 점, 없으면 지금 향하던 점 **하나만** 뺌. 출발 전에도 길 없는 점을 뺌 |
| DOCK | HOME 으로 가서 dock | waypoints 있으면 그 경로, 없으면 HOME 에서 0.3 m 더 나간 자리. dock 뒤 안 붙었으면 `DOCK_FAILED` |
| 초기 위치 | initialpose 설정 | `localize()`: 공분산 0.15 m·10° 넣어 보냄 → `settle()`: 제자리 ±15° 회전. 시작 때와 Dock 에서 작업 받아 undock 할 때 모두 |
| robot3 HOME | 2.855, 4.656, 176.7° | 2.806, 4.610, 179.9° (10/12 재측정) |
| 입출력 | — | `cmd_vel` 정지, `dock_status`, `compute_path_to_pose`, `navigate_to_pose`, `spin`, `drive_on_heading` 빠짐. initialpose 는 액션이 아니라 토픽 |
| 결과 `rssi_*` · `InspectFacility.run_id` · RCK 종류 | 아직 안 채움 (⚠) | 맞음, 아직 안 채움 |
