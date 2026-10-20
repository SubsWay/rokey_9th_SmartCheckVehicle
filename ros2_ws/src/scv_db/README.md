# SCV 운영 DB (3차)

기준: 2차(통신 구조 안 C — 제어 PC ↔ 관제 PC는 웹 API) + 3차 결정. 문서(SCV_DB구조도 · DB시스템제안설명 · DB시스템구현방안 · db항목별트리거정리)는 3차 판 하나로 두고 차수마다 덮어씁니다(지금은 앱 3-3차 기준).

**3차 변경** (로봇 2대 동시 운행 · 섹터 방식)
- **지도를 격자 기준 섹터 2개로 나눕니다.** `grid_cells.sector`(1 | 2)는 격자를 만들 때 관제 PC가 정합니다.
  - 지금은 임시 규칙입니다: ix 순으로 칸 수가 절반에 가장 가까운 열에서 나눠 왼쪽이 1, 오른쪽이 2입니다(바꿔 끼울 수 있는 함수).
  - 설비의 섹터는 view_pose가 속한 칸의 섹터로, 저장하지 않고 계산합니다.
- **runs.sector_robots** — 섹터 ↔ 로봇 배정입니다(예: `{"1": "robot3", "2": "robot4"}`).
  - 기본은 섹터 1 = robot3, 섹터 2 = robot4이며 순찰 시작 때 바꿀 수 있습니다.
  - 시작할 때 활성인 로봇의 섹터만 넣습니다.
  - task 로봇이 그 대상의 섹터 담당인지는 관제 PC가 확인합니다(DB 트리거 없음).
- **로봇마다 ACTIVE task 하나**입니다(`tasks_one_active_per_robot`). 두 로봇이 각자 섹터에서 동시에 운행합니다.
- **섹터별 실패** — 한 로봇의 task가 실패하면 그 로봇의 남은 task만 취소되고, 다른 로봇은 끝까지 수행합니다.
  - run status에 `partial`을 추가했습니다(일부 섹터만 실패).
  - 종료 판정은 `scv_close_run`, 비상정지·중단은 `scv_abort_run`입니다(§3).
- 재검증 로봇은 발견한 로봇과 같아도 됩니다(원문 C4 「다른 로봇이 재검증」 폐지). 자기 섹터의 후보는 자기가 정밀측정합니다.
- **AP · 설비가 지도에 속합니다** (격자 칸과 같은 방식). server page 정보 탭에서 지도를 골라 그 지도에 추가합니다.
  - `aps.map_id` 필수, 유일은 `(map_id, ssid)` — 같은 SSID를 지도마다 등록할 수 있고 ap_id는 지도마다 다릅니다. 설치 위치 `x · y`는 화면에서 입력(함께 넣거나 둘 다 비움).
  - `facilities` 키는 `(map_id, facility_id)` — 설비 ID는 지도 안에서 유일합니다. `inspections.map_id`를 추가해 `(map_id, facility_id)`로 설비를 가리키고, `map_id`가 그 run의 지도인지 트리거가 검사합니다. `tasks.target_ref` · `candidates.ref`의 설비 ID는 그 run의 지도 안에서 읽습니다.
  - 지도를 지우면 관제 PC가 그 지도의 격자 칸 · AP · 설비를 함께 지웁니다(기록이 참조하면 외래키로 거부 → 「사용 중」).
- **`maps.dock_poses`** — 로봇별 도크 접근 자세(도크 바로 앞에서 도크를 바라보는 자세). 예 `{"robot3": {"x": 1.0, "y": 0.6, "yaw": 1.5708}}`. 화면에서 입력하고, 관제 PC가 제어 PC에 보냅니다(연결 때 → 로봇 시작 위치 initialpose, 순찰 때 → DOCK task waypoints).
- 앞 3차 판의 역할 칼럼(`measure/detect/precise_robot_id`) · `runs_has_mission` · 트리거 `scv_check_task_role` · 「전체 ACTIVE 하나」는 없앴습니다.
- **측정 AP — `aps.is_target`** (앱 3-3차). 지도마다 정확히 하나(부분 유일 인덱스 `aps_one_target_per_map`). server page 정보 탭에서 **활성 AP 중에서만** 고르고(지도의 「현재 지도로」와 같은 방식), 순찰은 이 SSID의 표본만 모읍니다. 순찰 시작 조건에 「현재 지도의 측정 AP가 있고 활성」이 들어갑니다(관제 PC가 확인 — DB 트리거 없음).
- **`net_samples.task_id`** (앱 3-3차) — 표본을 모은 측정 task(SURVEY_SEGMENT). 제어 PC가 표본마다 붙이고, server page 이력 탭 task 「자세히」가 이것으로 그 task가 거친 칸 · 칸별 측정값을 보여 줍니다. 트리거 `scv_check_task_robot`이 같은 로봇 · 같은 run의 측정 task인지 검사합니다(비어 있어도 됨).
- **`runs.is_baseline_ok`** (앱 3-3차) — run이 done으로 끝날 때 정기 순찰(patrol)이고 RSSI 확정 이상이 없으면 관제 PC가 true로 둡니다. 기준선(baselines) 계산은 보고서 차수(3-5)입니다.
- **Redis를 쓰지 않습니다.** 로그인 세션 · 로봇 최신 상태 · 응답 없음(E3) 판정은 관제 PC server_page 메모리가 맡습니다(scv_app 3-2차). 컨테이너는 PostgreSQL 하나이고, 메모리에만 있는 기록은 없습니다(영구 기록은 모두 PostgreSQL).

```
scv_db/
├─ sql/
│  ├─ 01_schema.sql   테이블 20개 · 제약 · 인덱스 · 트리거 · 실패 처리 함수 · 설명(COMMENT 210개) · 이전 판 업그레이드
│  ├─ 02_seed.sql     로봇 robot3·robot4 · 판정 기준값 13개(key 설명 포함) · 수퍼 계정 admin
│  └─ 03_verify.sql   규칙 검증 90항목 (한 트랜잭션, 끝에 ROLLBACK → 데이터 남지 않음)
├─ dummy/
│  ├─ make_dummy.py   더미 데이터 생성 (기준 표 50행 안팎 + 지도·사진·보고서 파일)
│  └─ maps/           더미 지도 파일 yaml + pgm (dummy_testbed_a · dummy_floor_b · dummy_lab_c) — server page 「지도 등록」으로 다시 올리는 용도
└─ docker/
   └─ setup_db.sh     관제 PC에서 실행: PostgreSQL 컨테이너 생성 + SQL 적용
```

## 1. 실행 (관제 PC)

```bash
cd scv_db/docker
./setup_db.sh             # 컨테이너 + 01·02 적용
./setup_db.sh --verify    # 위 + 03 검증
./setup_db.sh --test               # 같은 컨테이너에 시험 DB robotdb4_test를 만들어 01·02 적용 (운영 DB는 그대로)
./setup_db.sh --test --verify      # 시험 DB + 03 검증
./setup_db.sh --test --reset       # 시험 DB를 지우고 처음부터 다시 (운영 DB는 지우지 않음)
```

스크립트가 하는 일:

| 순서 | 내용 |
|---|---|
| PostgreSQL | `docker pull postgres:16` → `docker volume create robotdata4_postgresql` → `docker run -d --name robotdb4_postgresql -e POSTGRES_DB=robotdb4_postgresql -e POSTGRES_USER=rokey -e POSTGRES_PASSWORD=rokey -p 5432:5432 -v robotdata4_postgresql:/var/lib/postgresql/data postgres:16` |
| SQL 적용 | `docker exec -i robotdb4_postgresql psql -h 127.0.0.1 -U rokey -d robotdb4_postgresql -v ON_ERROR_STOP=1 < ../sql/01_schema.sql` (02·03도 같은 방식) |

여러 번 실행해도 됩니다. 컨테이너·볼륨이 있으면 그대로 쓰고, SQL은 `IF NOT EXISTS` · `CREATE OR REPLACE` · `ON CONFLICT DO NOTHING`이라 이미 있는 데이터와 admin이 바꾼 설정값을 덮어쓰지 않습니다.

SQL만 따로 적용할 때:

```bash
docker exec -i robotdb4_postgresql psql -h 127.0.0.1 -U rokey -d robotdb4_postgresql -v ON_ERROR_STOP=1 < sql/01_schema.sql
docker exec -i robotdb4_postgresql psql -h 127.0.0.1 -U rokey -d robotdb4_postgresql -v ON_ERROR_STOP=1 < sql/02_seed.sql
```

## 2. 접속 정보

| | 주소 | 계정 |
|---|---|---|
| PostgreSQL | `postgresql://rokey:rokey@<관제PC>:5432/robotdb4_postgresql` | rokey / rokey |
| 수퍼 계정 | — | admin / `admin` (권한 admin — server page · client page, 비밀번호만 변경 가능, bcrypt 해시 저장) |
| 일반 계정 | — | 권한 user — client page만. server page 정보 탭에서 추가·수정·삭제 |

DB에 쓰는 것은 관제 PC의 api(inspection_manager 포함)뿐입니다. 제어 PC는 DB에 접속하지 않으므로, 컨테이너가 같은 PC에서만 쓰이면 포트를 `-p 127.0.0.1:5432:5432`로 묶어 외부 접속을 막는 것을 권장합니다. DBeaver 등으로 다른 PC에서 볼 때만 열어 두세요.

## 3. DB가 직접 지키는 규칙

애플리케이션 코드에 버그가 있어도 아래 규칙은 DB가 거부합니다(03_verify.sql에서 확인).

| 규칙 | 구현 |
|---|---|
| 진행 중인 run은 하나 | 부분 유일 인덱스 `runs_one_running` |
| 로봇마다 ACTIVE task 하나 — 두 로봇 동시 운행 (3차) | 부분 유일 인덱스 `tasks_one_active_per_robot` |
| run마다 섹터 ↔ 로봇 배정 필수(비어 있지 않은 JSON 객체) · 칸 섹터는 1 · 2 (3차) | CHECK `runs_sector_robots_check` · `grid_cells_sector_check` |
| 도크 접근 자세는 JSON 객체 · AP 설치 위치는 x · y 함께 (3차) | CHECK `maps_dock_poses_check` · `aps_xy_pair` |
| AP는 지도마다 SSID 하나 · 설비 ID는 지도 안에서 유일 (3차) | UK `aps_map_ssid_key` · PK `facilities_pkey (map_id, facility_id)` |
| 측정 AP는 지도마다 하나 (3-3차) | 부분 유일 인덱스 `aps_one_target_per_map` (활성 AP만 지정 · 순찰 중 잠금은 관제 PC) |
| 표본의 task = 같은 로봇 · 같은 run의 측정(SURVEY_SEGMENT) task (3-3차, 비어 있어도 됨) | 트리거 `scv_check_task_robot` (`trg_net_samples_check`) |
| 판정의 설비 = 그 run 지도의 설비 (3차) | FK `inspections_facility_fkey (map_id, facility_id)` · 트리거 `scv_check_task_robot` |
| 현재 지도 하나 · 칸·AP마다 현재 기준선 하나 | `maps_one_active` · `baselines_one_current` |
| 표본 중복 저장 0 (끊겼다 이어도) | `net_samples` PK `(robot_id, run_id, seq)` |
| 이벤트·판정·정지 측정·사진 재전송 중복 0 (제어 PC 아웃박스) | `robot_events.event_uid` UK · `inspections.inspection_uid` UK · `precise_measures (task_id, ap_id)` UK · `media.object_key` UK |
| 후보는 1차 task에서, 그 task의 로봇이 발견 | 트리거 `scv_check_candidate` |
| 재검증 task는 같은 run의 REINSPECTION, target_ref = candidate_id. 재검증 로봇은 발견 로봇과 같아도 됨(3차) | 트리거 `scv_check_candidate` |
| **재검증 없이 확정 0건** (findings는 outcome = CONFIRMED + 그 후보의 재검증 task일 때만) | 트리거 `scv_check_finding` |
| 확정 이상이 있는 후보의 결론 변경 금지 | 트리거 `scv_lock_confirmed_candidate` |
| 판정·정지 측정·이벤트의 robot_id = task의 로봇 | 트리거 `scv_check_task_robot` |
| 후보·확정 이벤트에는 task_id 필수, 관제 PC 판단(source = control)은 E3·E8만 | CHECK |
| done이 아닌 run(partial · aborted)은 기준선 회차가 될 수 없음 | CHECK `runs_baseline_only_done` |
| 권한 admin은 username `admin` 하나뿐 (다른 admin 추가 불가) | CHECK `users_admin_is_super` + username UK |
| 수퍼 계정 admin 삭제·이름·권한 변경 금지 (비밀번호 변경만 허용) | 트리거 `scv_protect_super_user` |

실패·종료 처리는 한 트랜잭션으로 묶은 함수로 제공합니다(3차 — 섹터별 실패).

```sql
SELECT scv_fail_task('r12-t003', 'BLOCKED');  -- 그 task FAILED + 같은 run의 그 로봇 남은 task만 CANCELED(SECTOR_ABORTED). 반환: 취소 수
SELECT scv_close_run(12);                     -- 남은 task가 없으면 종료. 반환: done | partial | aborted (남아 있으면 NULL)
SELECT scv_abort_run(12, 'ESTOP');            -- 비상정지·관리자 중단: 두 로봇의 남은 task 모두 CANCELED + aborted. 반환: 취소 수
```

| 종료 status | 조건 |
|---|---|
| done | 실패한 로봇 없음 — 배정된 섹터 모두 끝까지(로봇마다 DOCK) |
| partial | 일부 로봇만 실패 — 실패한 섹터만 중단, 다른 섹터는 끝까지 |
| aborted | 배정된 로봇 모두 실패, 또는 `scv_abort_run` |

실패한 로봇의 복귀 DOCK은 관제 PC가 새 task로 만들어 보냅니다(이전과 같음).

api는 제어 PC가 다시 보낸 데이터를 오류 없이 무시하도록 `ON CONFLICT … DO NOTHING`으로 저장합니다.

```sql
INSERT INTO robot_events (event_uid, robot_id, run_id, task_id, code, level, detail, ts)
VALUES ($1, $2, $3, $4, $5, $6, $7, $8)
ON CONFLICT (event_uid) DO NOTHING;            -- 다른 표: (robot_id, run_id, seq) · (inspection_uid) · (task_id, ap_id) · (object_key)
```

`event_uid`는 제어 PC scv_agent가 아웃박스에 쓸 때 붙이고(예: `robot3-r12-e000123`), 관제 PC가 판단하는 E3·E8은 api가 `control-<uuid>`로 붙입니다. `inspection_uid`는 detection_alert가 판정마다 만드는 UUID입니다.

## 4. 운영 메모

- **볼륨이 데이터입니다.** `docker rm`은 컨테이너만 지우고 데이터는 `robotdata4_postgresql`에 남습니다. 처음부터 다시 만들 때만 `docker rm -f robotdb4_postgresql && docker volume rm robotdata4_postgresql`.
- **이전 판에서 만든 Redis 컨테이너**(robotdb4_redis)는 더 쓰지 않습니다. 지우려면 `docker rm -f robotdb4_redis && docker volume rm robotdata4_redis`(PostgreSQL 데이터와 무관).
- **백업:** `docker exec robotdb4_postgresql pg_dump -U rokey -Fc robotdb4_postgresql > scv_$(date +%Y%m%d).dump`
- **이전 01_schema.sql로 이미 DB를 만들었다면** 새 01_schema.sql → 02_seed.sql을 그대로 다시 실행하면 됩니다(`./setup_db.sh`). 파일 끝의 업그레이드 구간이
  - `event_uid`·`inspection_uid` 컬럼과 세 유일키를 추가하고 기존 행에는 `legacy-<event_id>`·임의 UUID를 채웁니다.
  - `settings.description`을 추가합니다(맨 끝 컬럼으로 붙음 — 이름으로 읽으므로 동작은 같음). 설명은 02_seed.sql이 비어 있는 행에만 채우고, 값·단위·근거는 건드리지 않습니다.
  - users의 옛 권한(admin | viewer)을 admin | user로 바꿉니다. admin 외 계정(viewer, 추가했던 admin 권한 계정)은 모두 일반 계정 user가 되고, **admin 비밀번호는 `admin`으로 한 번 초기화**됩니다. 필요 없는 계정은 server page에서 지우세요.
  - **3차** — runs에 `sector_robots`를 추가합니다. 이전 run은 그 run에 task가 있던 로봇을 이름순으로 섹터 1, 2에 채웁니다(task가 없으면 첫 로봇). 섹터 분할 전 기록이라 실제 구역 구분은 없습니다.
  - 앞 3차 판의 역할 칼럼 3개 · `runs_has_mission` · 트리거 `scv_check_task_role` · 인덱스 `tasks_one_active`를 지우고, `tasks_one_active_per_robot`을 다시 만듭니다. run status CHECK에 `partial`을 더합니다.
  - `maps.dock_poses`를 추가합니다(이전 지도는 `{}` — 화면에서 입력).
  - aps를 지도에 묶습니다. 이전 AP의 지도는 측정 기록의 지도 → 현재 지도 → 첫 지도 순으로 정하고, 지도가 하나도 없으면 지웁니다. 유일 조건은 `ssid` → `(map_id, ssid)`로 바꿉니다.
  - facilities 키를 `(map_id, facility_id)`로 바꾸고, `inspections.map_id`(이전 판정은 그 설비의 지도)와 외래키 `(map_id, facility_id)`를 추가합니다.
  - `grid_cells.sector`를 추가하고, 이미 있는 칸은 격자 생성 때와 같은 규칙(ix 순 칸 수 절반)으로 채웁니다.
  - `robots`·`aps`의 `is_active` 기본값을 false로 바꿉니다. 이미 있는 행의 값은 server_page가 시작할 때 모두 비활성으로 맞춥니다.
  - **3-3차** — `aps.is_target`(기본 false)과 `aps_one_target_per_map`을 추가합니다. 이전 지도에는 측정 AP가 없으므로 server page 정보 탭에서 지정해야 순찰을 시작할 수 있습니다.
  - **3-3 보완** — 설비 종류에 `rack`(랙)을 더합니다. 이전 DB의 `facilities.facility_type` CHECK(3종류)를 4종류로 다시 만듭니다(기존 설비 값은 그대로).
  - **3-3차** — `net_samples.task_id`(외래키 tasks)와 트리거 `trg_net_samples_check`를 추가합니다. 이전 표본은 같은 로봇 · run의 측정 task 수행 시간(assigned_at ~ ended_at) 안이면 그 task로 채우고, 못 찾으면 비워 둡니다.
  - 02_seed.sql이 `wall_margin_m`의 이전 설명(「경로·격자 칸을…」)을 새 설명으로 바꿉니다(설명을 직접 고치지 않은 경우만).
- **03_verify.sql은 운영 전에.** 진행 중인 run이 있으면 `runs_one_running` 때문에 실패합니다. 모두 ROLLBACK되어 데이터는 남지 않고 번호 시퀀스만 조금 올라갑니다.
- `/docker-entrypoint-initdb.d`에 SQL을 넣는 방식은 **볼륨이 비어 있는 첫 실행 때만** 동작하므로, 이미 볼륨을 만든 상태에서도 쓸 수 있게 `docker exec`로 적용합니다.
- 지도는 SLAM 지도를 만든 뒤 server page 정보 탭 「지도 등록」(또는 등록 스크립트)으로, AP · 설비 · 도크 접근 자세는 그 지도를 골라 정보 탭에서 넣습니다. 순찰 전에 측정 AP를 하나 지정합니다(활성 AP만).
- 설정값의 `source`(BR-T04, SR-06 …)는 문서의 「제안」 근거이며, 첫 측정 뒤 조정합니다. `description`은 key 설명으로 server page 설정 화면에 함께 보여 줄 값입니다.
- **활성 여부** — `robots.is_active`·`aps.is_active`는 client(제어 PC)·로봇이 값을 보내 줄 때만 true입니다(기본값 false, server page에서는 바꾸지 않음). robots는 제어 PC WebSocket 연결(hello) · 끊김, aps는 client status의 `ssids`(최근 5초 안에 받은 SSID)로 정합니다(앱 3-3차).
- **격자 칸** — 지도의 흰 부분(빈 공간)이 조금이라도 걸치는 칸은 모두 만듭니다. `wall_margin_m`은 순찰 경로를 만들 때만 씁니다.
- 비밀번호(rokey, 초기 계정)는 시연용입니다. 외부망에 연결하면 바꾸세요.

## 5. 더미 데이터 (dummy/make_dummy.py)

화면·쿼리 시험용 데이터를 넣습니다. 01·02를 적용한 DB에서 관제 PC의 server_page와 같은 파이썬 환경(psycopg · numpy · pillow · pyyaml)으로 실행합니다.

더미 지도 3개 중 하나는 **실제 SLAM 지도**입니다. 기본으로 `/home/rokey/ws_cobot_pjt/5-4_Project_04/maps` 폴더의 yaml(이름순 첫 번째)과 그 yaml이 가리키는 pgm을 읽어 server page 지도 등록과 같은 방식(yaw = 0 · negate = 0 확인, pgm을 크기 그대로 PNG)으로 넣습니다. 폴더나 파일이 없으면 아무것도 넣지 않고 멈추며, `--map-dir` · `--map-yaml`로 바꾸거나 `--no-real-map`으로 합성 지도만 쓸 수 있습니다.

```bash
cd scv_db/dummy
python make_dummy.py              # 비어 있는 DB에 생성 (--media 기본 ../../scv_app/scv_media)
python make_dummy.py --reset      # 지우고 다시 생성
```

실행하면 지도 파일(yaml + pgm)도 `--map-out` 폴더(기본 `dummy/maps/`)에 씁니다. 합성 지도 · 실제 지도 모두 DB에 넣은 것과 같은 해상도 · origin · 그림이라, 지도를 지운 뒤 server page 정보 › 지도 › 「지도 등록」으로 다시 올리거나 다른 DB에 올릴 때 씁니다(yaml의 image는 같은 폴더의 pgm 이름). 묶음에 든 `dummy/maps/`는 `--no-real-map`으로 만든 합성 지도 3개입니다.

| 옵션 | 기본값 | 내용 |
|---|---|---|
| `--dsn` | `SCV_DB_DSN` 또는 `postgresql://rokey:rokey@localhost:5432/robotdb4_postgresql` | DB 주소 |
| `--media` | 이 스크립트 기준 `../../scv_app/scv_media`(server_page.py 기본 위치 — 실행한 폴더와 상관없음). 환경 변수 `SCV_MEDIA_DIR`이 있으면 그 값 | server_page.py의 `SCV_MEDIA_DIR`과 같은 곳. 지도 PNG·사진·보고서 PDF를 실제 파일로 만듦. **다른 곳에 만들면 server page에 지도 · 사진이 안 보임**(이력 탭 지도가 빈 화면이던 원인) |
| `--n` | 50 | 기준 개수 (users · aps · facilities · runs) |
| `--seed` | 2026 | 난수 시드 — 같은 시드면 같은 데이터 |
| `--reset` | — | robots · settings · admin만 남기고 모두 지운 뒤 생성(지운 행의 파일도 삭제). 없으면 데이터가 이미 있을 때 멈춤 |
| `--map-dir` | `/home/rokey/ws_cobot_pjt/5-4_Project_04/maps` | 실제 지도 폴더 |
| `--map-yaml` | 이름순 첫 번째 `*.yaml` | 그 폴더에서 쓸 yaml 파일 이름 |
| `--no-real-map` | — | 실제 지도 대신 합성 지도 `dummy_floor_b` 사용 |
| `--map-out` | 스크립트 옆 `maps/` | 지도 파일(yaml + pgm)을 쓸 폴더 |

만들어지는 행 수(기본값). 기준 표는 50행, 나머지는 트리거·CHECK 규칙(후보는 1차 task에서, 재검증은 발견한 로봇이 자기 섹터에서, 확정은 CONFIRMED 재검증으로만 …)을 지키며 그 데이터에 맞춰 생깁니다.

| 표 | 행 | 내용 |
|---|---|---|
| users | 50 | user01 ~ user50, 권한 user, 비밀번호 모두 `user1234` (+ admin) |
| aps | 50 | 현재 지도에 SCV_AP_01 ~ 40, 다른 두 지도에 같은 SSID 01 ~ 05(지도마다 다른 ap_id). **모두 비활성**(보고 전). 현재 지도의 01 ~ 05는 과거 측정 기록에 쓰인 AP이고 설치 위치가 있음. **측정 AP는 지도마다 SCV_AP_01**(화면에서는 활성 AP만 고를 수 있지만 더미는 보고가 없어 비활성 상태로 지정 — 순찰 시작 조건 「측정 AP 활성」은 client가 그 SSID를 보내야 만족) |
| maps | 3 | dummy_testbed_a(합성, 현재 지도, 9 × 6 m 방) · **실제 지도**(yaml 이름, 현재 지도 아님 — server page에서 「현재 지도로」 누르면 격자 생성) · dummy_lab_c(합성) — PNG 파일 생성. 합성 지도의 도크 접근 자세는 robot3 왼쪽 아래 · robot4 오른쪽 아래, 실제 지도는 비움(화면에서 입력) |
| grid_cells | 54 | 현재 지도에만 생성(server page에서 지도를 선택할 때와 같은 계산 — 흰 부분이 걸치는 칸 모두). 섹터 1 = 24칸 · 섹터 2 = 30칸 |
| facilities | 54 | 현재 지도에 소화기 · 셔터 · 유도등 · 랙 각 12 ~ 13개(2개 비활성) + dummy_lab_c에 같은 ID 4개(EXT-01 · EXT-02 · SHT-01 · EXL-01 — 설비 ID는 지도 안에서 유일) |
| runs | 50 | 하루 1회, done 42 · partial 4 · aborted 4, 기준선 재료(`is_baseline_ok`) 22 — done · patrol · RSSI 확정 이상 없음. 섹터 1 = robot3 · 섹터 2 = robot4 동시 운행(10회는 robot3 혼자 — 섹터 1만). partial은 주 로봇의 섹터만 실패, aborted는 혼자 운행한 회차의 실패 |
| tasks | 206 | 회차마다 주 task 1개(주 로봇을 번갈아, 자기 섹터의 SURVEY_SEGMENT · FACILITY_CHECK) + 다른 로봇의 자기 섹터 측정 1구간 + 정밀측정 26(발견한 로봇이 스스로) + 로봇마다 DOCK(경로점 = 그 로봇의 도크 접근 자세). 측정 경로는 server의 `plan_survey`처럼 **섹터 칸 중심을 지그재그로** 이어 20점씩 자르고 `seg:a-b`(섹터 경로의 점 번호) — 더미 지도는 합성이라 벽 여유는 계산하지 않음 |
| candidates · findings | 28 · 12 | CLOSED 24(CONFIRMED 12 · CLEARED 8 · UNCONFIRMED 4) · OPEN 2 · REINSPECTING 2. 네트워크 후보 칸 = 그 측정 task가 지난 칸 중 하나(실패한 task는 간 데까지), 그 칸 표본 4개는 기준(−72) 미만, 후보 지점 = 그 칸의 마지막 표본 위치 — client 후보 규칙과 같은 모양 |
| net_samples · grid_stats · baselines | 약 2,000 · 50 · 50 | 표본: **모든 측정 task**(실패는 진행률만큼)가 경로를 따라 칸마다 3 ~ 4개, **측정 AP(SCV_AP_01) 하나**, `task_id` 포함(3-3차 — 이력 「자세히」의 칸별 측정값 · 칸 순서). 값은 측정 AP까지 거리 + 잡음, 위치 불확실(`pose_ok = false`) 약 5 % · 기준선 25칸 × 버전 2 |
| precise_measures | 12 | 네트워크 후보 정밀측정 12건 × 측정 AP 1 |
| inspections · media | 37 · 50 | 설비 판정 · 사진 rgb/depth 25쌍 (파일 생성) |
| robot_events | 60 | 후보·결론 52 + 중단 사유 E1 · E2 · E3(관제 PC) · E7 |
| robot_status_log · reports | 약 8,000 · 50 | 상태: **모든 task**(측정 · 정밀측정 · 탐지 · 복귀) 동안 경로를 따라 위치 · 배터리 · `task_id`(이력 「자세히」의 지나간 길 · 배터리 · 실제 칸 순서). 실제는 1초마다지만 더미 task는 수십 분이라 task마다 최대 40점 · 보고서 PDF 25회차 × 2종 (파일 생성) |

robots(2)와 settings(13)는 02_seed.sql 그대로 두고, 로봇은 보고 전이므로 비활성으로만 맞춥니다.

## 6. 시험 DB 쓰는 법 (3차)

가상 로봇 시험은 운영 DB가 아닌 **시험 DB `robotdb4_test`**에서 합니다.

```bash
cd scv_db/docker
./setup_db.sh --test --reset        # 깨끗한 시험 DB (01 · 02 적용)
```

server_page.py는 `SCV_DB_DSN`으로 시험 DB를 가리킵니다.

```bash
SCV_DB_DSN="host=127.0.0.1 port=5432 dbname=robotdb4_test user=rokey password=rokey" python3 server_page.py
```

가상 세계(시험 전용 지도 · AP · 설비)는 scv_app의 등록 스크립트로 server API를 거쳐 넣습니다(앱 3-1차). 화면 확인용 과거 이력이 필요하면 `make_dummy.py --dsn …/robotdb4_test`로 더미를 넣을 수도 있지만, 가상 로봇 시험과는 섞지 않는 것을 권합니다.
