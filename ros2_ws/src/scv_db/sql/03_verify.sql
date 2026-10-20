-- =====================================================================
-- SCV 운영 DB 규칙 검증 (3차)
--   01_schema.sql · 02_seed.sql 다음에 실행. 전체를 한 트랜잭션에서 돌리고 마지막에 ROLLBACK
--   → DB에 아무것도 남지 않음 (번호 시퀀스만 조금 증가).
--   진행 중인 run이 이미 있으면 runs_one_running 때문에 실패하므로 운영 전에 실행.
--   결과: 모든 항목이 PASS면 「검증 통과」, 하나라도 FAIL이면 오류로 끝남.
-- =====================================================================

BEGIN;
SET client_min_messages = notice;

CREATE TEMP TABLE v_result (no SERIAL, item TEXT, result TEXT, message TEXT) ON COMMIT DROP;

-- 실패해야 하는 문장: 오류가 나면 PASS (그 문장만 되돌림)
CREATE FUNCTION pg_temp.expect_fail(p_item TEXT, p_sql TEXT) RETURNS void LANGUAGE plpgsql AS $$
BEGIN
    BEGIN
        EXECUTE p_sql;
        INSERT INTO v_result (item, result, message) VALUES (p_item, 'FAIL', '오류가 나야 하는데 성공함');
    EXCEPTION WHEN OTHERS THEN
        INSERT INTO v_result (item, result, message) VALUES (p_item, 'PASS', SQLERRM);
    END;
END $$;

-- 성공해야 하는 문장
CREATE FUNCTION pg_temp.expect_ok(p_item TEXT, p_sql TEXT) RETURNS void LANGUAGE plpgsql AS $$
BEGIN
    EXECUTE p_sql;
    INSERT INTO v_result (item, result, message) VALUES (p_item, 'PASS', '성공');
EXCEPTION WHEN OTHERS THEN
    INSERT INTO v_result (item, result, message) VALUES (p_item, 'FAIL', SQLERRM);
END $$;

-- ---------------------------------------------------------------------
-- 준비: 지도 1 · 칸 2 · AP 1 · 설비 1 · run 1 · task 4
--   run 900001 섹터 배정: 섹터 1 = robot3 · 섹터 2 = robot4 (3차 — 두 로봇이 각자 섹터에서 동시에 운행)
--   칸 900001 = 섹터 1, 칸 900002 = 섹터 2. 후보는 발견한 로봇이 스스로 재검증(같은 섹터)
-- ---------------------------------------------------------------------
INSERT INTO maps (map_id, name, robot_map_path, image_key, resolution_m, origin_x, origin_y, is_active)
VALUES (900001, 'verify_map', '/opt/scv/maps/verify_map.yaml', 'maps/verify_map.png', 0.05, -5.0, -5.0, false);
INSERT INTO grid_cells (cell_id, map_id, ix, iy, center_x, center_y, sector) VALUES
    (900001, 900001, 8, 12, 3.5, 7.5, 1),
    (900002, 900001, 9, 12, 4.5, 7.5, 2);
INSERT INTO aps (ap_id, map_id, ssid) VALUES (9001, 900001, 'VERIFY_AP');
INSERT INTO facilities (facility_id, map_id, facility_type, has_gauge, expected_x, expected_y, view_pose)
VALUES ('v-ext-01', 900001, 'extinguisher', true, 2.0, 3.0, '{"x": 2.0, "y": 2.2, "yaw": 1.57}');

INSERT INTO runs (run_id, map_id, run_type, sector_robots)
VALUES (900001, 900001, 'patrol', '{"1": "robot3", "2": "robot4"}');
INSERT INTO tasks (task_id, run_id, robot_id, type, target_ref, waypoints, status, assigned_at) VALUES
    ('v-t001', 900001, 'robot3', 'SURVEY_SEGMENT', 'seg:0-20', '[{"x":3.5,"y":7.5,"yaw":0}]', 'ACTIVE', now()),
    ('v-t002', 900001, 'robot4', 'FACILITY_CHECK', 'v-ext-01', '[{"x":2.0,"y":2.2,"yaw":1.57}]', 'PENDING', NULL),
    ('v-t003', 900001, 'robot3', 'REINSPECTION',   'v-c01',    '[{"x":3.5,"y":7.5,"yaw":0}]', 'PENDING', NULL),
    ('v-t004', 900001, 'robot4', 'REINSPECTION',   'v-c02',    '[{"x":2.0,"y":2.2,"yaw":1.57}]', 'PENDING', NULL);

-- 항목별 빈 결과 행은 숨기고 마지막 결과 표만 출력 (psql 전용)
\o /dev/null

-- ---------------------------------------------------------------------
-- 1. 단일 테이블 규칙 (CHECK · 부분 유일 인덱스)
-- ---------------------------------------------------------------------
SELECT pg_temp.expect_ok  ('1-1 RSSI 표본 저장',
    $q$INSERT INTO net_samples (robot_id, run_id, seq, stamp, x, y, yaw, pose_ok, ap_id, rssi_dbm, cell_id)
       VALUES ('robot3', 900001, 0, now(), 3.4, 7.6, 0, true, 9001, -78, 900001),
              ('robot3', 900001, 1, now(), 3.6, 7.4, 0, true, 9001, -80, 900001),
              ('robot3', 900001, 2, now(), 3.5, 7.5, 0, true, 9001, -76, 900001)$q$);
SELECT pg_temp.expect_fail('1-2 같은 (robot, run, seq) 표본 중복 저장 거부',
    $q$INSERT INTO net_samples (robot_id, run_id, seq, stamp, x, y, yaw, pose_ok, ap_id, rssi_dbm)
       VALUES ('robot3', 900001, 0, now(), 0, 0, 0, true, 9001, -70)$q$);
SELECT pg_temp.expect_fail('1-3 RSSI 범위 밖(+5 dBm) 거부',
    $q$INSERT INTO net_samples (robot_id, run_id, seq, stamp, x, y, yaw, pose_ok, ap_id, rssi_dbm)
       VALUES ('robot3', 900001, 9, now(), 0, 0, 0, true, 9001, 5)$q$);
SELECT pg_temp.expect_fail('1-4 진행 중 run 2개 거부',
    $q$INSERT INTO runs (map_id, run_type, sector_robots) VALUES (900001, 'patrol', '{"1": "robot3"}')$q$);
SELECT pg_temp.expect_fail('1-5 같은 로봇의 ACTIVE task 2개 거부 (robot3 측정 중 재검증 시작)',
    $q$UPDATE tasks SET status = 'ACTIVE', assigned_at = now() WHERE task_id = 'v-t003'$q$);
SELECT pg_temp.expect_fail('1-6 DOCK 외 task에 target_ref 없음 거부',
    $q$INSERT INTO tasks (task_id, run_id, robot_id, type) VALUES ('v-t099', 900001, 'robot3', 'SURVEY_SEGMENT')$q$);
SELECT pg_temp.expect_fail('1-7 끝난 task에 ended_at 없음 거부',
    $q$UPDATE tasks SET status = 'DONE' WHERE task_id = 'v-t002'$q$);
SELECT pg_temp.expect_fail('1-8 robot_id 형식(하이픈) 거부',
    $q$INSERT INTO robots (robot_id) VALUES ('robot-5')$q$);
SELECT pg_temp.expect_fail('1-9 정면이 아닌 판정에 게이지 상태 거부',
    $q$INSERT INTO inspections (inspection_uid, task_id, run_id, robot_id, map_id, facility_id, detail, final, frontal, gauge_state, model_version, inspected_at)
       VALUES (gen_random_uuid(), 'v-t002', 900001, 'robot4', 900001, 'v-ext-01', 'NORMAL', 'normal', false, 'green', 'yolo-v1', now())$q$);
SELECT pg_temp.expect_fail('1-10 NETWORK 후보에 VIS_ 종류 거부',
    $q$INSERT INTO candidates (candidate_id, run_id, source, kind, ref, target_x, target_y, detect_task_id, detected_by, detected_at)
       VALUES ('v-c09', 900001, 'NETWORK', 'VIS_OCCLUDED', '900002', 4.5, 7.5, 'v-t001', 'robot3', now())$q$);
SELECT pg_temp.expect_fail('1-11 후보·확정 이벤트에 task_id 없음 거부',
    $q$INSERT INTO robot_events (event_uid, robot_id, run_id, code, level, ts) VALUES ('v-e91', 'robot3', 900001, 'CANDIDATE', 'warn', now())$q$);
SELECT pg_temp.expect_fail('1-12 관제 PC 판단(source=control)은 E3·E8만',
    $q$INSERT INTO robot_events (event_uid, robot_id, code, source, ts) VALUES ('v-e92', 'robot3', 'E1', 'control', now())$q$);
SELECT pg_temp.expect_ok  ('1-13 관제 PC 판단 E3 저장',
    $q$INSERT INTO robot_events (event_uid, robot_id, run_id, code, source, level, detail, ts) VALUES ('control-v-e01', 'robot4', 900001, 'E3', 'control', 'error', '{"robot_alive": true}', now())$q$);

-- 3차 — 섹터(grid_cells.sector · runs.sector_robots)와 두 로봇 동시 운행
SELECT pg_temp.expect_fail('1-14 섹터 배정(sector_robots) 없는 run 거부',
    $q$INSERT INTO runs (map_id, run_type, status, ended_at) VALUES (900001, 'patrol', 'done', now())$q$);
SELECT pg_temp.expect_fail('1-15 빈 섹터 배정 {} 거부',
    $q$INSERT INTO runs (map_id, run_type, status, ended_at, sector_robots) VALUES (900001, 'patrol', 'done', now(), '{}')$q$);
SELECT pg_temp.expect_fail('1-16 객체가 아닌 섹터 배정(배열) 거부',
    $q$INSERT INTO runs (map_id, run_type, status, ended_at, sector_robots) VALUES (900001, 'patrol', 'done', now(), '["robot3"]')$q$);
SELECT pg_temp.expect_ok  ('1-17 활성 로봇 1대 — 섹터 1개만 배정한 run 허용',
    $q$INSERT INTO runs (map_id, run_type, status, ended_at, sector_robots) VALUES (900001, 'demo', 'done', now(), '{"2": "robot4"}')$q$);
SELECT pg_temp.expect_ok  ('1-18 일부 섹터 실패 run(status partial) 저장',
    $q$INSERT INTO runs (map_id, run_type, status, ended_at, sector_robots) VALUES (900001, 'patrol', 'partial', now(), '{"1": "robot3", "2": "robot4"}')$q$);
SELECT pg_temp.expect_fail('1-19 칸 섹터 번호 3 거부 (섹터는 1 · 2)',
    $q$INSERT INTO grid_cells (map_id, ix, iy, center_x, center_y, sector) VALUES (900001, 10, 12, 5.5, 7.5, 3)$q$);
SELECT pg_temp.expect_ok  ('1-20 칸 섹터 저장 확인 (900001 = 1, 900002 = 2)',
    $q$DO $d$ BEGIN
         IF (SELECT string_agg(sector::text, ',' ORDER BY cell_id) FROM grid_cells WHERE map_id = 900001) <> '1,2'
         THEN RAISE EXCEPTION '섹터 값이 다름'; END IF;
       END $d$$q$);
SELECT pg_temp.expect_ok  ('1-21 DOCK task 허용 (waypoints · target_ref 없이)',
    $q$DO $d$ BEGIN
         INSERT INTO tasks (task_id, run_id, robot_id, type) VALUES ('v-t093', 900001, 'robot3', 'DOCK');
         DELETE FROM tasks WHERE task_id = 'v-t093';
       END $d$$q$);
SELECT pg_temp.expect_ok  ('1-22 두 로봇 동시 운행 — robot3 측정 중 robot4 task ACTIVE 허용',
    $q$UPDATE tasks SET status = 'ACTIVE', assigned_at = now() WHERE task_id = 'v-t002'$q$);

-- 3차 — 지도 도크 접근 자세(maps.dock_poses) · AP · 설비가 지도에 속함
INSERT INTO maps (map_id, name, robot_map_path, image_key, resolution_m, origin_x, origin_y, is_active)
VALUES (900002, 'verify_map_2', '/opt/scv/maps/verify_map_2.yaml', 'maps/verify_map_2.png', 0.05, 0, 0, false);
SELECT pg_temp.expect_ok  ('1-23 지도 도크 접근 자세 저장 (로봇별 x, y, yaw)',
    $q$UPDATE maps SET dock_poses = '{"robot3": {"x": 1.0, "y": 0.6, "yaw": 1.57}, "robot4": {"x": 7.0, "y": 0.6, "yaw": 1.57}}' WHERE map_id = 900001$q$);
SELECT pg_temp.expect_fail('1-24 객체가 아닌 도크 위치(배열) 거부',
    $q$UPDATE maps SET dock_poses = '[1, 2]' WHERE map_id = 900001$q$);
SELECT pg_temp.expect_ok  ('1-25 AP 설치 위치 저장',
    $q$UPDATE aps SET x = 2.0, y = 6.5 WHERE ap_id = 9001$q$);
SELECT pg_temp.expect_fail('1-26 지도 없는 AP 거부',
    $q$INSERT INTO aps (ssid) VALUES ('VERIFY_AP_2')$q$);
SELECT pg_temp.expect_fail('1-27 x만 있고 y 없는 AP 위치 거부',
    $q$INSERT INTO aps (map_id, ssid, x) VALUES (900001, 'VERIFY_AP_3', 1.0)$q$);
SELECT pg_temp.expect_fail('1-28 같은 지도에 같은 SSID 거부',
    $q$INSERT INTO aps (map_id, ssid) VALUES (900001, 'VERIFY_AP')$q$);
SELECT pg_temp.expect_ok  ('1-29 다른 지도에는 같은 SSID 허용',
    $q$INSERT INTO aps (map_id, ssid) VALUES (900002, 'VERIFY_AP')$q$);
SELECT pg_temp.expect_ok  ('1-34 측정 AP 지정 (3-3차 aps.is_target)',
    $q$UPDATE aps SET is_target = true WHERE ap_id = 9001$q$);
SELECT pg_temp.expect_fail('1-35 같은 지도에 측정 AP 두 개 거부 (aps_one_target_per_map)',
    $q$INSERT INTO aps (map_id, ssid, is_target) VALUES (900001, 'VERIFY_AP_T', true)$q$);
SELECT pg_temp.expect_ok  ('1-36 다른 지도에는 측정 AP 따로 허용',
    $q$UPDATE aps SET is_target = true WHERE map_id = 900002 AND ssid = 'VERIFY_AP'$q$);
SELECT pg_temp.expect_fail('1-30 같은 지도에 같은 설비 ID 거부',
    $q$INSERT INTO facilities (facility_id, map_id, facility_type, expected_x, expected_y, view_pose)
       VALUES ('v-ext-01', 900001, 'extinguisher', 1, 1, '{"x": 1, "y": 1.8, "yaw": 1.57}')$q$);
SELECT pg_temp.expect_ok  ('1-31 다른 지도에는 같은 설비 ID 허용',
    $q$INSERT INTO facilities (facility_id, map_id, facility_type, expected_x, expected_y, view_pose)
       VALUES ('v-ext-01', 900002, 'extinguisher', 1, 1, '{"x": 1, "y": 1.8, "yaw": 1.57}')$q$);
SELECT pg_temp.expect_fail('1-32 run 지도와 다른 지도의 설비 판정 거부',
    $q$INSERT INTO inspections (inspection_uid, task_id, run_id, robot_id, map_id, facility_id, detail, final, frontal, model_version, inspected_at)
       VALUES (gen_random_uuid(), 'v-t002', 900001, 'robot4', 900002, 'v-ext-01', 'NORMAL', 'normal', true, 'yolo-v1', now())$q$);
SELECT pg_temp.expect_fail('1-33 지도에 없는 설비의 판정 거부 (외래키 (map_id, facility_id))',
    $q$INSERT INTO inspections (inspection_uid, task_id, run_id, robot_id, map_id, facility_id, detail, final, frontal, model_version, inspected_at)
       VALUES (gen_random_uuid(), 'v-t002', 900001, 'robot4', 900001, 'v-ext-99', 'NORMAL', 'normal', true, 'yolo-v1', now())$q$);
-- 3-3차 — 표본을 모은 측정 task (net_samples.task_id, 이력 「자세히」). 표본 수 확인(3-1)이 있으므로 넣고 바로 지움
SELECT pg_temp.expect_ok  ('1-37 표본에 측정 task 연결 (net_samples.task_id)',
    $q$DO $d$ BEGIN
         INSERT INTO net_samples (robot_id, run_id, seq, stamp, x, y, yaw, pose_ok, ap_id, rssi_dbm, cell_id, task_id)
         VALUES ('robot3', 900001, 10, now(), 3.5, 7.5, 0, true, 9001, -77, 900001, 'v-t001');
         DELETE FROM net_samples WHERE run_id = 900001 AND seq = 10;
       END $d$$q$);
SELECT pg_temp.expect_fail('1-38 다른 로봇의 측정 task로 표본 저장 거부',
    $q$INSERT INTO net_samples (robot_id, run_id, seq, stamp, x, y, yaw, pose_ok, ap_id, rssi_dbm, task_id)
       VALUES ('robot4', 900001, 10, now(), 4.5, 7.5, 0, true, 9001, -70, 'v-t001')$q$);
SELECT pg_temp.expect_fail('1-39 측정(SURVEY_SEGMENT)이 아닌 task로 표본 저장 거부',
    $q$INSERT INTO net_samples (robot_id, run_id, seq, stamp, x, y, yaw, pose_ok, ap_id, rssi_dbm, task_id)
       VALUES ('robot3', 900001, 11, now(), 3.5, 7.5, 0, true, 9001, -70, 'v-t003')$q$);
-- 3-3 보완 — 설비 종류 rack(랙). 다른 항목의 설비 수에 영향이 없도록 넣고 바로 지움
SELECT pg_temp.expect_ok  ('1-40 설비 종류 rack(랙) 저장',
    $q$DO $d$ BEGIN
         INSERT INTO facilities (facility_id, map_id, facility_type, expected_x, expected_y, view_pose)
         VALUES ('v-rck-01', 900001, 'rack', 4.0, 3.0, '{"x": 4.0, "y": 2.2, "yaw": 1.57}');
         DELETE FROM facilities WHERE map_id = 900001 AND facility_id = 'v-rck-01';
       END $d$$q$);
SELECT pg_temp.expect_fail('1-41 정해지지 않은 설비 종류 거부',
    $q$INSERT INTO facilities (facility_id, map_id, facility_type, expected_x, expected_y, view_pose)
       VALUES ('v-xxx-01', 900001, 'ladder', 4.0, 3.0, '{"x": 4.0, "y": 2.2, "yaw": 1.57}')$q$);

-- ---------------------------------------------------------------------
-- 2. 후보 → 재검증 → 확정 (트리거 규칙)
-- ---------------------------------------------------------------------
SELECT pg_temp.expect_ok  ('2-1 섹터 1 로봇(robot3)이 측정 중 네트워크 후보 발견 + CANDIDATE 이벤트',
    $q$INSERT INTO candidates (candidate_id, run_id, source, kind, ref, target_x, target_y, score, detect_task_id, detected_by, detected_at)
       VALUES ('v-c01', 900001, 'NETWORK', 'NET_COVERAGE', '900001', 3.5, 7.5, -78, 'v-t001', 'robot3', now());
       INSERT INTO robot_events (event_uid, robot_id, run_id, task_id, code, level, detail, ts)
       VALUES ('robot3-r900001-e000001', 'robot3', 900001, 'v-t001', 'CANDIDATE', 'warn', '{"candidate_id": "v-c01", "score": -78}', now())$q$);
SELECT pg_temp.expect_fail('2-2 같은 run·같은 칸 후보 중복 거부',
    $q$INSERT INTO candidates (candidate_id, run_id, source, kind, ref, target_x, target_y, detect_task_id, detected_by, detected_at)
       VALUES ('v-c08', 900001, 'NETWORK', 'NET_COVERAGE', '900001', 3.5, 7.5, 'v-t001', 'robot3', now())$q$);
SELECT pg_temp.expect_fail('2-3 detected_by가 발견 task의 로봇과 다르면 거부',
    $q$INSERT INTO candidates (candidate_id, run_id, source, kind, ref, target_x, target_y, detect_task_id, detected_by, detected_at)
       VALUES ('v-c07', 900001, 'NETWORK', 'NET_COVERAGE', '900002', 4.5, 7.5, 'v-t001', 'robot4', now())$q$);
SELECT pg_temp.expect_fail('2-4 REINSPECTION task를 발견 task로 쓰면 거부',
    $q$INSERT INTO candidates (candidate_id, run_id, source, kind, ref, target_x, target_y, detect_task_id, detected_by, detected_at)
       VALUES ('v-c06', 900001, 'NETWORK', 'NET_COVERAGE', '900002', 4.5, 7.5, 'v-t003', 'robot4', now())$q$);
SELECT pg_temp.expect_fail('2-5 CANDIDATE 이벤트의 로봇이 task 로봇과 다르면 거부',
    $q$INSERT INTO robot_events (event_uid, robot_id, run_id, task_id, code, ts) VALUES ('v-e93', 'robot4', 900001, 'v-t001', 'CANDIDATE', now())$q$);
SELECT pg_temp.expect_fail('2-6 재검증 전 확정(findings) 거부 — 재검증 없이 확정 0건',
    $q$INSERT INTO findings (candidate_id, task_id, category) VALUES ('v-c01', 'v-t001', 'coverage')$q$);
SELECT pg_temp.expect_fail('2-7 재검증 task 없이 REINSPECTING 거부',
    $q$UPDATE candidates SET status = 'REINSPECTING' WHERE candidate_id = 'v-c01'$q$);
SELECT pg_temp.expect_ok  ('2-8 발견한 robot3의 REINSPECTION task 연결 (같은 섹터에서 스스로 재검증)',
    $q$UPDATE candidates SET status = 'REINSPECTING', reinspect_task_id = 'v-t003' WHERE candidate_id = 'v-c01'$q$);

-- 설비 후보: 섹터 2 로봇 robot4가 설비 점검 중 발견, 재검증도 robot4
SELECT pg_temp.expect_ok  ('2-9 섹터 2 로봇(robot4)이 설비 점검 중 설비 후보 발견',
    $q$INSERT INTO candidates (candidate_id, run_id, source, kind, ref, target_x, target_y, score, detect_task_id, detected_by, detected_at)
       VALUES ('v-c02', 900001, 'VISION', 'VIS_OCCLUDED', 'v-ext-01', 2.0, 2.2, 0.81, 'v-t002', 'robot4', now())$q$);
SELECT pg_temp.expect_ok  ('2-10 발견한 로봇이 스스로 재검증 허용 (3차)',
    $q$UPDATE candidates SET status = 'REINSPECTING', reinspect_task_id = 'v-t004' WHERE candidate_id = 'v-c02'$q$);
SELECT pg_temp.expect_fail('2-11 재검증 task의 target_ref가 다른 후보를 가리키면 거부',
    $q$UPDATE candidates SET status = 'REINSPECTING', reinspect_task_id = 'v-t003' WHERE candidate_id = 'v-c02'$q$);

-- 재검증 로봇의 정지 측정 → 결론
SELECT pg_temp.expect_ok  ('2-12 robot3가 측정 task를 끝내고 재검증 task 시작 (로봇마다 한 번에 하나)',
    $q$UPDATE tasks SET status = 'DONE', result_ok = true, progress = 1, ended_at = now() WHERE task_id = 'v-t001';
       UPDATE tasks SET status = 'ACTIVE', assigned_at = now() WHERE task_id = 'v-t003'$q$);
SELECT pg_temp.expect_fail('2-13 1차 task로 정지 측정 저장 거부(REINSPECTION만)',
    $q$INSERT INTO precise_measures (task_id, robot_id, ap_id, cell_id, duration_s, n_samples, rssi_median, rssi_iqr, measured_at)
       VALUES ('v-t001', 'robot3', 9001, 900001, 30, 30, -77, 2, now())$q$);
SELECT pg_temp.expect_fail('2-14 정지 측정 robot_id가 task 로봇과 다르면 거부',
    $q$INSERT INTO precise_measures (task_id, robot_id, ap_id, cell_id, duration_s, n_samples, rssi_median, rssi_iqr, measured_at)
       VALUES ('v-t003', 'robot4', 9001, 900001, 30, 30, -77, 2, now())$q$);
SELECT pg_temp.expect_ok  ('2-15 재검증 정지 측정 저장',
    $q$INSERT INTO precise_measures (task_id, robot_id, ap_id, cell_id, duration_s, n_samples, rssi_median, rssi_iqr, measured_at)
       VALUES ('v-t003', 'robot3', 9001, 900001, 30, 30, -77, 2, now())$q$);
SELECT pg_temp.expect_fail('2-16 CLOSED인데 outcome 없음 거부',
    $q$UPDATE candidates SET status = 'CLOSED' WHERE candidate_id = 'v-c01'$q$);
SELECT pg_temp.expect_ok  ('2-17 재검증 로봇이 CONFIRMED 결론 + 이벤트',
    $q$UPDATE candidates SET status = 'CLOSED', outcome = 'CONFIRMED', decided_at = now() WHERE candidate_id = 'v-c01';
       INSERT INTO robot_events (event_uid, robot_id, run_id, task_id, code, level, detail, ts)
       VALUES ('robot3-r900001-e000002', 'robot3', 900001, 'v-t003', 'CONFIRMED', 'error', '{"candidate_id": "v-c01", "rssi_median": -77}', now());
       UPDATE tasks SET status = 'DONE', result_ok = true, progress = 1, ended_at = now() WHERE task_id = 'v-t003'$q$);
SELECT pg_temp.expect_fail('2-18 확정 task가 재검증 task가 아니면 거부',
    $q$INSERT INTO findings (candidate_id, task_id, category) VALUES ('v-c01', 'v-t001', 'coverage')$q$);
SELECT pg_temp.expect_ok  ('2-19 확정 이상(findings) 생성',
    $q$INSERT INTO findings (candidate_id, task_id, category) VALUES ('v-c01', 'v-t003', 'coverage')$q$);
SELECT pg_temp.expect_fail('2-20 같은 후보로 확정 이상 2건 거부',
    $q$INSERT INTO findings (candidate_id, task_id, category) VALUES ('v-c01', 'v-t003', 'coverage')$q$);
SELECT pg_temp.expect_fail('2-21 findings가 있는 확정 후보의 결론 변경 거부',
    $q$UPDATE candidates SET outcome = 'CLEARED' WHERE candidate_id = 'v-c01'$q$);
SELECT pg_temp.expect_fail('2-22 ack 하면서 확인자 없음 거부',
    $q$UPDATE findings SET status = 'acked' WHERE candidate_id = 'v-c01'$q$);
SELECT pg_temp.expect_ok  ('2-23 admin이 ack',
    $q$UPDATE findings SET status = 'acked', acked_by = (SELECT user_id FROM users WHERE username = 'admin') WHERE candidate_id = 'v-c01'$q$);

-- ---------------------------------------------------------------------
-- 3. 재전송 중복 방지 (안 C 아웃박스 — 같은 데이터가 다시 와도 한 번만 저장)
-- ---------------------------------------------------------------------
SELECT pg_temp.expect_ok  ('3-1 표본 재전송은 ON CONFLICT DO NOTHING으로 무시',
    $q$DO $d$ BEGIN
         INSERT INTO net_samples (robot_id, run_id, seq, stamp, x, y, yaw, pose_ok, ap_id, rssi_dbm, cell_id)
         VALUES ('robot3', 900001, 0, now(), 3.4, 7.6, 0, true, 9001, -78, 900001)
         ON CONFLICT (robot_id, run_id, seq) DO NOTHING;
         IF (SELECT count(*) FROM net_samples WHERE run_id = 900001) <> 3 THEN RAISE EXCEPTION '표본 수가 바뀜'; END IF;
       END $d$$q$);
SELECT pg_temp.expect_fail('3-2 같은 event_uid 이벤트 다시 저장 거부',
    $q$INSERT INTO robot_events (event_uid, robot_id, run_id, task_id, code, level, ts)
       VALUES ('robot3-r900001-e000001', 'robot3', 900001, 'v-t001', 'CANDIDATE', 'warn', now())$q$);
SELECT pg_temp.expect_ok  ('3-3 이벤트 재전송은 ON CONFLICT (event_uid) DO NOTHING으로 무시',
    $q$DO $d$ BEGIN
         INSERT INTO robot_events (event_uid, robot_id, run_id, task_id, code, level, ts)
         VALUES ('robot3-r900001-e000001', 'robot3', 900001, 'v-t001', 'CANDIDATE', 'warn', now())
         ON CONFLICT (event_uid) DO NOTHING;
         IF (SELECT count(*) FROM robot_events WHERE event_uid = 'robot3-r900001-e000001') <> 1 THEN RAISE EXCEPTION '이벤트가 중복 저장됨'; END IF;
       END $d$$q$);
SELECT pg_temp.expect_ok  ('3-4 설비 판정 저장',
    $q$INSERT INTO inspections (inspection_uid, task_id, run_id, robot_id, map_id, facility_id, detail, final, frontal, gauge_state, confidence, model_version, inspected_at)
       VALUES ('6f1d2c3e-0000-4000-8000-000000000001', 'v-t002', 900001, 'robot4', 900001, 'v-ext-01', 'OCCLUDED', 'unconfirmed', true, NULL, 0.81, 'yolo-v1', now())$q$);
SELECT pg_temp.expect_fail('3-5 같은 inspection_uid 판정 다시 저장 거부',
    $q$INSERT INTO inspections (inspection_uid, task_id, run_id, robot_id, map_id, facility_id, detail, final, frontal, model_version, inspected_at)
       VALUES ('6f1d2c3e-0000-4000-8000-000000000001', 'v-t002', 900001, 'robot4', 900001, 'v-ext-01', 'OCCLUDED', 'unconfirmed', true, 'yolo-v1', now())$q$);
SELECT pg_temp.expect_fail('3-6 같은 (task_id, ap_id) 정지 측정 다시 저장 거부',
    $q$INSERT INTO precise_measures (task_id, robot_id, ap_id, cell_id, duration_s, n_samples, rssi_median, rssi_iqr, measured_at)
       VALUES ('v-t003', 'robot3', 9001, 900001, 30, 30, -77, 2, now())$q$);
SELECT pg_temp.expect_ok  ('3-7 같은 사진(object_key) 재전송은 ON CONFLICT로 무시',
    $q$DO $d$ DECLARE v_id BIGINT; BEGIN
         SELECT inspection_id INTO v_id FROM inspections WHERE inspection_uid = '6f1d2c3e-0000-4000-8000-000000000001';
         INSERT INTO media (inspection_id, robot_id, kind, object_key, sha256, size_bytes, captured_at)
         VALUES (v_id, 'robot4', 'rgb', 'robot4/900001/6f1d2c3e-0000-4000-8000-000000000001/rgb.jpg', repeat('a', 64), 1024, now());
         INSERT INTO media (inspection_id, robot_id, kind, object_key, sha256, size_bytes, captured_at)
         VALUES (v_id, 'robot4', 'rgb', 'robot4/900001/6f1d2c3e-0000-4000-8000-000000000001/rgb.jpg', repeat('a', 64), 1024, now())
         ON CONFLICT (object_key) DO NOTHING;
         IF (SELECT count(*) FROM media WHERE inspection_id = v_id) <> 1 THEN RAISE EXCEPTION '사진이 중복 저장됨'; END IF;
       END $d$$q$);

-- ---------------------------------------------------------------------
-- 4. 실패·종료 처리 (3차 — 섹터별 실패)
--   robot4(섹터 2)의 task 실패 → robot4의 남은 task만 CANCELED, robot3(섹터 1)는 계속 → 끝나면 partial
-- ---------------------------------------------------------------------
SELECT pg_temp.expect_ok  ('4-1 scv_fail_task(v-t002, BLOCKED) — robot4의 남은 task 1개 취소',
    $q$DO $d$ BEGIN
         IF scv_fail_task('v-t002', 'BLOCKED') <> 1 THEN RAISE EXCEPTION '취소된 task 수가 1이 아님'; END IF;
       END $d$$q$);
SELECT pg_temp.expect_ok  ('4-2 그 섹터만 중단 — run은 계속, robot3 task는 그대로',
    $q$DO $d$ BEGIN
         IF (SELECT status FROM runs WHERE run_id = 900001) <> 'running' THEN RAISE EXCEPTION 'run이 running이 아님'; END IF;
         IF (SELECT status || ':' || reason FROM tasks WHERE task_id = 'v-t004') <> 'CANCELED:SECTOR_ABORTED:v-t002'
         THEN RAISE EXCEPTION 'robot4의 남은 task가 취소되지 않음'; END IF;
         IF (SELECT count(*) FROM tasks WHERE run_id = 900001 AND robot_id = 'robot3' AND status = 'CANCELED') <> 0
         THEN RAISE EXCEPTION 'robot3 task가 취소됨'; END IF;
       END $d$$q$);
SELECT pg_temp.expect_ok  ('4-3 robot3 DOCK이 남아 있으면 scv_close_run은 종료하지 않음(NULL)',
    $q$DO $d$ BEGIN
         INSERT INTO tasks (task_id, run_id, robot_id, type, status, assigned_at) VALUES ('v-t005', 900001, 'robot3', 'DOCK', 'ACTIVE', now());
         IF scv_close_run(900001) IS NOT NULL THEN RAISE EXCEPTION '남은 task가 있는데 종료됨'; END IF;
       END $d$$q$);
SELECT pg_temp.expect_ok  ('4-4 robot3 DOCK 완료 → scv_close_run = partial (일부 섹터 실패)',
    $q$DO $d$ BEGIN
         UPDATE tasks SET status = 'DONE', result_ok = true, progress = 1, ended_at = now() WHERE task_id = 'v-t005';
         IF scv_close_run(900001) IS DISTINCT FROM 'partial' THEN RAISE EXCEPTION 'partial이 아님'; END IF;
         IF (SELECT ended_at IS NULL OR is_baseline_ok FROM runs WHERE run_id = 900001) THEN RAISE EXCEPTION '종료 값이 틀림'; END IF;
       END $d$$q$);
SELECT pg_temp.expect_fail('4-5 partial 회차를 기준선 회차로 표시 거부',
    $q$UPDATE runs SET is_baseline_ok = true WHERE run_id = 900001$q$);
SELECT pg_temp.expect_fail('4-6 끝난 run을 다시 종료 거부',
    $q$SELECT scv_close_run(900001)$q$);
SELECT pg_temp.expect_ok  ('4-7 실패가 없으면 done',
    $q$DO $d$ BEGIN
         INSERT INTO runs (run_id, map_id, run_type, sector_robots) VALUES (900002, 900001, 'patrol', '{"1": "robot3", "2": "robot4"}');
         INSERT INTO tasks (task_id, run_id, robot_id, type, status, result_ok, ended_at) VALUES
             ('v2-t001', 900002, 'robot3', 'DOCK', 'DONE', true, now()),
             ('v2-t002', 900002, 'robot4', 'DOCK', 'DONE', true, now());
         IF scv_close_run(900002) IS DISTINCT FROM 'done' THEN RAISE EXCEPTION 'done이 아님'; END IF;
       END $d$$q$);
SELECT pg_temp.expect_ok  ('4-8 배정된 섹터가 모두 실패하면 aborted',
    $q$DO $d$ BEGIN
         INSERT INTO runs (run_id, map_id, run_type, sector_robots) VALUES (900003, 900001, 'patrol', '{"1": "robot3"}');
         INSERT INTO tasks (task_id, run_id, robot_id, type, target_ref, status, assigned_at) VALUES
             ('v3-t001', 900003, 'robot3', 'SURVEY_SEGMENT', 'seg:0-20', 'ACTIVE', now()),
             ('v3-t002', 900003, 'robot3', 'DOCK', NULL, 'PENDING', NULL);
         PERFORM scv_fail_task('v3-t001', 'BATTERY_LOW');
         IF scv_close_run(900003) IS DISTINCT FROM 'aborted' THEN RAISE EXCEPTION 'aborted가 아님'; END IF;
       END $d$$q$);
SELECT pg_temp.expect_ok  ('4-9 scv_abort_run(비상정지) — 두 로봇 task 모두 취소 + aborted',
    $q$DO $d$ BEGIN
         INSERT INTO runs (run_id, map_id, run_type, sector_robots) VALUES (900004, 900001, 'patrol', '{"1": "robot3", "2": "robot4"}');
         INSERT INTO tasks (task_id, run_id, robot_id, type, target_ref, status, assigned_at) VALUES
             ('v4-t001', 900004, 'robot3', 'SURVEY_SEGMENT', 'seg:0-20', 'ACTIVE', now()),
             ('v4-t002', 900004, 'robot4', 'SURVEY_SEGMENT', 'seg:0-20', 'ACTIVE', now());
         IF scv_abort_run(900004, 'ESTOP') <> 2 THEN RAISE EXCEPTION '취소 수가 2가 아님'; END IF;
         IF (SELECT status FROM runs WHERE run_id = 900004) <> 'aborted' THEN RAISE EXCEPTION 'aborted가 아님'; END IF;
       END $d$$q$);
SELECT pg_temp.expect_ok  ('4-10 중단 뒤 새 run 시작 가능',
    $q$INSERT INTO runs (map_id, run_type, sector_robots) VALUES (900001, 'patrol', '{"1": "robot3"}')$q$);

-- ---------------------------------------------------------------------
-- 5. 계정 · 설정 — 수퍼 계정 admin 보호, 일반 계정 관리, settings 설명
-- ---------------------------------------------------------------------
SELECT pg_temp.expect_ok  ('5-1 수퍼 계정 admin 하나 · 권한 admin',
    $q$DO $d$ BEGIN
         IF (SELECT count(*) FROM users WHERE role = 'admin') <> 1 THEN RAISE EXCEPTION 'admin 권한 계정 수가 1이 아님'; END IF;
         IF (SELECT role FROM users WHERE username = 'admin') IS DISTINCT FROM 'admin' THEN RAISE EXCEPTION 'admin 계정이 없거나 권한이 다름'; END IF;
       END $d$$q$);
SELECT pg_temp.expect_ok  ('5-2 일반 계정(user) 추가',
    $q$INSERT INTO users (username, password_hash, role) VALUES ('v_user1', crypt('pw1', gen_salt('bf', 4)), 'user')$q$);
SELECT pg_temp.expect_ok  ('5-3 일반 계정 이름·비밀번호 수정',
    $q$UPDATE users SET username = 'v_user2', password_hash = crypt('pw2', gen_salt('bf', 4)) WHERE username = 'v_user1'$q$);
SELECT pg_temp.expect_ok  ('5-4 일반 계정 삭제',
    $q$DELETE FROM users WHERE username = 'v_user2'$q$);
SELECT pg_temp.expect_ok  ('5-5 admin 비밀번호 변경',
    $q$UPDATE users SET password_hash = crypt('new-admin-pw', gen_salt('bf', 4)) WHERE username = 'admin'$q$);
SELECT pg_temp.expect_fail('5-6 admin 삭제 거부',
    $q$DELETE FROM users WHERE username = 'admin'$q$);
SELECT pg_temp.expect_fail('5-7 admin 이름 변경 거부',
    $q$UPDATE users SET username = 'root' WHERE username = 'admin'$q$);
SELECT pg_temp.expect_fail('5-8 admin 권한을 user로 변경 거부',
    $q$UPDATE users SET role = 'user' WHERE username = 'admin'$q$);
SELECT pg_temp.expect_fail('5-9 다른 이름의 admin 권한 계정 추가 거부',
    $q$INSERT INTO users (username, password_hash, role) VALUES ('admin2', 'x', 'admin')$q$);
SELECT pg_temp.expect_fail('5-10 이름이 admin인 계정 추가 거부',
    $q$INSERT INTO users (username, password_hash, role) VALUES ('admin', 'x', 'user')$q$);
INSERT INTO users (username, password_hash, role) VALUES ('v_user3', 'x', 'user');
SELECT pg_temp.expect_fail('5-11 일반 계정을 admin 권한으로 변경 거부',
    $q$UPDATE users SET role = 'admin' WHERE username = 'v_user3'$q$);
SELECT pg_temp.expect_fail('5-12 권한 viewer(이전 판 값) 거부',
    $q$INSERT INTO users (username, password_hash, role) VALUES ('v_viewer', 'x', 'viewer')$q$);
SELECT pg_temp.expect_ok  ('5-13 settings 초기 key 13개 모두 설명(description) 있음',
    $q$DO $d$ BEGIN
         IF (SELECT count(*) FROM settings WHERE description IS NOT NULL AND description <> '') < 13 THEN
             RAISE EXCEPTION '설명이 빈 설정이 있음';
         END IF;
       END $d$$q$);
SELECT pg_temp.expect_ok  ('5-14 로봇·AP는 보고 전 비활성 (is_active 기본값 false)',
    $q$DO $d$ BEGIN
         INSERT INTO robots (robot_id) VALUES ('v_robot');
         IF (SELECT is_active FROM robots WHERE robot_id = 'v_robot') THEN RAISE EXCEPTION '로봇 기본값이 true'; END IF;
         IF (SELECT is_active FROM aps WHERE ap_id = 9001) THEN RAISE EXCEPTION 'AP 기본값이 true'; END IF;
       END $d$$q$);

-- ---------------------------------------------------------------------
-- 결과
-- ---------------------------------------------------------------------
\o
SELECT no, result, item, message FROM v_result ORDER BY no;

DO $$
DECLARE
    n_fail INT;
    n_all  INT;
BEGIN
    SELECT count(*) FILTER (WHERE result = 'FAIL'), count(*) INTO n_fail, n_all FROM v_result;
    IF n_fail > 0 THEN
        RAISE EXCEPTION '검증 실패: % / %건', n_fail, n_all;
    END IF;
    RAISE NOTICE '검증 통과: %건 모두 PASS', n_all;
END $$;

ROLLBACK;
