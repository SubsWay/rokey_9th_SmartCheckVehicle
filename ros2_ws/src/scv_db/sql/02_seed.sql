-- =====================================================================
-- SCV 운영 DB 초기값 (3차)
--   01_schema.sql 다음에 실행. 여러 번 실행해도 값이 겹치지 않음(ON CONFLICT).
--   이미 있는 설정값은 덮어쓰지 않음 — 운영 중 admin이 바꾼 값을 보존.
-- =====================================================================

BEGIN;

-- 비밀번호 해시(bcrypt)용. postgres:16 공식 이미지에 포함된 확장
CREATE EXTENSION IF NOT EXISTS pgcrypto;

-- ---------------------------------------------------------------------
-- 로봇 2대 (ROS 이름공간 /robot3, /robot4)
--   역할(1차 순찰 / 재검증)은 순찰 시작 때 task를 만들며 정해지므로 여기에는 두지 않음
--   활성(is_active)은 기본값 false — client(제어 PC)가 값을 보내 줄 때만 true
-- ---------------------------------------------------------------------
INSERT INTO robots (robot_id, robot_pc_host, rpi_host) VALUES
    ('robot3', NULL, NULL),
    ('robot4', NULL, NULL)
ON CONFLICT (robot_id) DO NOTHING;

-- ---------------------------------------------------------------------
-- 판정 기준값 (문서의 「제안」 값 — 첫 측정 뒤 server page 설정 탭에서 조정)
--   description = key 설명. 이전 판 DB처럼 설명이 비어 있는 행에만 채움(값·단위·근거는 건드리지 않음)
-- ---------------------------------------------------------------------
INSERT INTO settings (key, value, unit, source, description) VALUES
    -- 로봇이 실시간 판단에 쓰는 값 (순찰 시작 때 두 제어 PC로 전달)
    ('rssi_suspect_dbm',     '-72',  'dBm', 'BR-T04', '약한 신호 기준 — 검출 RSSI 중앙값이 이 값 미만이면 약한 신호, 재검증 정지 측정도 같은 기준으로 CONFIRMED/CLEARED'),
    ('min_samples',          '3',    NULL,  'SR-06',  '후보 판단 최소 표본 수 — 칸 표본이 이보다 적으면 후보를 내지 않음(보고서 신뢰도 「낮음」 기준과 같음)'),
    ('precise_duration_s',   '15',   's',   'SR-08',  '재검증 정지 측정 시간 — 재검증 로봇이 후보 칸에 멈춰 RSSI를 재는 시간'),
    ('max_pose_retry',       '3',    NULL,  'SR-21',  '설비 정면 자세 보정 최대 횟수 — 넘기면 미확인(E6)'),
    ('battery_return_pct',   '15',   '%',   'E2',     '배터리 복귀 기준 — 잔량이 이 값 이하이면 복귀(E2)'),
    -- 경로 생성 (inspection_manager)
    ('path_spacing_m',       '2.0',  'm',   'BR-T01', '순찰 왕복 경로 간격 — 자동 생성 경로의 줄 사이 거리'),
    ('wall_margin_m',        '0.3',  'm',   'SR-05',  '벽 여유 거리 — 순찰 경로를 만들 때 벽에서 띄우는 거리(격자 칸에는 쓰지 않음)'),
    ('segment_points',       '20',   NULL,  'SR-05',  'SURVEY_SEGMENT 구간 점 수 — 순찰 경로를 task 하나에 담는 점 개수'),
    -- 관제 PC
    ('status_timeout_s',     '30',   's',   'E3',     '상태 수신 제한 시간 — 로봇 상태가 이 시간 동안 안 오면 응답 없음(E3). 관제 PC가 1초마다 확인'),
    -- 보고서 (실시간 판단에는 쓰지 않음)
    ('baseline_runs_min',    '3',    NULL,  'SR-07',  '기준선 최소 회차 수 — 정상 회차가 이만큼 모여야 기준선을 만듦'),
    ('baseline_sigma',       '2',    NULL,  'SR-07',  '기준선 이탈 계수 k — 평균 − kσ 미만이면 이탈(보고서)'),
    ('baseline_drop_db',     '6',    'dB',  'SR-07',  '기준선 이탈 폭 — 평균 − N dB 미만이면 이탈(보고서, baseline_sigma와 둘 중 하나로 통일)'),
    ('photo_retention_days', '30',   'day', 'BR-R05', '정상 판정 사진 보관 기간 — 지난 사진은 정리 대상')
ON CONFLICT (key) DO UPDATE SET description = EXCLUDED.description
    WHERE settings.description IS NULL;

-- 이전 판 설명 바로잡기 — 격자 칸은 벽 여유를 쓰지 않음 (설명을 손대지 않은 경우만)
UPDATE settings SET description = '벽 여유 거리 — 순찰 경로를 만들 때 벽에서 띄우는 거리(격자 칸에는 쓰지 않음)'
 WHERE key = 'wall_margin_m' AND description = '벽 여유 거리 — 경로·격자 칸을 만들 때 벽에서 띄우는 거리';
UPDATE settings SET description = '상태 수신 제한 시간 — 로봇 상태가 이 시간 동안 안 오면 응답 없음(E3). 관제 PC가 1초마다 확인'
 WHERE key = 'status_timeout_s' AND description = '상태 수신 제한 시간 — 로봇 상태가 이 시간 동안 안 오면 응답 없음(E3). Redis robot:{rid}:state TTL과 같음';

-- ---------------------------------------------------------------------
-- 수퍼 계정 admin / admin (권한 admin — server page · client page 접속)
--   비밀번호만 바꿀 수 있고 삭제·추가·이름·권한 변경은 DB가 거부(01_schema.sql 규칙 E)
--   일반 클라이언트 계정(권한 user — client page만)은 server page 설정 탭에서 추가
--   비밀번호는 bcrypt 해시로만 저장 ($2a$ 형식 — Python bcrypt 등에서도 그대로 검증 가능)
-- ---------------------------------------------------------------------
INSERT INTO users (username, password_hash, role) VALUES
    ('admin', crypt('admin', gen_salt('bf', 12)), 'admin')
ON CONFLICT (username) DO NOTHING;

-- ---------------------------------------------------------------------
-- 측정 대상 AP — SSID를 실제 값으로 바꾼 뒤 주석을 풀어 실행
-- ---------------------------------------------------------------------
-- INSERT INTO aps (ssid, note) VALUES ('SCV_HOTSPOT_A', '핫스팟 A')
-- ON CONFLICT (ssid) DO NOTHING;

-- 시험용 더미 데이터는 dummy/make_dummy.py (README 5장)
-- 지도(maps)·격자(grid_cells)·설비(facilities)는 SLAM 지도를 만든 뒤
-- server page 설정 탭(또는 api 등록 스크립트)에서 등록 — 예시는 03_verify.sql 참고

COMMIT;
