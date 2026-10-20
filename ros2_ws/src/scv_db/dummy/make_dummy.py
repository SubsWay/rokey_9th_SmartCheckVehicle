#!/usr/bin/env python3
"""
SCV 운영 DB 더미 데이터 생성 (3차)

  01_schema.sql · 02_seed.sql을 적용한 DB에 화면·쿼리 시험용 더미 데이터를 넣습니다.
  기준 테이블은 50행 안팎, 나머지는 DB 규칙(트리거·CHECK)을 지키는 범위에서 그 데이터에 맞춰 생깁니다.

  사용 (관제 PC):
    python make_dummy.py              # 비어 있는 DB에 생성 (--media 기본 ../../scv_app/scv_media)
    python make_dummy.py --reset      # 기존 데이터를 지우고 다시 생성

  옵션 · 환경 변수
    --dsn     DB 접속 문자열 (기본 SCV_DB_DSN 또는 postgresql://rokey:rokey@localhost:5432/robotdb4_postgresql)
    --media   파일 저장 위치 = server_page.py의 SCV_MEDIA_DIR
              (기본: 이 스크립트 기준 ../../scv_app/scv_media — server_page.py 기본 위치, 실행한 폴더와 상관없음.
               환경 변수 SCV_MEDIA_DIR이 있으면 그 값 — server_page.py와 같은 변수)
              지도 PNG · 사진 · 보고서 PDF를 이 아래에 실제 파일로 만듦. server와 다른 곳이면 지도 · 사진이 화면에 안 보임
    --n       기준 개수 (기본 50) — users · aps · facilities · runs
    --seed    난수 시드 (기본 2026) — 같은 시드면 같은 데이터
    --reset   robots · settings · 수퍼 계정 admin만 남기고 모두 지운 뒤 생성 (지운 행이 가리키던 파일도 삭제)
    --map-dir 실제 지도 폴더 (기본 REAL_MAP_DIR). 이 폴더의 yaml + pgm 하나를 더미 지도 중 하나로 등록
    --map-yaml 그 폴더에서 쓸 yaml 파일 이름 (기본: 이름순 첫 번째 *.yaml)
    --no-real-map  실제 지도 대신 합성 지도(dummy_floor_b)를 씀
    --map-out 더미 지도 파일(yaml + pgm)을 쓸 폴더 (기본: 이 스크립트 옆 maps/)
              server page 정보 › 지도 › 지도 등록으로 다시 올릴 수 있는 파일 (3-3차)

  필요 패키지: psycopg[binary] · numpy · pillow · pyyaml (server_page.py와 같음)

  예외로 개수를 고정한 표
    robots    2  (robot3 · robot4 — 02_seed.sql 그대로, 보고가 없으므로 비활성)
    settings  13 (02_seed.sql 그대로, 값을 바꾸지 않음)
    maps      3  (합성 2 + 실제 지도 1, PNG 파일 생성, 합성 dummy_testbed_a만 현재 지도)
    grid_cells    현재 지도 1개에만 생성 — server page에서 지도를 선택할 때 만드는 방식과 같음
                  (지도의 흰 부분이 걸치는 칸은 모두. 벽 여유로 빼지 않음 · 섹터 2개로 나눔)

  순찰(3차): 섹터 1 = robot3 · 섹터 2 = robot4가 각자 자기 섹터에서 동시에 운행
    회차마다 주 task(측정 또는 설비 점검)를 맡는 로봇을 번갈아 두고, 다른 로봇은 자기 섹터 측정 1구간 + DOCK
    여덟 번째 회차마다 robot3 혼자(섹터 1만 배정 — 비활성 로봇의 섹터는 순찰하지 않음)
    partial = 주 로봇의 섹터만 실패(다른 로봇은 끝까지) · aborted = 혼자 운행한 회차의 실패

  활성 여부: robots · aps는 client(제어 PC)·로봇이 값을 보내 줄 때만 활성이므로 더미에서는 모두 비활성
  측정 AP(aps.is_target, 3-3차): 지도마다 SCV_AP_01 하나 (화면에서는 활성 AP만 고를 수 있지만 더미는 보고가 없어 비활성 상태로 지정)
  3-3차 기록 모양(이력 「자세히」가 보여 줄 수 있게)
    측정 경로   섹터 칸 중심을 지그재그로(server plan_survey와 같은 순서) 20점씩 — seg:a-b
    표본        모든 측정 task가 경로를 따라 칸마다 3 ~ 4개, 측정 AP 하나, task_id. 네트워크 후보 칸은 그 task가 지난 칸
    정밀측정    측정 AP 하나
    status 기록 모든 task 동안 경로를 따라 위치 · 배터리 · task_id (task마다 최대 40점)
    DOCK        경로점 = 그 로봇의 도크 접근 자세
"""
from __future__ import annotations

import argparse
import hashlib
import io
import math
import os
import random
import sys
import uuid
from datetime import datetime, timedelta, timezone
from pathlib import Path

import numpy as np
import psycopg
from PIL import Image, ImageDraw
from psycopg.rows import dict_row
from psycopg.types.json import Jsonb

DEFAULT_DSN = "postgresql://rokey:rokey@localhost:5432/robotdb4_postgresql"
REAL_MAP_DIR = "/home/rokey/ws_cobot_pjt/5-4_Project_04/maps"

# server_page.py와 같은 흰 부분(빈 공간) 판정 (yaml 기본 free 임계값, negate = 0)
FREE_THRESH = 0.25
UNKNOWN_PIXEL = 205
FREE_PIXEL, WALL_PIXEL = 254, 0

# 섹터 ↔ 로봇 기본 배정(3차). 재검증은 후보를 발견한 로봇이 자기 섹터에서 스스로 수행
SECTOR_ROBOTS = {"1": "robot3", "2": "robot4"}
MODEL_VERSION = "yolov8n-scv-1.2"
RSSI_SUSPECT = -72
SEGMENT_POINTS = 20                    # settings.segment_points 기본값 — 측정 task 하나의 경로점 수
STATUS_POINTS = 40                     # 더미 status 기록 — task마다 최대 점 수

# 운영 데이터 표 — 지우는 순서(참조하는 쪽 먼저)
DATA_TABLES = ["reports", "robot_status_log", "robot_events", "findings", "candidates", "media", "inspections",
               "baselines", "grid_stats", "precise_measures", "net_samples", "tasks", "runs",
               "facilities", "grid_cells", "maps", "aps"]


# ---------------------------------------------------------------------------
# 지도 — SLAM pgm과 같은 값(빈 254 · 벽 0 · 미탐색 205)으로 평면도를 그림
# ---------------------------------------------------------------------------
def floor_plan(width_m: float, height_m: float, res: float, walls: list[tuple[float, float, float, float]],
               border_m: float = 1.0) -> np.ndarray:
    """바깥 border_m은 미탐색, 그 안쪽이 방. walls는 방 안쪽 좌표(m) 기준 (x0, y0, x1, y1) 사각형."""
    W = int(round((width_m + 2 * border_m) / res))
    H = int(round((height_m + 2 * border_m) / res))
    img = np.full((H, W), UNKNOWN_PIXEL, np.uint8)
    b = int(round(border_m / res))
    t = max(1, int(round(0.1 / res)))                    # 벽 두께 10 cm
    img[b - t:H - b + t, b - t:W - b + t] = WALL_PIXEL
    img[b:H - b, b:W - b] = FREE_PIXEL

    def to_px(x, y):                                      # 방 좌표(m, 왼쪽 아래 원점) → 픽셀 (col, row)
        return b + int(round(x / res)), H - b - int(round(y / res))

    for x0, y0, x1, y1 in walls:
        c0, r1 = to_px(x0, y0)
        c1, r0 = to_px(x1, y1)
        img[max(r0, 0):max(r1, r0 + 1), max(c0, 0):max(c1, c0 + 1)] = WALL_PIXEL
    return img


def grid_from_image(raw: np.ndarray, res: float, ox: float, oy: float, g: float) -> np.ndarray:
    """server_page.generate_cells와 같은 계산 — 흰 부분이 조금이라도 걸치는 칸 (ix, iy) 목록 (벽 여유로 빼지 않음)."""
    occ = 1.0 - raw.astype(np.float32) / 255.0
    free = (occ < FREE_THRESH) & (raw != UNKNOWN_PIXEL)
    rows, cols = np.nonzero(free)
    h = raw.shape[0]
    x = ox + (cols + 0.5) * res
    y = oy + (h - rows - 0.5) * res
    ix = np.floor((x - ox) / g).astype(np.int64)
    iy = np.floor((y - oy) / g).astype(np.int64)
    return np.unique(np.stack([ix, iy], axis=1), axis=0)


def split_sectors(cells) -> dict[tuple[int, int], int]:
    """server_page.split_sectors와 같은 임시 규칙 — ix 순으로 칸 수가 절반에 가장 가까운 열에서 나눔.
    왼쪽(그 열까지) = 섹터 1, 오른쪽 = 섹터 2. 열이 하나뿐이면 모두 섹터 1."""
    cells = [(int(a), int(b)) for a, b in cells]
    cols: dict[int, int] = {}
    for ix, _ in cells:
        cols[ix] = cols.get(ix, 0) + 1
    order = sorted(cols)
    total, left, split, best = len(cells), 0, None, None
    for ix in order[:-1]:
        left += cols[ix]
        diff = abs(2 * left - total)
        if best is None or diff < best:
            best, split = diff, ix
    return {c: (1 if split is None or c[0] <= split else 2) for c in cells}


MAPS = [
    # name, 방 크기(m), 벽(방 좌표), 현재 지도
    ("dummy_testbed_a", 9.0, 6.0, [(5.0, 0.0, 5.1, 4.0),           # 칸막이 (위쪽 2 m는 통로)
                                   (1.5, 3.5, 2.5, 4.5),           # 기둥
                                   (7.0, 1.0, 8.0, 1.6)], True),   # 선반
    ("dummy_floor_b", 14.0, 8.0, [(4.0, 0.0, 4.1, 5.5), (9.0, 2.5, 9.1, 8.0), (11.0, 1.0, 12.5, 1.8)], False),
    ("dummy_lab_c", 6.0, 5.0, [(2.5, 2.0, 3.5, 3.0)], False),
]
RES, GRID = 0.05, 1.0
REAL_MAP_SLOT = "dummy_floor_b"      # 실제 지도로 바꿔 넣는 합성 지도


def load_real_map(map_dir: str, yaml_name: str | None) -> dict:
    """실제 SLAM 지도(yaml + pgm)를 읽음 — server page 지도 등록과 같은 확인(yaw = 0, negate = 0)."""
    import yaml                                            # server_page.py와 같은 pyyaml
    d = Path(map_dir).expanduser()
    if not d.is_dir():
        raise SystemExit(f"실제 지도 폴더를 찾지 못했습니다: {d}\n"
                         "  --map-dir로 폴더를 지정하거나, 합성 지도만 쓰려면 --no-real-map을 붙이세요")
    if yaml_name:
        yp = d / yaml_name
    else:
        found = sorted(list(d.glob("*.yaml")) + list(d.glob("*.yml")))
        if not found:
            raise SystemExit(f"{d}에 yaml 파일이 없습니다 (--map-yaml로 지정하거나 --no-real-map)")
        yp = found[0]
    if not yp.is_file():
        raise SystemExit(f"yaml 파일이 없습니다: {yp}")
    try:
        meta = yaml.safe_load(yp.read_text(encoding="utf-8"))
        res = float(meta["resolution"])
        ox, oy, oyaw = (float(v) for v in meta["origin"][:3])
        img_path = Path(str(meta["image"]))
    except Exception as e:
        raise SystemExit(f"{yp.name}에서 image · resolution · origin을 읽지 못했습니다: {e}")
    if abs(oyaw) > 1e-9 or int(meta.get("negate", 0) or 0) != 0:
        raise SystemExit(f"{yp.name}: origin yaw = 0, negate = 0인 지도만 쓸 수 있습니다 (server page 등록 규칙과 같음)")
    if not img_path.is_absolute():
        img_path = yp.parent / img_path
    if not img_path.is_file():
        raise SystemExit(f"{yp.name}이 가리키는 지도 이미지가 없습니다: {img_path}")
    with Image.open(img_path) as im:
        raw = np.asarray(im.convert("L"))
    print(f"실제 지도: {yp} (+ {img_path.name}, {raw.shape[1]} × {raw.shape[0]} px, {res} m/px)")
    return {"name": yp.stem, "yaml": yp.name, "raw": raw, "res": res, "ox": ox, "oy": oy,
            }


# ---------------------------------------------------------------------------
# 파일 — 사진(jpg · png) · 보고서(pdf). 내용은 확인용 표시만
# ---------------------------------------------------------------------------
def png_bytes(arr_or_img) -> bytes:
    img = arr_or_img if isinstance(arr_or_img, Image.Image) else Image.fromarray(arr_or_img)
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    return buf.getvalue()


def photo_rgb(label: str, abnormal: bool, rnd: random.Random) -> bytes:
    img = Image.new("RGB", (320, 240), (200, 60, 50) if abnormal else (60, 140, 120))
    d = ImageDraw.Draw(img)
    x0, y0 = rnd.randint(60, 140), rnd.randint(40, 90)
    d.rectangle([x0, y0, x0 + 110, y0 + 120], outline=(255, 255, 255), width=3)
    d.text((10, 10), "SCV dummy photo", fill=(255, 255, 255))
    d.text((10, 220), label, fill=(255, 255, 255))
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=70)
    return buf.getvalue()


def photo_depth(rnd: random.Random) -> bytes:
    yy, xx = np.mgrid[0:120, 0:160]
    arr = ((xx * 1.2 + yy * 0.6 + rnd.randint(0, 60)) % 255).astype(np.uint8)
    return png_bytes(arr)


def report_pdf(lines: list[str]) -> bytes:
    img = Image.new("RGB", (595, 842), "white")         # A4 72 dpi
    d = ImageDraw.Draw(img)
    for i, line in enumerate(lines):
        d.text((48, 60 + i * 22), line, fill="black")
    buf = io.BytesIO()
    img.save(buf, format="PDF")
    return buf.getvalue()


# ---------------------------------------------------------------------------
# 생성
# ---------------------------------------------------------------------------
class Dummy:
    def __init__(self, conn: psycopg.Connection, media: Path, n: int, rnd: random.Random, real_map: dict | None):
        self.c, self.media_dir, self.n, self.r, self.real_map = conn, media, n, rnd, real_map
        self.files: dict[str, bytes] = {}                 # media 기준 경로 → 내용 (커밋 뒤 저장)
        self.counts: dict[str, int] = {}
        today = datetime.now(timezone.utc).replace(hour=1, minute=0, second=0, microsecond=0)   # 10:00 KST
        self.day0 = today - timedelta(days=n)

    def q(self, sql, params=None):
        return self.c.execute(sql, params)

    def count(self, table, k):
        self.counts[table] = self.counts.get(table, 0) + k

    # ---------------- 기준 정보 ----------------
    def users(self):
        self.admin_id = self.q("SELECT user_id FROM users WHERE username = 'admin'").fetchone()["user_id"]
        rows = []
        for i in range(1, self.n + 1):
            created = self.day0 - timedelta(days=self.r.randint(1, 30), minutes=self.r.randint(0, 600))
            rows.append((f"user{i:02d}", "user1234", created))
        with self.c.cursor() as cur:
            cur.executemany("INSERT INTO users (username, password_hash, role, created_at) "
                            "VALUES (%s, crypt(%s, gen_salt('bf', 8)), 'user', %s)", rows)
        self.count("users", len(rows))

    def aps(self):
        """AP는 지도에 속함(3차). 현재 지도에 SCV_AP_01 ~ 40, 다른 두 지도에는 같은 SSID 01 ~ 05를 다시 등록
        (지도마다 SSID는 하나 — 같은 SSID가 지도마다 다른 ap_id). 활성은 보고로만 정해지므로 모두 비활성"""
        places = ["1층 복도", "2층 복도", "사무실 A", "사무실 B", "회의실", "창고", "로비", "계단실", "휴게실", "서버실"]
        self.ap_ids, self.active_aps = [], []
        w, h = self.room
        n_cur = max(5, self.n - 10)
        for i in range(1, n_cur + 1):
            measured = i <= 5                              # 앞 5개는 과거 측정 기록에 쓰인 AP — 설치 위치도 있음
            note = f"{'측정 대상' if measured else '예비'} — {places[(i - 1) % len(places)]}"
            x = round(0.8 + (w - 1.6) * (i - 1) / 4, 2) if measured else None
            y = (h - 0.8 if i % 2 == 0 else 0.8 + h / 2) if measured else None
            ap_id = self.q("INSERT INTO aps (map_id, ssid, note, x, y, is_active, is_target) VALUES (%s, %s, %s, %s, %s, false, %s) "
                           "RETURNING ap_id", (self.map_id, f"SCV_AP_{i:02d}", note, x, y, i == 1)).fetchone()["ap_id"]
            self.ap_ids.append(ap_id)
            if measured:
                self.active_aps.append(ap_id)
            if i == 1:                                     # 측정 AP (3-3차) — 측정 표본 · 정밀측정은 이 AP만
                self.target_ap, self.target_xy = ap_id, (x, y)
        n_other = 0
        for name, mid in self.map_ids.items():
            if mid == self.map_id:
                continue
            for i in range(1, 6):
                self.q("INSERT INTO aps (map_id, ssid, note, is_target) VALUES (%s, %s, %s, %s)",
                       (mid, f"SCV_AP_{i:02d}", f"{name} — 같은 SSID를 이 지도에도 등록", i == 1))
                n_other += 1
        self.q("UPDATE robots SET is_active = false WHERE is_active")    # 로봇도 보고 전이므로 비활성
        self.count("aps", n_cur + n_other)

    def maps(self):
        self.map_ids = {}
        self.map_files: dict[str, bytes] = {}             # 파일 이름 → 내용 (--map-out 폴더)
        for idx, (name, w, h, walls, active) in enumerate(MAPS):
            created = self.day0 - timedelta(days=40 - idx)
            if name == REAL_MAP_SLOT and self.real_map:      # 실제 SLAM 지도 (현재 지도 아님 → 격자 없음)
                rm = self.real_map
                key = "maps/dummy_real_" + "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in rm["name"]) + ".png"
                self.files[key] = png_bytes(rm["raw"])        # pgm을 크기 그대로 PNG로 (server page 등록과 같음)
                self.map_files.update(map_file_pair(rm["name"], rm["raw"], rm["res"], rm["ox"], rm["oy"]))
                real_id = self.q(
                    "INSERT INTO maps (name, robot_map_path, image_key, resolution_m, origin_x, origin_y, grid_size_m, "
                    "dock_poses, is_active, created_at) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, false, %s) RETURNING map_id",
                    (rm["name"], f"/opt/scv/maps/{rm['yaml']}", key, rm["res"], rm["ox"], rm["oy"], GRID,
                     Jsonb({}), created)).fetchone()["map_id"]    # 도크 접근 자세는 server page에서 입력 — 실제 지도는 비워 둠
                self.map_ids[rm["name"]] = real_id
                continue
            raw = floor_plan(w, h, RES, walls)
            ox, oy = -1.0, -1.0                            # 방 왼쪽 아래 모서리 = map (0, 0)
            key = f"maps/{name}.png"
            self.files[key] = png_bytes(raw)
            self.map_files.update(map_file_pair(name, raw, RES, ox, oy))   # 다시 등록할 수 있는 yaml + pgm
            # 도크 접근 자세(server page에서 입력하는 값) — robot3는 왼쪽 아래(섹터 1), robot4는 오른쪽 아래(섹터 2)
            docks = {"robot3": {"x": 1.0, "y": 0.6, "yaw": 1.5708}, "robot4": {"x": w - 1.0, "y": 0.6, "yaw": 1.5708}}
            map_id = self.q(
                "INSERT INTO maps (name, robot_map_path, image_key, resolution_m, origin_x, origin_y, grid_size_m, "
                "dock_poses, is_active, created_at) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s) RETURNING map_id",
                (name, f"/opt/scv/maps/{name}.yaml", key, RES, ox, oy, GRID, Jsonb(docks), active,
                 created)).fetchone()["map_id"]
            self.map_ids[name] = map_id
            if active:
                self.map_id = map_id
                self.dock_poses = docks
                cells = grid_from_image(raw, RES, ox, oy, GRID)
                sectors = split_sectors(cells)
                self.cells = []                            # [(cell_id, ix, iy, cx, cy, sector)]
                for ix, iy in cells:
                    cx, cy = ox + (ix + 0.5) * GRID, oy + (iy + 0.5) * GRID
                    sec = sectors[(int(ix), int(iy))]
                    cid = self.q("INSERT INTO grid_cells (map_id, ix, iy, center_x, center_y, sector) "
                                 "VALUES (%s, %s, %s, %s, %s, %s) RETURNING cell_id",
                                 (map_id, int(ix), int(iy), cx, cy, sec)).fetchone()["cell_id"]
                    self.cells.append((cid, int(ix), int(iy), cx, cy, sec))
                self.count("grid_cells", len(self.cells))
                self.room = (w, h)
                self.origin = (ox, oy)
                self.cell_sector = {(c[1], c[2]): c[5] for c in self.cells}
                self.cell_by_ixiy = {(c[1], c[2]): c for c in self.cells}
                # 섹터별 측정 경로(3-3차) — server_page.plan_survey처럼 칸마다 한 점, ix 열마다 iy 방향을 번갈아(지그재그).
                #   더미 지도는 합성이라 벽 여유는 계산하지 않고 칸 중심을 씀
                self.survey_path = {}
                for sec in (1, 2):
                    cols: dict[int, list] = {}
                    for c in self.cells:
                        if c[5] == sec:
                            cols.setdefault(c[1], []).append(c)
                    pts = []
                    for j, ix in enumerate(sorted(cols)):
                        for c in sorted(cols[ix], key=lambda c: c[2], reverse=bool(j % 2)):
                            pts.append({"x": round(c[3], 3), "y": round(c[4], 3), "yaw": 0.0})
                    for j in range(len(pts)):              # 방향 = 다음 점 쪽 (마지막은 앞 점과 같음)
                        nxt = pts[j + 1] if j + 1 < len(pts) else None
                        pts[j]["yaw"] = round(math.atan2(nxt["y"] - pts[j]["y"], nxt["x"] - pts[j]["x"]), 4) if nxt \
                            else (pts[j - 1]["yaw"] if j else 0.0)
                    self.survey_path[sec] = pts
        self.count("maps", len(MAPS))

    def facilities(self):
        types = [("extinguisher", "EXT"), ("shutter", "SHT"), ("exit_light", "EXL"), ("rack", "RCK")]   # rack: 3-3 보완
        w, h = self.room
        self.fac = {}                                      # facility_id → dict
        seq = {p: 0 for _, p in types}
        for i in range(self.n):
            ftype, prefix = types[i % len(types)]           # 종류를 차례로 돌아가며
            seq[prefix] += 1
            fid = f"{prefix}-{seq[prefix]:02d}"
            # 벽 가까운 곳에 두고, 관측 자세는 방 안쪽 0.8 m 앞에서 설비를 바라봄
            side = self.r.choice(["S", "N", "W", "E"])
            if side in "SN":
                ex, ey = round(self.r.uniform(0.4, w - 0.4), 2), (0.15 if side == "S" else h - 0.15)
                vx, vy = ex, (ey + 0.8 if side == "S" else ey - 0.8)
            else:
                ex, ey = (0.15 if side == "W" else w - 0.15), round(self.r.uniform(0.4, h - 0.4), 2)
                vx, vy = (ex + 0.8 if side == "W" else ex - 0.8), ey
            yaw = round(math.atan2(ey - vy, ex - vx), 3)
            f = dict(facility_id=fid, facility_type=ftype, has_gauge=(ftype == "extinguisher" and i % 2 == 0),
                     expected_x=ex, expected_y=ey, view_pose={"x": round(vx, 2), "y": round(vy, 2), "yaw": yaw},
                     is_active=(i % 17 != 16))
            self.q("INSERT INTO facilities (facility_id, map_id, facility_type, has_gauge, expected_x, expected_y, "
                   "view_pose, is_active) VALUES (%s, %s, %s, %s, %s, %s, %s, %s)",
                   (fid, self.map_id, ftype, f["has_gauge"], ex, ey, Jsonb(f["view_pose"]), f["is_active"]))
            self.fac[fid] = f
        self.active_fac = [k for k, v in self.fac.items() if v["is_active"]]
        # 설비 ID는 지도 안에서 유일(3차) — 다른 합성 지도에 같은 ID 4개를 다시 등록
        other = self.map_ids.get("dummy_lab_c")
        if other:
            for fid, ftype in (("EXT-01", "extinguisher"), ("EXT-02", "extinguisher"), ("SHT-01", "shutter"), ("EXL-01", "exit_light")):
                self.q("INSERT INTO facilities (facility_id, map_id, facility_type, expected_x, expected_y, view_pose) "
                       "VALUES (%s, %s, %s, 0.15, 2.0, %s)", (fid, other, ftype, Jsonb({"x": 0.95, "y": 2.0, "yaw": 3.1416})))
            self.count("facilities", 4)
        # 설비의 섹터 = view_pose가 속한 칸의 섹터 (저장하지 않고 계산)
        ox, oy = self.origin
        for f in self.fac.values():
            key = (math.floor((f["view_pose"]["x"] - ox) / GRID), math.floor((f["view_pose"]["y"] - oy) / GRID))
            f["sector"] = self.cell_sector.get(key, 1)
        self.count("facilities", self.n)

    # ---------------- 순찰·작업 ----------------
    def task(self, task_id, run, robot, ttype, target, waypoints, status, start, end, progress=None, reason=None):
        done = status in ("DONE", "FAILED", "CANCELED")
        self.task_list.append(dict(task_id=task_id, run=run, robot=robot, type=ttype, wp=waypoints, status=status,
                                   start=None if status in ("PENDING", "CANCELED") else start, end=end if done else None,
                                   progress=progress if progress is not None else (1.0 if status == "DONE" else 0.0)))
        self.q("INSERT INTO tasks (task_id, run_id, robot_id, type, target_ref, waypoints, status, progress, "
               "result_ok, reason, assigned_at, ended_at) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)",
               (task_id, run["run_id"], robot, ttype, target, Jsonb(waypoints), status,
                progress if progress is not None else (1.0 if status == "DONE" else 0.0),
                {"DONE": True, "FAILED": False}.get(status), reason,
                None if status in ("PENDING", "CANCELED") else start, end if done else None))
        self.count("tasks", 1)

    def tid(self, run):
        """run 안에서 만든 순서대로 task 번호 (두 로봇 통틀어)"""
        run["n_task"] = run.get("n_task", 0) + 1
        return f"r{run['run_id']}-t{run['n_task']:03d}"

    def survey_waypoints(self, k, sector):
        """섹터 측정 경로의 k번째 구간(20점, segment_points) → (target_ref "seg:a-b", 경로점). a~b = 섹터 경로의 점 번호"""
        pts = self.survey_path[sector]
        nseg = max(1, math.ceil(len(pts) / SEGMENT_POINTS))
        a = (k % nseg) * SEGMENT_POINTS
        part = pts[a:a + SEGMENT_POINTS]
        return f"seg:{a}-{a + len(part) - 1}", part

    def runs(self):
        reasons = ["BLOCKED", "BATTERY_LOW", "NO_RESPONSE", "ESTOP"]
        self.run_list, self.task_list = [], []
        for k in range(1, self.n + 1):
            start = self.day0 + timedelta(days=k - 1, minutes=self.r.randint(0, 40))
            rtype = "remeasure" if k % 10 == 0 else "demo" if k % 7 == 0 else "patrol"
            solo = k % 8 == 0 or k % 12 == 9               # robot3 혼자 (섹터 1만)
            failed = k % 12 in (6, 9)                      # 주 로봇의 섹터 실패 — 6: partial, 9: aborted(혼자)
            status = "done" if not failed else "aborted" if solo else "partial"
            sector_robots = {"1": SECTOR_ROBOTS["1"]} if solo else dict(SECTOR_ROBOTS)
            sec = 1 if solo or k % 2 else 2                # 주 task를 맡는 섹터 — 회차마다 번갈아
            robot = sector_robots[str(sec)]
            end = start + timedelta(minutes=self.r.randint(35, 55) if not failed else self.r.randint(8, 25))
            run_id = self.q("INSERT INTO runs (map_id, run_type, sector_robots, status, started_by, started_at, ended_at) "
                            "VALUES (%s, %s, %s, %s, %s, %s, %s) RETURNING run_id",
                            (self.map_id, rtype, Jsonb(sector_robots), status, self.admin_id, start,
                             end)).fetchone()["run_id"]
            run = dict(run_id=run_id, k=k, type=rtype, aborted=failed, status=status, start=start, end=end,
                       robot=robot, precise=robot, sector=sec, sector_robots=sector_robots,
                       reason=reasons[sum(1 for r in self.run_list if r["aborted"]) % 4] if failed else None,
                       confirmed=False)
            # 주 task — 홀수 회차 SURVEY_SEGMENT, 짝수 회차 FACILITY_CHECK (주 로봇의 섹터 안)
            t1 = self.tid(run)
            t_start, t_end = start + timedelta(minutes=1), end - timedelta(minutes=2)
            fac_here = [f for f in self.active_fac if self.fac[f]["sector"] == sec]
            if k % 2 or not fac_here:
                ref, wp = self.survey_waypoints(k, sec)
                run.update(ptype="SURVEY_SEGMENT", target=ref, wp=wp)
            else:
                fid = fac_here[(k // 2) % len(fac_here)]
                run.update(ptype="FACILITY_CHECK", target=fid, wp=[self.fac[fid]["view_pose"]])
            run["t1"], run["t1_start"], run["t1_end"] = t1, t_start, t_end
            if failed:
                run["t1_progress"] = round(self.r.uniform(0.2, 0.7), 2)
                self.task(t1, run, robot, run["ptype"], run["target"], run["wp"], "FAILED", t_start, end,
                          progress=run["t1_progress"], reason=run["reason"])
            else:
                self.task(t1, run, robot, run["ptype"], run["target"], run["wp"], "DONE", t_start, t_end)
            # 다른 로봇 — 자기 섹터 측정 1구간 (같은 시각에 동시 운행)
            run["others"] = []
            for s_key, other in sector_robots.items():
                if other == robot:
                    continue
                o_sec = int(s_key)
                o_tid = self.tid(run)
                o_ref, o_wp = self.survey_waypoints(k + 1, o_sec)
                self.task(o_tid, run, other, "SURVEY_SEGMENT", o_ref, o_wp,
                          "DONE", t_start, t_start + timedelta(minutes=self.r.randint(15, 25)))
                run["others"].append((other, o_tid))
            self.run_list.append(run)
        self.count("runs", self.n)

    def docks(self):
        """끝에 로봇마다 DOCK. 실패한 주 로봇도 복귀 DOCK(관제 PC가 새로 만드는 task)으로 돌아감"""
        for run in self.run_list:
            robots = [run["robot"]] + [o for o, _ in run["others"]]
            for i, robot in enumerate(robots):
                t = self.tid(run)
                end = run["end"] - timedelta(seconds=20 * i)
                dock = self.dock_poses.get(robot)          # 3-3차 — DOCK 경로점 = 그 로봇의 도크 접근 자세
                self.task(t, run, robot, "DOCK", None, [dock] if dock else [], "DONE",
                          end - timedelta(minutes=1, seconds=30), end)

    # ---------------- 이상 후보 → 재검증 → 확정 ----------------
    def candidates(self):
        outcomes = ["CONFIRMED", "CLEARED", "CONFIRMED", "UNCONFIRMED", "CONFIRMED", "CLEARED"]
        vis_kinds = ["VIS_NOT_DETECTED", "VIS_OCCLUDED", "VIS_POSITION", "VIS_ABNORMAL"]
        done_runs = [r for r in self.run_list if not r["aborted"]]
        cand_runs = done_runs[:24] + [r for r in self.run_list if r["aborted"]][:4]
        self.cands = []
        for i, run in enumerate(cand_runs):
            cid = f"r{run['run_id']}-c01"
            det_at = run["t1_start"] + timedelta(minutes=self.r.randint(3, 10))
            if run["ptype"] == "SURVEY_SEGMENT":
                # 3-3차 — 그 측정 task가 지난 칸 중 하나(경로점의 칸). 지점 = 그 칸에서 마지막으로 잰 위치(net_samples가 맞춤)
                reached = run["wp"][:max(1, math.ceil(len(run["wp"]) * run.get("t1_progress", 1.0)))]   # 실패면 간 데까지
                p = self.r.choice(reached[2:] or reached)
                cell = self.cell_of(p["x"], p["y"])
                tx, ty = round(p["x"] + self.r.uniform(-0.12, 0.12), 3), round(p["y"] + self.r.uniform(-0.12, 0.12), 3)
                c = dict(source="NETWORK", kind="NET_COVERAGE", ref=str(cell[0]), tx=tx, ty=ty,
                         score=float(self.r.randint(-84, -74)), cell=cell)
            else:
                fac = self.fac[run["target"]]
                kind = vis_kinds[i % 4]
                c = dict(source="VISION", kind=kind, ref=run["target"], tx=fac["view_pose"]["x"],
                         ty=fac["view_pose"]["y"], score=round(self.r.uniform(0.55, 0.9), 2), fac=fac)
            c.update(candidate_id=cid, run=run, detected_at=det_at)
            # 결론 — 끝난 회차는 CLOSED, 중단된 회차는 OPEN 또는 REINSPECTING(재검증 task CANCELED)
            if run["aborted"]:
                c.update(status="REINSPECTING" if i % 2 else "OPEN", outcome=None, decided_at=None)
            else:                                          # 재검증은 1차 task가 끝난 뒤 (한 번에 한 로봇)
                r_start = run["t1_end"] + timedelta(seconds=30)
                c.update(status="CLOSED", outcome=outcomes[i % len(outcomes)],
                         decided_at=r_start + timedelta(minutes=self.r.randint(3, 8)))
            self.q("INSERT INTO candidates (candidate_id, run_id, source, kind, ref, target_x, target_y, score, "
                   "detect_task_id, detected_by, status, detected_at) "
                   "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, 'OPEN', %s)",
                   (cid, run["run_id"], c["source"], c["kind"], c["ref"], c["tx"], c["ty"], c["score"],
                    run["t1"], run["robot"], det_at))
            if c["status"] != "OPEN":
                t2 = self.tid(run)
                wp = [{"x": round(c["tx"], 2), "y": round(c["ty"], 2), "yaw": 0.0}]
                r_start = run["t1_end"] + timedelta(seconds=30)
                if run["aborted"]:
                    self.task(t2, run, run["precise"], "REINSPECTION", cid, wp, "CANCELED", None, run["end"],
                              reason=f"SECTOR_ABORTED:{run['t1']}")
                else:
                    self.task(t2, run, run["precise"], "REINSPECTION", cid, wp, "DONE", r_start, c["decided_at"])
                    run["end"] = c["decided_at"] + timedelta(minutes=2)      # 재검증까지 끝난 뒤 run 종료
                    self.q("UPDATE runs SET ended_at = %s WHERE run_id = %s", (run["end"], run["run_id"]))
                c.update(t2=t2, t2_start=r_start)
                self.q("UPDATE candidates SET status = %s, reinspect_task_id = %s, outcome = %s, decided_at = %s "
                       "WHERE candidate_id = %s", (c["status"], t2, c["outcome"], c["decided_at"], cid))
            if c["outcome"] == "CONFIRMED":
                run["confirmed"] = True
            self.cands.append(c)
        self.count("candidates", len(self.cands))

        # 기준선 회차: 끝난 patrol 회차 중 확정 이상이 없는 회차
        for run in self.run_list:
            run["baseline_ok"] = run["status"] == "done" and run["type"] == "patrol" and not run["confirmed"]
            if run["baseline_ok"]:
                self.q("UPDATE runs SET is_baseline_ok = true WHERE run_id = %s", (run["run_id"],))

    def findings(self):
        n = 0
        for i, c in enumerate(x for x in self.cands if x["outcome"] == "CONFIRMED"):
            if c["source"] == "NETWORK":
                cat = "coverage"
            elif c["kind"] == "VIS_ABNORMAL" and c["fac"]["has_gauge"]:
                cat = "gauge"
            else:
                cat = c["fac"]["facility_type"]
            acked = i % 2 == 0
            self.q("INSERT INTO findings (candidate_id, task_id, category, status, acked_by, confirmed_at) "
                   "VALUES (%s, %s, %s, %s, %s, %s)",
                   (c["candidate_id"], c["t2"], cat, "acked" if acked else "open",
                    self.admin_id if acked else None, c["decided_at"]))
            n += 1
        self.count("findings", n)

    # ---------------- 통신 인프라 ----------------
    def cell_of(self, x, y):
        """map 좌표 → 칸 (cell_id, ix, iy, cx, cy, sector) 또는 None — server_page.cell_at과 같은 규칙"""
        ox, oy = self.origin
        return self.cell_by_ixiy.get((math.floor((x - ox) / GRID), math.floor((y - oy) / GRID)))

    def rssi(self, x, y):
        """측정 AP까지 거리로 정한 RSSI (dBm, 잡음 포함) — 더미 값"""
        tx, ty = self.target_xy
        d = max(0.5, math.hypot(x - tx, y - ty))
        return int(round(max(-95, min(-30, -35 - 22 * math.log10(d) + self.r.gauss(0, 2.0)))))

    def net_samples(self):
        """3-3차 — 모든 측정 task(SURVEY_SEGMENT)가 경로를 따라 칸마다 3~4개(rssi 1 Hz 가정), 측정 AP 하나, task_id.
        실패한 task는 진행률만큼. 네트워크 후보 칸은 기준(−72) 미만 값, 마지막 표본 위치 = 후보 지점"""
        cand = {c["run"]["run_id"]: c for c in self.cands if c["source"] == "NETWORK"}
        seq: dict[tuple, int] = {}
        rows = []
        for t in self.task_list:
            if t["type"] != "SURVEY_SEGMENT" or t["start"] is None:
                continue
            run = t["run"]
            c = cand.get(run["run_id"]) if t["task_id"] == run.get("t1") else None
            wps = t["wp"][:max(1, math.ceil(len(t["wp"]) * t["progress"]))]
            span = ((t["end"] or run["end"]) - t["start"]).total_seconds()
            cand_cell = c["cell"][0] if c else None
            per = [4 if (cc := self.cell_of(p["x"], p["y"])) and cc[0] == cand_cell else self.r.randint(3, 4) for p in wps]
            total, k = sum(per), 0
            for p, n in zip(wps, per):
                cell = self.cell_of(p["x"], p["y"])
                low = c is not None and cell is not None and cell[0] == c["cell"][0]
                for j in range(n):
                    stamp = t["start"] + timedelta(seconds=span * (k + 0.5) / total)
                    k += 1
                    if low and j == n - 1:                 # 후보 칸의 마지막 표본 = 후보 지점
                        x, y = c["tx"], c["ty"]
                    else:
                        x, y = p["x"] + self.r.uniform(-0.15, 0.15), p["y"] + self.r.uniform(-0.15, 0.15)
                    pose_ok = low or self.r.random() > 0.05     # 후보 칸 표본은 모두 위치 확실(min_samples 이상)
                    sc = self.cell_of(x, y) if pose_ok else None
                    v = int(round(c["score"] + self.r.gauss(0, 1.5))) if low else self.rssi(x, y)
                    key = (t["robot"], run["run_id"])
                    seq[key] = seq.get(key, -1) + 1
                    rows.append((t["robot"], run["run_id"], seq[key], stamp, x, y, p["yaw"], pose_ok, self.target_ap,
                                 max(-127, min(0, v)), sc and sc[0],
                                 stamp + timedelta(milliseconds=self.r.randint(80, 600)), t["task_id"]))
        with self.c.cursor() as cur:
            cur.executemany("INSERT INTO net_samples (robot_id, run_id, seq, stamp, x, y, yaw, pose_ok, ap_id, rssi_dbm, "
                            "cell_id, received_at, task_id) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)", rows)
        self.count("net_samples", len(rows))

    def precise_measures(self):
        n = 0
        for c in self.cands:
            if c["source"] != "NETWORK" or c["outcome"] is None:
                continue
            for ap in (self.target_ap,):                   # 3-3차 — 측정 AP 하나
                if c["outcome"] == "CONFIRMED":
                    med, iqr = self.r.uniform(-84, -75), self.r.uniform(1, 4)
                elif c["outcome"] == "CLEARED":
                    med, iqr = self.r.uniform(-68, -55), self.r.uniform(1, 4)
                else:
                    med, iqr = self.r.uniform(-74, -70), self.r.uniform(6, 12)
                self.q("INSERT INTO precise_measures (task_id, robot_id, ap_id, cell_id, duration_s, n_samples, "
                       "rssi_median, rssi_iqr, measured_at) VALUES (%s, %s, %s, %s, 30, %s, %s, %s, %s)",
                       (c["t2"], c["run"]["precise"], ap, c["cell"][0], self.r.randint(27, 31), round(med, 1),
                        round(iqr, 1), c["decided_at"] - timedelta(seconds=5)))
                n += 1
        self.count("precise_measures", n)

    def grid_stats(self):
        runs = [r for r in self.run_list if r["ptype"] == "SURVEY_SEGMENT" and not r["aborted"]][:5]
        per = max(1, self.n // max(1, len(runs)))
        total = 0
        for run in runs:
            for cid, *_ in self.r.sample(self.cells, min(per, len(self.cells))):
                n = self.r.choice([0, 1, 2, 3, 4, 5, 6, 8, 10])
                mean = round(self.r.uniform(-82, -50), 1)
                interp = n == 0
                state = ("unknown" if n < 3 else "coverage_suspect" if mean < RSSI_SUSPECT else "normal")
                self.q("INSERT INTO grid_stats (run_id, cell_id, ap_id, n, mean, median, var, min, max, confidence, "
                       "interpolated, state, baseline_delta, computed_at) "
                       "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)",
                       (run["run_id"], cid, self.active_aps[0], n, mean,
                        None if interp else round(mean + self.r.uniform(-1.5, 1.5), 1),
                        None if n < 2 else round(self.r.uniform(1, 25), 1),
                        None if interp else int(mean - self.r.randint(2, 8)),
                        None if interp else int(mean + self.r.randint(2, 8)),
                        "low" if n < 3 else "ok", interp, state,
                        None if interp else round(self.r.uniform(-8, 3), 1), run["end"] + timedelta(minutes=1)))
                total += 1
        self.count("grid_stats", total)

    def baselines(self):
        ok = [r for r in self.run_list if r["baseline_ok"]]
        if len(ok) < 6:
            return
        v1, v2 = ok[:3], ok[3:6]
        total = 0
        for cid, *_ in self.cells[: max(1, self.n // 2)]:
            mean = self.r.uniform(-80, -52)
            for ver, group, current in ((1, v1, False), (2, v2, True)):
                self.q("INSERT INTO baselines (cell_id, ap_id, version, run_ids, mean, var, n, built_at, is_current) "
                       "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s)",
                       (cid, self.active_aps[0], ver, [r["run_id"] for r in group],
                        round(mean + self.r.uniform(-2, 2), 1), round(self.r.uniform(2, 20), 1),
                        self.r.randint(9, 30), group[-1]["end"] + timedelta(minutes=5), current))
                total += 1
        self.count("baselines", total)

    # ---------------- 물리 인프라 ----------------
    def inspections(self):
        detail_of = {"VIS_NOT_DETECTED": "NOT_DETECTED", "VIS_OCCLUDED": "OCCLUDED",
                     "VIS_POSITION": "POSITION_ANOMALY", "VIS_ABNORMAL": "ABNORMAL"}
        cand_by_run = {c["run"]["run_id"]: c for c in self.cands}
        self.insp = []

        def add(run, task_id, robot, fac, detail, at):
            frontal = detail not in ("OCCLUDED",)
            final = ("normal" if detail == "NORMAL" else "unconfirmed" if detail == "OCCLUDED" else "abnormal")
            gauge = None
            if fac["has_gauge"] and frontal and detail in ("NORMAL", "ABNORMAL"):
                gauge = "green" if detail == "NORMAL" else "abnormal"
            detected = detail != "NOT_DETECTED"
            uid = uuid.UUID(int=self.r.getrandbits(128), version=4)
            iid = self.q(
                "INSERT INTO inspections (inspection_uid, task_id, run_id, robot_id, map_id, facility_id, detail, final, "
                "frontal, pose_retry, gauge_state, confidence, bbox, model_version, inspected_at) "
                "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s) RETURNING inspection_id",
                (uid, task_id, run["run_id"], robot, self.map_id, fac["facility_id"], detail, final, frontal,
                 3 if detail == "OCCLUDED" else self.r.randint(0, 1), gauge,
                 round(self.r.uniform(0.6, 0.97), 2) if detected else None,
                 Jsonb([110, 60, 230, 200]) if detected else None, MODEL_VERSION, at)).fetchone()["inspection_id"]
            self.insp.append(dict(inspection_id=iid, uid=uid, run=run, robot=robot, fac=fac, detail=detail, at=at))

        for run in self.run_list:
            if run["ptype"] != "FACILITY_CHECK":
                continue
            fac = self.fac[run["target"]]
            c = cand_by_run.get(run["run_id"])
            at = run["t1_start"] + timedelta(minutes=2)
            add(run, run["t1"], run["robot"], fac, detail_of[c["kind"]] if c else "NORMAL", at)
            if c and c["outcome"]:                         # 재검증 로봇의 근접 재촬영
                d2 = {"CONFIRMED": detail_of[c["kind"]], "CLEARED": "NORMAL", "UNCONFIRMED": "OCCLUDED"}[c["outcome"]]
                add(run, c["t2"], run["precise"], fac, d2, c["decided_at"] - timedelta(seconds=10))
        self.count("inspections", len(self.insp))

    def media(self):
        n = 0
        for ins in [i for i in self.insp if i["detail"] != "NOT_DETECTED"][: max(1, self.n // 2)]:
            day = ins["at"].strftime("%Y/%m/%d")
            base = f"{day}/r{ins['run']['run_id']}/{ins['fac']['facility_id']}_{str(ins['uid'])[:8]}"
            label = f"r{ins['run']['run_id']} {ins['fac']['facility_id']} {ins['detail']}"
            for kind, key, data in (("rgb", base + "_rgb.jpg", photo_rgb(label, ins["detail"] != "NORMAL", self.r)),
                                    ("depth", base + "_depth.png", photo_depth(self.r))):
                self.files[key] = data
                self.q("INSERT INTO media (inspection_id, robot_id, kind, object_key, sha256, size_bytes, captured_at) "
                       "VALUES (%s, %s, %s, %s, %s, %s, %s)",
                       (ins["inspection_id"], ins["robot"], kind, key, hashlib.sha256(data).hexdigest(), len(data),
                        ins["at"] - timedelta(seconds=2)))
                n += 1
        self.count("media", n)

    # ---------------- 이벤트·이력·보고 ----------------
    def event(self, robot, run, task_id, code, level, detail, ts, source="robot"):
        if source == "control":
            uid = f"control-{uuid.UUID(int=self.r.getrandbits(128), version=4)}"
        else:
            self.ev_seq[robot] = self.ev_seq.get(robot, 0) + 1
            uid = f"{robot}-r{run['run_id']}-e{self.ev_seq[robot]:06d}"
        self.q("INSERT INTO robot_events (event_uid, robot_id, run_id, task_id, code, source, level, detail, ts) "
               "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s)",
               (uid, robot, run["run_id"], task_id, code, source, level, Jsonb(detail), ts))
        self.count("robot_events", 1)

    def robot_events(self):
        self.ev_seq = {}
        for c in self.cands:
            run = c["run"]
            self.event(run["robot"], run, run["t1"], "CANDIDATE", "warn",
                       {"candidate_id": c["candidate_id"], "kind": c["kind"], "ref": c["ref"], "score": c["score"],
                        "target": [round(c["tx"], 2), round(c["ty"], 2)]}, c["detected_at"])
            if c["outcome"]:
                self.event(run["precise"], run, c["t2"], c["outcome"],
                           {"CONFIRMED": "error", "CLEARED": "info", "UNCONFIRMED": "warn"}[c["outcome"]],
                           {"candidate_id": c["candidate_id"]}, c["decided_at"])
        code_of = {"BLOCKED": "E1", "BATTERY_LOW": "E2", "NO_RESPONSE": "E3", "ESTOP": "E7"}
        for run in [r for r in self.run_list if r["aborted"]]:
            code = code_of[run["reason"]]
            ctrl = code == "E3"
            self.event(run["robot"], run, run["t1"], code, "error",
                       {"reason": run["reason"], "battery": 14.2} if code == "E2" else {"reason": run["reason"]},
                       run["end"] - timedelta(seconds=5), source="control" if ctrl else "robot")

    def robot_status_log(self):
        """3-3차 — 모든 task(측정 · 정밀측정 · 탐지 · 복귀) 동안 경로를 따라 위치 · 배터리 · task_id.
        실제는 1초마다지만 더미 task는 수십 분이라 task마다 최대 STATUS_POINTS점(같은 간격)"""
        state = {"SURVEY_SEGMENT": "NAVIGATING", "REINSPECTION": "MEASURING", "FACILITY_CHECK": "INSPECTING",
                 "DOCK": "DOCKING"}
        last: dict[tuple, tuple] = {}                      # (robot, run) → 마지막 위치 (처음은 도크)
        battery: dict[tuple, float] = {}
        rows = []
        for t in sorted((t for t in self.task_list if t["start"] is not None and t["end"] is not None),
                        key=lambda t: (t["run"]["run_id"], t["robot"], t["start"])):
            key = (t["robot"], t["run"]["run_id"])
            dock = self.dock_poses.get(t["robot"], {"x": 1.0, "y": 0.6})
            here = last.get(key, (dock["x"], dock["y"]))
            bat = battery.setdefault(key, self.r.uniform(82, 97))
            wps = t["wp"][:max(1, math.ceil(len(t["wp"]) * t["progress"]))] if t["wp"] else []
            path = [here] + [(p["x"], p["y"]) for p in wps]
            segs = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:])]
            length = sum(segs)
            span = (t["end"] - t["start"]).total_seconds()
            n = int(max(2, min(STATUS_POINTS, span)))
            move = 1.0 if t["type"] == "SURVEY_SEGMENT" else 0.6   # 그 밖은 앞 60 % 시간에 도착하고 멈춰 있음
            for i in range(n):
                f = i / (n - 1)
                d = length * min(1.0, f / move)
                x, y = path[-1]
                for (a, b), sl in zip(zip(path, path[1:]), segs):   # 경로 위 d 지점
                    if d <= sl and sl > 0:
                        x, y = a[0] + (b[0] - a[0]) * d / sl, a[1] + (b[1] - a[1]) * d / sl
                        break
                    d -= sl
                bat -= 0.05 * span / n / 60                # 1분에 0.05 %씩 (더미)
                rows.append((t["robot"], t["start"] + timedelta(seconds=span * f), t["run"]["run_id"], t["task_id"],
                             round(x + self.r.uniform(-0.03, 0.03), 3), round(y + self.r.uniform(-0.03, 0.03), 3),
                             0.0, round(max(5.0, bat), 1), state[t["type"]], round(f * t["progress"], 2)))
            last[key], battery[key] = path[-1], bat
        with self.c.cursor() as cur:
            cur.executemany("INSERT INTO robot_status_log (robot_id, ts, run_id, task_id, x, y, yaw, battery_pct, nav_state, "
                            "progress) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s) ON CONFLICT DO NOTHING", rows)
        self.count("robot_status_log", len(rows))

    def reports(self):
        done = [r for r in self.run_list if not r["aborted"]][-max(1, self.n // 2):]
        cands = {c["run"]["run_id"]: c for c in self.cands}
        n = 0
        for run in done:
            c = cands.get(run["run_id"])
            findings = 1 if c and c["outcome"] == "CONFIRMED" else 0
            for kind in ("network_report", "checklist"):
                key = f"reports/r{run['run_id']}_{kind}.pdf"
                if kind == "network_report":
                    summary = {"suspect_cells": self.r.randint(0, 6), "coverage_pct": round(self.r.uniform(88, 99.5), 1),
                               "findings": findings if c and c["source"] == "NETWORK" else 0}
                else:
                    summary = {"checked": 1 if run["ptype"] == "FACILITY_CHECK" else 0,
                               "abnormal": findings if c and c["source"] == "VISION" else 0, "unconfirmed": 0}
                self.files[key] = report_pdf([f"SCV dummy report - run {run['run_id']} ({run['type']})", kind,
                                              f"started {run['start']:%Y-%m-%d %H:%M} UTC", str(summary)])
                self.q("INSERT INTO reports (run_id, kind, file_key, summary, generated_at) VALUES (%s, %s, %s, %s, %s)",
                       (run["run_id"], kind, key, Jsonb(summary), run["end"] + timedelta(minutes=2)))
                n += 1
        self.count("reports", n)

    def run_all(self):
        for step in (self.users, self.maps, self.aps, self.facilities, self.runs, self.candidates, self.docks,
                     self.findings,
                     self.net_samples, self.precise_measures, self.grid_stats, self.baselines, self.inspections,
                     self.media, self.robot_events, self.robot_status_log, self.reports):
            step()


# ---------------------------------------------------------------------------
def media_path(media: Path, key: str) -> Path | None:
    p = (media / key).resolve()
    return p if media in p.parents else None


def map_file_pair(name: str, raw: np.ndarray, res: float, ox: float, oy: float) -> dict[str, bytes]:
    """server page 「지도 등록」으로 올릴 수 있는 ROS map_server 형식 — <name>.pgm(P5) + <name>.yaml"""
    buf = io.BytesIO()
    Image.fromarray(np.asarray(raw, dtype=np.uint8), mode="L").save(buf, format="PPM")   # L 모드 → P5 pgm
    meta = (f"image: {name}.pgm\nmode: trinary\nresolution: {res}\norigin: [{ox}, {oy}, 0.0]\n"
            f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    return {f"{name}.pgm": buf.getvalue(), f"{name}.yaml": meta.encode()}


def reset(conn, media: Path) -> list[Path]:
    """robots · settings · admin만 남기고 지움. 지운 행이 가리키던 파일 경로를 돌려줌(커밋 뒤 삭제)."""
    keys = [r["k"] for r in conn.execute(
        "SELECT image_key AS k FROM maps UNION ALL SELECT object_key FROM media UNION ALL SELECT file_key FROM reports")]
    conn.execute(f"TRUNCATE {', '.join(DATA_TABLES)} RESTART IDENTITY")
    conn.execute("UPDATE settings SET updated_by = NULL "
                 "WHERE updated_by IN (SELECT user_id FROM users WHERE username <> 'admin')")
    conn.execute("DELETE FROM users WHERE username <> 'admin'")
    return [p for k in keys if (p := media_path(media, k))]


DEFAULT_MEDIA = "../../scv_app/scv_media"            # 이 스크립트(dummy/) 기준 — scv_db와 나란히 있는 scv_app의 scv_media


def default_media() -> str:
    """--media 기본값 — 이 스크립트 기준 ../../scv_app/scv_media (server_page.py 기본 SCV_MEDIA_DIR).
    실행한 폴더와 상관없이 같은 곳. scv_app 폴더가 없으면 그 경로를 새로 만드니 server의 SCV_MEDIA_DIR과 같은지 확인"""
    return str((Path(__file__).resolve().parent / DEFAULT_MEDIA).resolve())


def main():
    ap = argparse.ArgumentParser(description="SCV 운영 DB 더미 데이터 생성")
    ap.add_argument("--dsn", default=os.environ.get("SCV_DB_DSN", DEFAULT_DSN))
    ap.add_argument("--media", default=os.environ.get("SCV_MEDIA_DIR") or default_media())
    ap.add_argument("--n", type=int, default=50)
    ap.add_argument("--seed", type=int, default=2026)
    ap.add_argument("--reset", action="store_true")
    ap.add_argument("--map-dir", default=REAL_MAP_DIR)
    ap.add_argument("--map-yaml", default=None)
    ap.add_argument("--no-real-map", action="store_true")
    ap.add_argument("--map-out", default=str(Path(__file__).resolve().parent / "maps"))
    a = ap.parse_args()
    if a.n < 12:
        sys.exit("--n은 12 이상이어야 합니다 (기준선·후보 데이터를 만들 회차가 부족)")
    media = Path(a.media).resolve()
    real_map = None if a.no_real_map else load_real_map(a.map_dir, a.map_yaml)   # DB 접속 전에 확인

    try:
        conn = psycopg.connect(a.dsn, row_factory=dict_row)
    except psycopg.OperationalError as e:
        sys.exit(f"DB에 접속하지 못했습니다: {e}\n  컨테이너 실행(docker ps)과 --dsn / SCV_DB_DSN을 확인하세요")

    with conn:
        robots = {r["robot_id"] for r in conn.execute("SELECT robot_id FROM robots")}
        if not set(SECTOR_ROBOTS.values()) <= robots or \
                not conn.execute("SELECT 1 FROM users WHERE username = 'admin'").fetchone() or \
                not conn.execute("SELECT 1 FROM settings LIMIT 1").fetchone():
            sys.exit("robots · admin 계정 · settings가 없습니다. 01_schema.sql · 02_seed.sql을 먼저 적용하세요")
        if not conn.execute("SELECT 1 FROM information_schema.columns "
                            "WHERE table_name = 'settings' AND column_name = 'description'").fetchone():
            sys.exit("이전 판 스키마입니다. 새 01_schema.sql · 02_seed.sql을 먼저 적용하세요")

        old_files: list[Path] = []
        if a.reset:
            old_files = reset(conn, media)
        else:
            used = conn.execute(
                "SELECT (SELECT count(*) FROM runs) + (SELECT count(*) FROM maps) + (SELECT count(*) FROM aps) "
                "+ (SELECT count(*) FROM facilities) + (SELECT count(*) FROM users WHERE username <> 'admin') AS n"
            ).fetchone()["n"]
            if used:
                sys.exit("이미 데이터가 있습니다 (runs · maps · aps · facilities · 일반 계정 중 하나 이상).\n"
                         "  지우고 다시 만들려면 --reset을 붙이세요 (robots · settings · admin은 남김)")
        d = Dummy(conn, media, a.n, random.Random(a.seed), real_map)
        d.run_all()
    # 커밋된 뒤에만 파일을 지우고 씀 — 중간에 오류가 나면 DB도 파일도 그대로
    for p in old_files:
        p.unlink(missing_ok=True)
    for key, data in d.files.items():
        p = media_path(media, key)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(data)
    out = Path(a.map_out).resolve()                    # 더미 지도 yaml + pgm (server page 지도 등록용)
    out.mkdir(parents=True, exist_ok=True)
    for fn, data in d.map_files.items():
        (out / fn).write_bytes(data)
    print(f"지도 파일 {len(d.map_files)}개 → {out} (server page 정보 › 지도 › 지도 등록으로 올릴 수 있음)")
    conn.close()

    print(f"더미 데이터 생성 완료 — 파일 {len(d.files)}개를 {media} 아래에 저장")
    order = ["users", "aps", "maps", "grid_cells", "facilities", "runs", "tasks", "net_samples", "precise_measures",
             "grid_stats", "baselines", "inspections", "media", "candidates", "findings", "robot_events",
             "robot_status_log", "reports"]
    for t in order:
        print(f"  {t:<18}{d.counts.get(t, 0):>5}")
    print("  (robots 2 · settings 13은 02_seed.sql 그대로, 일반 계정 비밀번호는 모두 user1234)")


if __name__ == "__main__":
    main()
