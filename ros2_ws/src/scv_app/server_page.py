#!/usr/bin/env python3
"""
SCV server page — 관제 PC (3-3차)

  범위: PostgreSQL 기준 정보 7개 표(robots · users · settings · aps · maps · grid_cells · facilities)
  화면: /server/login → /server/monitor (현재 지도) · /server/master (정보 탭 — 세로 탭으로 표 선택 + 목록 창)
  제어 PC(client page) 연결: POST /api/client/login (계정 확인) · WebSocket /ws/agent (hello → 로봇 활성, status 1 Hz)
               관제 PC → 제어 PC(WS): welcome · config · run_config · task_assign · task_cancel · run_cancel · run_end · error
               제어 PC → 관제 PC(WS): hello · status · task_progress · task_accepted · task_rejected
  실시간 상태(3-2차 — Redis 대신 이 프로세스 메모리): 로그인 세션(8 h) · 로봇별 최신 status
               1초마다 점검 — settings.status_timeout_s(30 s) 동안 status가 없으면 상태 삭제 후 E3 기록
               (순찰 중이면 그 로봇 task 실패 처리) · SSID 값이 끊긴 AP 비활성
  화면 실시간: WebSocket /ws/ui (모니터 — 로봇 위치·상태 1초마다 · 최근 이벤트 · AP 활성 · 순찰 시작 조건)
  순찰(3-3차): 모니터 「로봇 상태」에서 순찰 시작(섹터 배정 — 정상 · 도킹 로봇만, 현재 지도 · 격자 · 활성 측정 AP 필요) · 중단.
               이력 탭에서 현재 지도의 지난 순찰 결과를 봄. inspection_manager가 로봇마다
               측정(SURVEY_SEGMENT) → 후보 칸 정밀측정(REINSPECTION) → 설비 판정 1회(FACILITY_CHECK) → 복귀(DOCK)를 차례로 보냄
               제어 PC 기록은 REST /api/agent/*(아웃박스): samples · events · precise-measures · inspections · media · tasks/{id}/result
               (+ GET last-seq — 제어 PC가 표본 seq를 이어 붙일 때). 이 경로들은 로그인 확인 없음
  섹터(3-2차): 격자를 만들 때 칸을 섹터 1 · 2로 나눔(split_sectors — 임시 규칙, 바꿔 끼울 수 있음).
               모니터에 섹터를 색으로 표시, 순찰(run) 진행 중에는 그 지도의 격자 다시 생성·삭제와 현재 지도 바꾸기 금지
  실행: python3 server_page.py           (기본 http://0.0.0.0:8000)
  환경변수:
    SCV_DB_DSN     기본 "host=127.0.0.1 port=5432 dbname=robotdb4_postgresql user=rokey password=rokey"
    SCV_MEDIA_DIR  지도 PNG 등 파일 저장 위치 (기본 ./scv_media)
    SCV_HOST / SCV_PORT  기본 0.0.0.0 / 8000
    SCV_RUN_OWNER  web(기본) / ros — ros 면 순찰 · 작업은 inspection_manager 가 정하고 ros_bridge.py 가 상태 · 알림 ·
                   판정 · 표본을 받아 이 서버의 저장 함수로 넘김, 순찰 시작 · 중단 · 재개는 /inspection/* 서비스 (10/14)

  워커는 1개로만 실행 — 세션 · 로봇 상태 · 제어 PC WebSocket 연결을 이 프로세스 메모리에 두기 때문
  (메모리에는 다시 채울 수 있는 값만 둠 — 세션은 다시 로그인, 나머지는 status 수신 · DB 재집계로 다시 참. 기록은 모두 PostgreSQL)
  메모리: SESSIONS · ROBOT_STATE · AGENTS · LAST_E3 · CURRENT_RUN · RECENT_EVENTS · LIVE_CELLS · AP_SEEN · GRID_CACHE
"""
from __future__ import annotations

import asyncio
import json
import math
import os
import re
import secrets
import threading
import uuid
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

import numpy as np
import psycopg
import psycopg.errors
import uvicorn
import yaml
from datetime import datetime, timezone

from fastapi import FastAPI, File, Form, HTTPException, Request, UploadFile, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles
from PIL import Image
from psycopg import sql
from psycopg.rows import dict_row
from psycopg.types.json import Jsonb
from psycopg_pool import ConnectionPool
from starlette.concurrency import run_in_threadpool

# ---------------------------------------------------------------------------
# 설정
# ---------------------------------------------------------------------------
BASE = Path(__file__).resolve().parent
STATIC_DIR = BASE / "static"
DSN = os.environ.get("SCV_DB_DSN",
                     "host=127.0.0.1 port=5432 dbname=robotdb4_postgresql user=rokey password=rokey")
MEDIA_DIR = Path(os.environ.get("SCV_MEDIA_DIR", BASE / "scv_media")).resolve()
HOST = os.environ.get("SCV_HOST", "0.0.0.0")
PORT = int(os.environ.get("SCV_PORT", "8000"))
RUN_OWNER = os.environ.get("SCV_RUN_OWNER", "web").lower()
if RUN_OWNER not in ("web", "ros"):
    raise ValueError("SCV_RUN_OWNER는 web 또는 ros여야 합니다")
BRIDGE = None                                  # ros 모드의 ros_bridge.Bridge (lifespan에서 켬)

SESSION_COOKIE = "scv_session"
SESSION_TTL_S = 8 * 3600                     # 로그인 세션 8 h (요청마다 연장)
PAGES = {"login", "monitor", "history", "master"}

# 지도 빈 공간(흰 부분) 판정 — map_server(yaml) 기본 free 임계값 (제안설명서 4.2절)
FREE_THRESH = 0.25
UNKNOWN_PIXEL = 205          # map_saver가 미탐색 칸을 저장하는 값
SECTORS = (1, 2)
ROBOT_ID_RE = re.compile(r"^[a-z][a-z0-9_]*$")     # DB CHECK robots_robot_id_format과 같은 규칙
# 섹터 ↔ 로봇 기본 배정 (3차). 순찰 시작 때 바꿀 수 있고 run마다 runs.sector_robots에 저장(3-3차)
DEFAULT_SECTOR_ROBOTS = {"1": "robot3", "2": "robot4"}
# ros 모드 — 섹터 · 담당 로봇은 inspection_manager 와 같은 zones.yaml (SCV_ZONES, 기본 scv_bringup/config/zones.yaml)
ZONES: list[tuple[int, list]] = []
if RUN_OWNER == "ros":
    _zf = Path(os.environ.get("SCV_ZONES", BASE.parent / "scv_bringup" / "config" / "zones.yaml"))   # scv_app 은 src 안 (10/15 이동)
    for _z in (yaml.safe_load(_zf.read_text(encoding="utf-8")) or {}).get("zones", []):
        _n = int("".join(ch for ch in str(_z["id"]) if ch.isdigit()))
        ZONES.append((_n, _z["polygon"]))
        DEFAULT_SECTOR_ROBOTS[str(_n)] = _z.get("robot")

pool = ConnectionPool(DSN, min_size=1, max_size=5, kwargs={"row_factory": dict_row}, open=False)
# 관제 PC 메모리 상태 (3-2차 — Redis 대신). 요청 처리 스레드 풀과 이벤트 루프가 함께 쓰므로 잠금 하나로 보호
MEM_LOCK = threading.Lock()
SESSIONS: dict[str, dict[str, Any]] = {}       # token → {user_id, username, role, created_at, expires(monotonic)}
ROBOT_STATE: dict[str, dict[str, Any]] = {}    # robot_id → {"fields": status 필드 + received_at(모두 문자열), "rx": 받은 시각(monotonic)}


@asynccontextmanager
async def lifespan(_app: FastAPI):
    """시작: 미디어 폴더 · DB 풀 열기 → robots · aps 활성 초기화 → 메모리 복원 → state_watchdog 시작. 끝: 점검 중지 · 풀 닫기"""
    MEDIA_DIR.joinpath("maps").mkdir(parents=True, exist_ok=True)
    pool.open(wait=True, timeout=10)
    global MAIN_LOOP
    MAIN_LOOP = asyncio.get_running_loop()
    reset_reported_active()
    restore_memory()                                   # 진행 중 run · 최근 이벤트 · 순찰 중 로봇 대기 상태 (3-3차)
    watchdog = asyncio.create_task(state_watchdog())
    if RUN_OWNER == "ros":
        global BRIDGE
        import sys
        import ros_bridge
        BRIDGE = ros_bridge.start(sys.modules[__name__])
    yield
    watchdog.cancel()
    pool.close()


app = FastAPI(title="SCV server page", version="3-3", lifespan=lifespan)


@app.middleware("http")
async def guard_ros_owner(request: Request, call_next):
    """ros 모드 — client_page 기록(REST /api/agent/*)을 받지 않음. 기록은 ros_bridge 가 ROS 에서 받아 넣음"""
    if RUN_OWNER == "ros" and request.url.path.startswith("/api/agent/") and request.method != "GET":
        return JSONResponse(status_code=409, content={"detail": "ROS 관제(SCV_RUN_OWNER=ros)에서는 client 기록을 받지 않습니다"})
    return await call_next(request)
MAIN_LOOP: asyncio.AbstractEventLoop | None = None       # 동기 API에서 제어 PC로 config를 밀어 보낼 때 씀

# ---------------------------------------------------------------------------
# 활성 여부 — robots · aps는 client(제어 PC)·로봇이 값을 보내 줄 때만 활성
#   server page에서는 바꾸지 못함. 아직 받은 값이 없으므로 시작할 때 모두 비활성으로 맞춤
#   (robots: /ws/agent hello → 활성, 연결 끊김 → 비활성 — set_robot_active
#    aps: status의 ssids → 활성(mark_aps_seen), status_timeout_s 동안 못 받으면 비활성(expire_aps))
# ---------------------------------------------------------------------------
REPORTED_ACTIVE_TABLES = ("robots", "aps")


def reset_reported_active() -> None:
    """서버 시작 때 robots · aps의 is_active를 모두 false로 (메모리 연결 정보가 비었으므로)"""
    with pool.connection() as conn:
        for t in REPORTED_ACTIVE_TABLES:
            conn.execute(sql.SQL("UPDATE {} SET is_active = false WHERE is_active").format(sql.Identifier(t)))

# ---------------------------------------------------------------------------
# 세션 — 메모리 SESSIONS (user_id · username · role · created_at), 8 h (요청마다 연장)
#   server_page를 다시 켜면 비므로 다시 로그인
# ---------------------------------------------------------------------------
def session_new(user: dict[str, Any]) -> str:
    """로그인 성공 → 새 토큰을 SESSIONS에 넣고 돌려줌 (쿠키 값)"""
    token = secrets.token_urlsafe(32)
    with MEM_LOCK:
        SESSIONS[token] = {"user_id": int(user["user_id"]), "username": user["username"], "role": user["role"],
                           "created_at": datetime.now(timezone.utc).isoformat(),
                           "expires": time.monotonic() + SESSION_TTL_S}
    return token


def session_get(token: str | None) -> dict[str, Any] | None:
    """토큰 → 세션 정보(expires 뺀 복사본). 만료면 지우고 None, 유효하면 만료를 8 h 연장"""
    if not token:
        return None
    with MEM_LOCK:
        s = SESSIONS.get(token)
        if not s:
            return None
        if s["expires"] < time.monotonic():
            SESSIONS.pop(token, None)
            return None
        s["expires"] = time.monotonic() + SESSION_TTL_S
        return {k: v for k, v in s.items() if k != "expires"}


def session_tokens_of(user_id: int) -> list[str]:
    """그 계정의 세션 토큰 목록 (계정 삭제 때 세션 정리용)"""
    with MEM_LOCK:
        return [t for t, s in SESSIONS.items() if s["user_id"] == user_id]


def session_drop(token: str):
    """세션 하나 삭제 (로그아웃 · 계정 삭제)"""
    with MEM_LOCK:
        SESSIONS.pop(token, None)


def current_user(request: Request) -> dict[str, Any]:
    """요청 쿠키의 로그인 세션 → 사용자 정보. 없거나 만료면 401 (server page API 공통 확인)"""
    s = session_get(request.cookies.get(SESSION_COOKIE))
    if not s:
        raise HTTPException(401, "로그인이 필요합니다")
    return s


# ---------------------------------------------------------------------------
# DB 오류 → 사용자 메시지
# ---------------------------------------------------------------------------
def db_error(e: psycopg.Error, in_use_hint: str = "") -> HTTPException:
    """psycopg 오류 → SQLSTATE별 HTTPException (409 외래키·중복 / 400 규칙·형식·트리거 / 500 그 밖). raise는 호출한 쪽에서"""
    state = e.sqlstate or ""
    detail = (e.diag.message_primary or str(e)).strip()
    if state == "23503":
        return HTTPException(409, "다른 표가 이 행을 사용 중이라 지울 수 없습니다. " + in_use_hint + "(" + detail + ")")
    if state == "23505":
        return HTTPException(409, "이미 있는 값입니다. (" + detail + ")")
    if state in ("23514", "23502"):
        return HTTPException(400, "입력값이 규칙에 맞지 않습니다. (" + detail + ")")
    if state.startswith("22"):
        return HTTPException(400, "입력 형식이 맞지 않습니다. (" + detail + ")")
    if state == "P0001":                     # 트리거 규칙 (예: 수퍼 계정 admin 보호)
        return HTTPException(400, detail)
    return HTTPException(500, "DB 오류: " + detail)


# ---------------------------------------------------------------------------
# 기준 정보 표 정의 — 화면(목록 창)도 이 정의로 그림
#   type: text · int · real · bool · select · pose · password · datetime
#   add: 행 추가 때 입력 · edit: 수정 가능 · ops: 이 표에서 허용하는 동작
#   order: 목록·요약 정렬 — 최신 순(시각 컬럼이 없으면 번호·ID 내림차순). 한 쪽에 PAGE_SIZE행
# ---------------------------------------------------------------------------
PAGE_SIZE = 10
SUPER_USER = "admin"         # 수퍼 계정 — 비밀번호만 변경, 삭제 불가 (DB CHECK·트리거와 같은 규칙)

def C(name, label, type_="text", add=False, edit=False, required=False, options=None, help_=None, default=None,
      hide=False):
    """컬럼 정의 한 개 (TABLES[...]["columns"]용 dict)"""
    return {"name": name, "label": label, "type": type_, "add": add, "edit": edit,
            "required": required, "options": options, "help": help_, "default": default, "hide": hide}


# 점검 설비 종류 — DB CHECK facilities.facility_type과 같은 목록 (rack: 3-3 보완에서 추가)
FACILITY_TYPES = [{"value": v, "label": v} for v in ("extinguisher", "shutter", "exit_light", "rack")]

TABLES: dict[str, dict[str, Any]] = {
    "robots": {
        "title": "로봇", "desc": "SCV에 등록된 로봇과 짝인 제어 PC 주소. 활성은 client(제어 PC)가 값을 보내 줄 때만 — 받지 못하면 비활성",
        "pk": "robot_id", "ops": ["add", "edit", "delete"],
        "columns": [
            C("robot_id", "로봇 ID", add=True, required=True, help_="영문 소문자로 시작, 하이픈 없이 (예: robot3)"),
            C("robot_pc_host", "제어 PC 주소", add=True, edit=True),
            C("rpi_host", "로봇(RPi) 주소", add=True, edit=True),
            C("is_active", "활성 (client 보고)", "bool"),
            C("registered_at", "등록 시각", "datetime"),
        ],
        "list_sql": "SELECT * FROM robots",
        "order": "registered_at DESC, robot_id",
    },
    "users": {
        "title": "사용자 계정",
        "desc": "client page 로그인 계정(권한 user — 추가·수정·삭제) + 수퍼 계정 admin(server page·client page, 비밀번호만 변경)",
        "pk": "user_id", "ops": ["add", "edit", "delete"],
        "columns": [
            C("user_id", "번호", "int"),
            C("username", "아이디", add=True, edit=True, required=True),
            C("password", "비밀번호", "password", add=True, edit=True, required=True,
              help_="4자 이상 · bcrypt 해시로 저장. 수정 때는 바꿀 때만 입력"),
            C("role", "권한"),
            C("created_at", "생성 시각", "datetime"),
        ],
        "list_sql": "SELECT user_id, username, role, created_at FROM users",
        "order": "created_at DESC, user_id DESC",
    },
    "settings": {
        "title": "판정 기준값", "desc": "제안 수치 키-값. 바꾸면 변경자·시각이 남음",
        "pk": "key", "ops": ["add", "edit", "delete"],
        "columns": [
            C("key", "설정 이름", add=True, required=True),
            C("description", "설명", add=True, edit=True, help_="이 설정이 정하는 것 · 쓰는 곳"),
            C("value", "값", add=True, edit=True, required=True),
            C("unit", "단위", add=True, edit=True),
            C("source", "근거", add=True, edit=True),
            C("updated_by_name", "변경자"),
            C("updated_at", "변경 시각", "datetime"),
        ],
        "list_sql": ("SELECT s.key, s.description, s.value, s.unit, s.source, u.username AS updated_by_name, "
                     "s.updated_at FROM settings s LEFT JOIN users u ON u.user_id = s.updated_by"),
        "order": "updated_at DESC, key",
    },
    "aps": {
        "title": "측정 대상 AP", "desc": "지도마다 등록 — 목록 위에서 지도를 고르고(처음은 현재 지도) 그 지도에 추가. rssi_scanner가 측정할 핫스팟 SSID와 설치 위치. 활성은 client가 그 SSID 값을 받을 때만(실시간) — 받지 못하면 비활성. 활성 AP 중 하나를 「측정 AP로」 지정하면 순찰이 그 SSID만 측정",
        "pk": "ap_id", "ops": ["add", "edit", "delete", "target"], "map_scoped": True,
        "columns": [
            C("ap_id", "번호", "int"),
            C("map_id", "지도", "int", hide=True),
            C("ssid", "SSID", add=True, edit=True, required=True),
            C("note", "메모", add=True, edit=True),
            C("x", "설치 x(m)", "real", add=True, edit=True, help_="x · y 함께 입력 또는 둘 다 비움"),
            C("y", "설치 y(m)", "real", add=True, edit=True),
            C("is_active", "활성 (client·로봇 보고)", "bool"),
            C("is_target", "측정 AP", "bool"),
        ],
        "list_sql": "SELECT * FROM aps",
        "order": "ap_id DESC",
    },
    "maps": {
        "title": "지도", "desc": "yaml + pgm 업로드로 등록(값은 파일에서 읽음). 수정은 로봇별 도크 접근 자세(도크 바로 앞에서 도크를 바라보는 자세)만. 지우면 그 지도의 격자 · AP · 설비도 함께 지움(순찰 기록이 있으면 거부)",
        "pk": "map_id", "ops": ["upload", "activate", "edit", "delete"],
        "columns": [
            C("map_id", "번호", "int"),
            C("name", "이름"),
            C("robot_map_path", "제어 PC 지도 경로"),
            C("image_key", "이미지", hide=True),
            C("resolution_m", "해상도(m/px)", "real"),
            C("origin_x", "원점 x(m)", "real"),
            C("origin_y", "원점 y(m)", "real"),
            C("grid_size_m", "격자(m)", "real"),
            C("dock_poses", "도크 접근 자세 (로봇별 x m, y m, yaw rad)", "docks", edit=True),   # 지도 좌표 m · 방향 rad
            C("cell_count", "격자 칸 수", "int"),
            C("is_active", "현재 지도", "bool"),
            C("created_at", "등록 시각", "datetime"),
        ],
        "list_sql": ("SELECT m.*, (SELECT count(*) FROM grid_cells g WHERE g.map_id = m.map_id) AS cell_count "
                     "FROM maps m"),
        "order": "created_at DESC, map_id DESC",
    },
    "grid_cells": {
        "title": "격자 칸", "desc": "현재 지도로 선택할 때 그 지도만 자동 생성 — 지도의 흰 부분이 걸치는 칸은 모두. 섹터 1 · 2로 나눔(ix 순 칸 수 절반). 손으로 추가·수정하지 않음",
        "pk": "cell_id", "ops": ["regenerate", "delete_map_cells"],
        "columns": [
            C("cell_id", "번호", "int"),
            C("map_id", "지도", "int"),
            C("ix", "ix", "int"),
            C("iy", "iy", "int"),
            C("center_x", "중심 x(m)", "real"),
            C("center_y", "중심 y(m)", "real"),
            C("sector", "섹터", "int"),
        ],
        "list_sql": None,       # 지도별로 조회 (아래 list_grid_cells)
        "order": "cell_id DESC",
    },
    "facilities": {
        "title": "점검 설비", "desc": "지도마다 등록 — 목록 위에서 지도를 고르고(처음은 현재 지도) 그 지도에 추가. 설비 ID는 지도 안에서 유일. 소화기·셔터·유도등·랙 위치와 관측 자세",
        "pk": "facility_id", "ops": ["add", "edit", "delete"], "map_scoped": True,
        "columns": [
            C("facility_id", "설비 ID", add=True, required=True),
            C("map_id", "지도", "int", hide=True),
            C("facility_type", "종류", "select", add=True, edit=True, required=True, options=FACILITY_TYPES),
            C("has_gauge", "압력계", "bool", add=True, edit=True, default=False),
            C("expected_x", "위치 x(m)", "real", add=True, edit=True, required=True),
            C("expected_y", "위치 y(m)", "real", add=True, edit=True, required=True),
            C("view_pose", "관측 자세 (x m, y m, yaw rad)", "pose", add=True, edit=True, required=True),   # 지도 좌표 m · 방향 rad
            C("is_active", "활성", "bool", add=True, edit=True, default=True),
        ],
        "list_sql": "SELECT * FROM facilities",
        "order": "facility_id DESC",          # 등록 시각 컬럼이 없어 ID 내림차순
    },
}
ORDER = ["robots", "users", "settings", "aps", "maps", "grid_cells", "facilities"]


def table_meta(name: str) -> dict[str, Any]:
    """표 이름 → TABLES 정의. 없는 표면 404"""
    meta = TABLES.get(name)
    if not meta:
        raise HTTPException(404, "없는 표입니다")
    return meta


def coerce(col: dict[str, Any], value: Any) -> Any:
    """화면에서 온 값을 컬럼 형식으로 바꿈. 빈 문자열은 NULL (필수 컬럼이면 400).
    pose · docks는 Jsonb로 감쌈. 형식이 틀리면 400"""
    t = col["type"]
    if value is None or (isinstance(value, str) and value.strip() == ""):
        if col["required"]:
            raise HTTPException(400, f"「{col['label']}」을(를) 입력하세요")
        return None
    try:
        if t == "int" or (t == "select" and col["options"] == "maps"):
            return int(value)
        if t == "real":
            v = float(value)
            if not math.isfinite(v):
                raise ValueError
            return v
        if t == "bool":
            return value if isinstance(value, bool) else str(value).lower() in ("true", "1", "on", "yes")
        if t == "pose":
            pose = {k: float(value[k]) for k in ("x", "y", "yaw")}
            return Jsonb(pose)
        if t == "docks":                               # {robot_id: {x, y, yaw}} — x · y가 빈 로봇은 도크 없음
            out = {}
            for rid, p in dict(value).items():
                if not ROBOT_ID_RE.match(str(rid)):
                    raise ValueError
                if p is None or all(str(p.get(k, "")).strip() == "" for k in ("x", "y", "yaw")):
                    continue
                txt = {k: str(p.get(k, "") if p.get(k) is not None else "").strip() for k in ("x", "y", "yaw")}
                if not txt["x"] or not txt["y"]:           # x · y는 함께 (yaw만 비우면 0)
                    raise ValueError
                v = {"x": float(txt["x"]), "y": float(txt["y"]), "yaw": float(txt["yaw"] or 0.0)}
                if not all(math.isfinite(z) for z in v.values()):
                    raise ValueError
                out[str(rid)] = v
            return Jsonb(out)
    except (TypeError, ValueError, KeyError):
        raise HTTPException(400, f"「{col['label']}」 형식이 맞지 않습니다")
    return str(value).strip()


def select_options(conn) -> dict[str, list[dict[str, Any]]]:
    """select 컬럼(options="maps")의 선택지 — 지도 목록 {value, label}"""
    maps = conn.execute("SELECT map_id, name, is_active FROM maps ORDER BY map_id").fetchall()
    return {"maps": [{"value": m["map_id"], "label": f"{m['map_id']} · {m['name']}" + (" (현재)" if m["is_active"] else "")}
                     for m in maps]}


# ---------------------------------------------------------------------------
# 화면 (SPA) · 정적 파일
# ---------------------------------------------------------------------------
@app.get("/", include_in_schema=False)
def root():
    """/ → 로그인 화면으로"""
    return RedirectResponse("/server/login")


@app.get("/server", include_in_schema=False)
def server_root():
    """/server → 모니터 화면으로"""
    return RedirectResponse("/server/monitor")


@app.get("/server/{page}", include_in_schema=False)
def server_page(page: str):
    """SPA 화면(login · monitor · history · master) — 모두 같은 index.html, 화면 전환은 브라우저 쪽"""
    if page not in PAGES:
        raise HTTPException(404, "없는 화면입니다")
    return FileResponse(STATIC_DIR / "server" / "index.html")


app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")
MEDIA_DIR.mkdir(parents=True, exist_ok=True)
app.mount("/media", StaticFiles(directory=MEDIA_DIR), name="media")


# ---------------------------------------------------------------------------
# 로그인
# ---------------------------------------------------------------------------
@app.post("/api/login")
async def login(request: Request):
    """server page 로그인 — users 비밀번호(crypt) 확인, admin만 허용 → 세션 쿠키 발급"""
    body = await request.json()
    username = str(body.get("username", "")).strip()
    password = str(body.get("password", ""))
    if not username or not password:
        raise HTTPException(400, "아이디와 비밀번호를 입력하세요")
    with pool.connection() as conn:
        user = conn.execute(
            "SELECT user_id, username, role FROM users "
            "WHERE username = %s AND password_hash = crypt(%s, password_hash)",
            (username, password)).fetchone()
    if not user:
        raise HTTPException(401, "아이디 또는 비밀번호가 맞지 않습니다")
    if user["role"] != "admin":
        raise HTTPException(403, "일반 계정은 client page용입니다. server page는 admin만 로그인할 수 있습니다")
    token = session_new(user)
    resp = JSONResponse({"username": user["username"]})
    resp.set_cookie(SESSION_COOKIE, token, httponly=True, samesite="lax", max_age=SESSION_TTL_S, path="/")
    return resp


@app.post("/api/logout")
def logout(request: Request):
    """로그아웃 — 메모리 세션 삭제 + 쿠키 삭제 (열린 /ws/ui는 끊지 않음)"""
    token = request.cookies.get(SESSION_COOKIE)
    if token:
        session_drop(token)
    resp = JSONResponse({"ok": True})
    resp.delete_cookie(SESSION_COOKIE, path="/")
    return resp


@app.get("/api/me")
def me(request: Request):
    """로그인 여부 확인 (로그인 전에도 200 — 화면이 401 오류를 띄우지 않게)."""
    try:
        u = current_user(request)
    except HTTPException:
        return {"user": None}
    return {"user": {"user_id": u["user_id"], "username": u["username"]}}


# ---------------------------------------------------------------------------
# 쪽 나누기 — 최신 순 PAGE_SIZE행씩
# ---------------------------------------------------------------------------
def paging(total: int, page: int | None) -> tuple[int, int, int]:
    """(쪽 번호, 전체 쪽 수, OFFSET). 범위를 넘으면 마지막 쪽으로."""
    pages = max(1, math.ceil(total / PAGE_SIZE))
    p = min(max(1, page or 1), pages)
    return p, pages, (p - 1) * PAGE_SIZE


def page_rows(conn, base_sql: str, order: str, page: int | None, params=()) -> dict[str, Any]:
    """base_sql 결과를 order로 정렬해 한 쪽(PAGE_SIZE행)만 → {rows, total, page, pages, size}.
    base_sql · order는 코드 안 정의(TABLES)에서만 옴 — 문자열로 이어 붙임"""
    total = conn.execute(f"SELECT count(*) AS n FROM ({base_sql}) t", params).fetchone()["n"]
    p, pages, off = paging(total, page)
    rows = conn.execute(f"SELECT * FROM ({base_sql}) t ORDER BY {order} LIMIT {PAGE_SIZE} OFFSET {off}",
                        params).fetchall()
    return {"rows": rows, "total": total, "page": p, "pages": pages, "size": PAGE_SIZE}


# ---------------------------------------------------------------------------
# 정보 탭 — 표별 요약 (행을 나열하는 표는 최신 순 10개씩 쪽 나눔)
# ---------------------------------------------------------------------------
def summary_card(conn, t: str, page: int | None = None) -> dict[str, Any]:
    """정보 탭 카드 하나 — {table, title, desc, count, items([[이름, 값, …]]), page, pages}. 읽기만"""
    meta = TABLES[t]
    count = conn.execute(sql.SQL("SELECT count(*) AS n FROM {}").format(sql.Identifier(t))).fetchone()["n"]
    items: list[list[Any]] = []
    p, pages = 1, 1
    if t in ("aps", "facilities"):                    # 지도에 속한 표 — 현재 지도 기준
        cur = conn.execute("SELECT map_id, name FROM maps WHERE is_active").fetchone()
        if not cur:
            return {"table": t, "title": meta["title"], "desc": meta["desc"], "count": 0,
                    "items": [["현재 지도", "없음"]], "page": 1, "pages": 1}
        count = conn.execute(sql.SQL("SELECT count(*) AS n FROM {} WHERE map_id = %s").format(sql.Identifier(t)),
                             (cur["map_id"],)).fetchone()["n"]
        if t == "aps":                                # AP — 쪽 나눔 · SSID · 측정 AP · 활성 · 설치 위치
            pg = page_rows(conn, "SELECT * FROM aps WHERE map_id = %s", meta["order"], page, (cur["map_id"],))
            p, pages = pg["page"], pg["pages"]
            for r in pg["rows"]:
                pos = f" · ({r['x']:g}, {r['y']:g})" if r["x"] is not None else " · 위치 없음"
                items.append([r["ssid"], ("측정 AP · " if r["is_target"] else "") + ("활성" if r["is_active"] else "비활성") + pos])
        else:                                         # 설비 — 종류별 개수(활성 수)만
            for r in conn.execute("SELECT facility_type, count(*) AS n, count(*) FILTER (WHERE is_active) AS a "
                                  "FROM facilities WHERE map_id = %s GROUP BY facility_type ORDER BY facility_type",
                                  (cur["map_id"],)):
                items.append([r["facility_type"], f"{r['n']}개 (활성 {r['a']})"])
        return {"table": t, "title": meta["title"], "desc": meta["desc"], "count": count, "items": items,
                "page": p, "pages": pages}
    if t in ("robots", "users", "settings"):          # 행을 나열하는 표 — 최신 순 쪽 나눔
        pg = page_rows(conn, meta["list_sql"], meta["order"], page)
        p, pages = pg["page"], pg["pages"]
        for r in pg["rows"]:
            if t == "robots":
                items.append([r["robot_id"], ("활성" if r["is_active"] else "비활성") +
                              (f" · {r['robot_pc_host']}" if r["robot_pc_host"] else "")])
            elif t == "users":
                items.append([r["username"], "admin (수퍼 계정)" if r["role"] == "admin" else r["role"]])
            elif t == "settings":
                items.append([r["key"], r["value"] + (f" {r['unit']}" if r["unit"] else ""), r["description"] or ""])
    elif t == "maps":                                 # 지도 — 현재 지도 이름 · 해상도만 (count는 전체 지도 수)
        cur = conn.execute("SELECT name, resolution_m FROM maps WHERE is_active").fetchone()
        items.append(["현재 지도", cur["name"] if cur else "없음"])
        if cur:
            items.append(["해상도", f"{cur['resolution_m']:g} m/px"])
    elif t == "grid_cells":                           # 격자 — 현재 지도 칸 수 · 섹터별 칸 수/ix 범위/기본 배정 로봇
        cur = conn.execute("SELECT m.name, (SELECT count(*) FROM grid_cells g WHERE g.map_id = m.map_id) AS n "
                           "FROM maps m WHERE m.is_active").fetchone()
        items.append(["현재 지도 칸", f"{cur['n']}개 ({cur['name']})" if cur else "현재 지도 없음"])
        if cur:
            for r in conn.execute("SELECT g.sector, count(*) AS n, min(g.ix) AS a, max(g.ix) AS b FROM grid_cells g "
                                  "JOIN maps m USING (map_id) WHERE m.is_active GROUP BY g.sector ORDER BY g.sector"):
                rid = DEFAULT_SECTOR_ROBOTS.get(str(r["sector"]), "—")
                items.append([f"섹터 {r['sector']}", f"{r['n']}칸 · ix {r['a']}~{r['b']}", f"기본 배정 {rid}"])
        n_maps = conn.execute("SELECT count(DISTINCT map_id) AS n FROM grid_cells").fetchone()["n"]
        items.append(["격자가 있는 지도", f"{n_maps}개"])
    return {"table": t, "title": meta["title"], "desc": meta["desc"], "count": count, "items": items,
            "page": p, "pages": pages}


@app.get("/api/master/summary")
def master_summary(request: Request):
    """정보 탭 첫 화면 — 7개 표 카드 모두 (각 1쪽)"""
    current_user(request)
    with pool.connection() as conn:
        return {"cards": [summary_card(conn, t) for t in ORDER]}


@app.get("/api/master/summary/{table}")
def master_summary_one(table: str, request: Request, page: int = 1):
    """정보 탭 카드 하나 — 쪽 넘김 · 새로 고침용"""
    current_user(request)
    table_meta(table)
    with pool.connection() as conn:
        return summary_card(conn, table, page)


# ---------------------------------------------------------------------------
# master — 목록·추가·수정·삭제 (일반 표)
# ---------------------------------------------------------------------------
def public_meta(name: str, conn) -> dict[str, Any]:
    """화면에 보낼 표 정의 — 컬럼 선택지를 실제 값(지도 목록 · 도크 입력용 로봇 목록)으로 채운 복사본"""
    meta = table_meta(name)
    opts = select_options(conn)
    cols = []
    for c in meta["columns"]:
        c2 = dict(c)
        if c2["options"] == "maps":
            c2["options"] = opts["maps"]
        if c2["type"] == "docks":                      # 도크 입력 칸 = 등록된 로봇마다
            c2["options"] = [r["robot_id"] for r in conn.execute("SELECT robot_id FROM robots ORDER BY robot_id")]
        cols.append(c2)
    return {"table": name, "title": meta["title"], "desc": meta["desc"], "pk": meta["pk"],
            "ops": meta["ops"], "columns": cols, "map_scoped": bool(meta.get("map_scoped"))}


@app.get("/api/master/{table}")
def list_rows(table: str, request: Request, map_id: int | None = None, page: int = 1):
    """정보 탭 목록 창 — {meta, rows, total, page, pages, size}. 지도에 속한 표는 list_by_map(map_id · maps · running_run 추가)"""
    current_user(request)
    with pool.connection() as conn:
        meta = public_meta(table, conn)
        if table == "grid_cells" or TABLES[table].get("map_scoped"):
            return list_by_map(conn, table, meta, map_id, page)
        pg = page_rows(conn, TABLES[table]["list_sql"], TABLES[table]["order"], page)
    if table == "users":                       # 수퍼 계정: 비밀번호만 수정, 삭제 버튼 없음
        for row in pg["rows"]:
            if row["username"] == SUPER_USER:
                row["_edit_only"] = ["password"]
                row["_no_delete"] = True
    return {"meta": meta, **pg}


def list_by_map(conn, table, meta, map_id, page):
    """지도에 속한 표(grid_cells · aps · facilities) — 고른 지도의 행만. 처음은 현재 지도 (3-2차)
    + 그 지도에 진행 중 run 번호(화면 잠금 안내용) · 지도 선택지"""
    maps = conn.execute("SELECT map_id, name, is_active FROM maps ORDER BY map_id").fetchall()
    if map_id is None:                                 # 지정 없음 → 현재 지도, 없으면 첫 지도
        act = next((m for m in maps if m["is_active"]), None)
        map_id = act["map_id"] if act else (maps[0]["map_id"] if maps else None)
    pg = {"rows": [], "total": 0, "page": 1, "pages": 1, "size": PAGE_SIZE}
    if map_id is not None:
        pg = page_rows(conn, sql.SQL("SELECT * FROM {} WHERE map_id = %s").format(sql.Identifier(table)).as_string(conn),
                       TABLES[table]["order"], page, (map_id,))
    run = conn.execute("SELECT run_id FROM runs WHERE status = 'running' AND map_id = %s", (map_id,)).fetchone() \
        if map_id is not None else None
    return {"meta": meta, **pg, "map_id": map_id, "running_run": run["run_id"] if run else None,
            "maps": [{"value": m["map_id"], "label": f"{m['map_id']} · {m['name']}" + (" (현재)" if m["is_active"] else "")}
                     for m in maps]}


@app.post("/api/master/{table}")
async def add_row(table: str, request: Request):
    """정보 탭 행 추가 — users는 bcrypt 해시로 role 'user' 계정, 그 밖은 TABLES의 add 컬럼만 INSERT.
    settings는 updated_by, robots · aps는 is_active = false, aps · facilities는 고른 지도(순찰 중이면 409)"""
    user = current_user(request)
    meta = table_meta(table)
    if "add" not in meta["ops"]:
        raise HTTPException(405, "이 표는 행 추가를 지원하지 않습니다")
    body = await request.json()
    try:
        with pool.connection() as conn:
            if table == "users":                       # 계정 — 비밀번호는 DB crypt로 해시, 권한은 항상 user
                username = str(body.get("username", "")).strip()
                password = str(body.get("password", ""))
                if not username:
                    raise HTTPException(400, "「아이디」를 입력하세요")
                if len(password) < 4:
                    raise HTTPException(400, "비밀번호는 4자 이상이어야 합니다")
                if username == SUPER_USER:
                    raise HTTPException(400, f"「{SUPER_USER}」은 수퍼 계정 이름이라 쓸 수 없습니다")
                conn.execute("INSERT INTO users (username, password_hash, role) "
                             "VALUES (%s, crypt(%s, gen_salt('bf', 12)), 'user')", (username, password))
                return {"ok": True}
            cols, vals = [], []
            for c in meta["columns"]:                  # add 컬럼만 — 값이 없으면 기본값, coerce로 형식 확인
                if not c["add"]:
                    continue
                v = body.get(c["name"])
                if v is None:
                    v = c["default"]
                cols.append(sql.Identifier(c["name"]))
                vals.append(coerce(c, v))
            if table == "settings":
                cols.append(sql.Identifier("updated_by"))
                vals.append(user["user_id"])
            if table in REPORTED_ACTIVE_TABLES:      # 보고를 받기 전에는 비활성
                cols.append(sql.Identifier("is_active"))
                vals.append(False)
            if meta.get("map_scoped"):               # 지도에 속한 표 — 목록에서 고른 지도에 추가
                try:
                    mid = int(body.get("map_id"))
                except (TypeError, ValueError):
                    raise HTTPException(400, "지도를 먼저 고르세요")
                get_map(conn, mid)
                ensure_no_running_run(conn, mid, {"aps": "AP를", "facilities": "설비를"}[table])
                cols.append(sql.Identifier("map_id"))
                vals.append(mid)
            q = sql.SQL("INSERT INTO {} ({}) VALUES ({})").format(
                sql.Identifier(table), sql.SQL(", ").join(cols), sql.SQL(", ").join(sql.Placeholder() * len(vals)))
            conn.execute(q, vals)
    except psycopg.Error as e:
        raise db_error(e)
    return {"ok": True}


@app.patch("/api/master/{table}/{pk}")
async def update_row(table: str, pk: str, request: Request):
    """정보 탭 행 수정 — body에 있는 edit 컬럼만 UPDATE (users는 update_user).
    순찰 중인 지도의 AP · 설비 · 도크는 409. maps(도크) 수정 뒤 연결된 제어 PC에 config 다시 보냄"""
    user = current_user(request)
    meta = table_meta(table)
    if "edit" not in meta["ops"]:
        raise HTTPException(405, "이 표는 수정할 수 없습니다")
    body = await request.json()
    if table == "users":
        return update_user(pk_value(meta, pk), body)
    sets, vals = [], []
    for c in meta["columns"]:                           # 보낸 값 중 edit 컬럼만 SET
        if c["edit"] and c["name"] in body:
            sets.append(sql.SQL("{} = {}").format(sql.Identifier(c["name"]), sql.Placeholder()))
            vals.append(coerce(c, body[c["name"]]))
    if not sets:
        raise HTTPException(400, "바꿀 값이 없습니다")
    if table == "settings":
        sets.append(sql.SQL("updated_by = {}").format(sql.Placeholder()))
        vals.append(user["user_id"])
    key = pk_value(meta, pk)
    where, wvals = sql.SQL("{} = {}").format(sql.Identifier(meta["pk"]), sql.Placeholder()), [key]
    try:
        with pool.connection() as conn:
            with conn.transaction():
                mid = row_map_id(conn, table, key, request)
                if mid is not None:
                    ensure_no_running_run(conn, mid, LOCK_WHAT[table])    # 순찰 중인 지도의 AP · 설비 · 도크는 잠금
                    if table == "facilities":           # 설비 키 = (map_id, facility_id)
                        where = sql.SQL("{} AND map_id = {}").format(where, sql.Placeholder())
                        wvals.append(mid)
                q = sql.SQL("UPDATE {} SET {} WHERE {}").format(sql.Identifier(table), sql.SQL(", ").join(sets), where)
                n = conn.execute(q, [*vals, *wvals]).rowcount
    except psycopg.Error as e:
        raise db_error(e)
    if n == 0:
        raise HTTPException(404, "행이 없습니다")
    if table == "maps":                                 # 도크가 바뀜 → 연결된 제어 PC 모두에 config(현재 지도 기준) 다시 보냄
        push_config()
    return {"ok": True}


LOCK_WHAT = {"aps": "AP를", "facilities": "설비를", "maps": "도크 접근 자세를"}    # 순찰 중 잠금 안내 문구 (표별)


def row_map_id(conn, table: str, key, request: Request) -> int | None:
    """그 행이 속한 지도 번호 (잠금 확인용). maps는 자기 번호, 설비는 ?map_id= 필수, AP는 행에서 읽음"""
    if table == "maps":
        return int(key)
    if table == "facilities":
        try:
            return int(request.query_params["map_id"])
        except (KeyError, ValueError):
            raise HTTPException(400, "설비는 지도 번호(map_id)와 함께 지정해야 합니다")
    if table == "aps":
        r = conn.execute("SELECT map_id FROM aps WHERE ap_id = %s", (key,)).fetchone()
        if not r:
            raise HTTPException(404, "행이 없습니다")
        return r["map_id"]
    return None


def update_user(user_id: int, body: dict[str, Any]):
    """일반 계정: 아이디·비밀번호 / 수퍼 계정 admin: 비밀번호만. 비밀번호는 입력했을 때만 바꿈."""
    username = body.get("username")
    password = str(body.get("password") or "")
    try:
        with pool.connection() as conn:
            row = conn.execute("SELECT username FROM users WHERE user_id = %s", (user_id,)).fetchone()
            if not row:
                raise HTTPException(404, "행이 없습니다")
            sets, vals = [], []
            # 아이디는 바뀌었을 때만 — admin 이름 변경 · 다른 계정을 admin으로 바꾸기는 거부
            if username is not None and str(username).strip() != row["username"]:
                username = str(username).strip()
                if row["username"] == SUPER_USER:
                    raise HTTPException(400, "수퍼 계정 admin은 비밀번호만 바꿀 수 있습니다")
                if not username:
                    raise HTTPException(400, "「아이디」를 입력하세요")
                if username == SUPER_USER:
                    raise HTTPException(400, f"「{SUPER_USER}」은 수퍼 계정 이름이라 쓸 수 없습니다")
                sets.append("username = %s"); vals.append(username)
            if password:
                if len(password) < 4:
                    raise HTTPException(400, "비밀번호는 4자 이상이어야 합니다")
                sets.append("password_hash = crypt(%s, gen_salt('bf', 12))"); vals.append(password)
            if not sets:
                raise HTTPException(400, "바꿀 값이 없습니다 (비밀번호는 바꿀 때만 입력)")
            conn.execute(f"UPDATE users SET {', '.join(sets)} WHERE user_id = %s", (*vals, user_id))
    except psycopg.Error as e:
        raise db_error(e)
    if isinstance(username, str) and username:     # 로그인 중인 세션의 이름도 맞춤
        with MEM_LOCK:
            for s in SESSIONS.values():
                if s["user_id"] == user_id:
                    s["username"] = username
    return {"ok": True}


def pk_value(meta, pk: str):
    """URL의 기본키 문자열 → 컬럼 형식 (int 컬럼이면 int, 아니면 400)"""
    col = next(c for c in meta["columns"] if c["name"] == meta["pk"])
    if col["type"] == "int":
        try:
            return int(pk)
        except ValueError:
            raise HTTPException(400, "번호 형식이 맞지 않습니다")
    return pk


@app.delete("/api/master/{table}/{pk}")
def delete_row(table: str, pk: str, request: Request):
    """정보 탭 행 삭제 — admin 계정은 거부, maps는 delete_map(딸린 격자 · AP · 설비 함께), AP · 설비는 순찰 중이면 409.
    다른 표가 참조하면 409(외래키). 지운 계정의 세션도 메모리에서 지움"""
    user = current_user(request)
    meta = table_meta(table)
    if "delete" not in meta["ops"]:
        raise HTTPException(405, "이 표는 행 삭제를 지원하지 않습니다")
    key = pk_value(meta, pk)
    try:
        with pool.connection() as conn:
            if table == "users":                       # 수퍼 계정 보호 (DB 트리거도 막음)
                row = conn.execute("SELECT username FROM users WHERE user_id = %s", (key,)).fetchone()
                if row and row["username"] == SUPER_USER:
                    raise HTTPException(400, "수퍼 계정 admin은 삭제할 수 없습니다")
            if table == "maps":                        # 지도 — 딸린 표와 함께 (진행 중 run 포함 기록이 있으면 외래키로 409)
                try:
                    return delete_map(conn, key)
                except psycopg.errors.ForeignKeyViolation as e:
                    raise HTTPException(409, "사용 중 — 이 지도의 순찰 · 측정 · 판정 기록이 있어 지울 수 없습니다 ("
                                        + (e.diag.message_primary or "") + ")")
            mid = row_map_id(conn, table, key, request)     # AP · 설비는 속한 지도 → 순찰 중 잠금 확인
            q = sql.SQL("DELETE FROM {} WHERE {} = %s").format(sql.Identifier(table), sql.Identifier(meta["pk"]))
            params = [key]
            if mid is not None:
                ensure_no_running_run(conn, mid, LOCK_WHAT[table])
                if table == "facilities":
                    q = sql.SQL("{} AND map_id = %s").format(q)
                    params.append(mid)
            n = conn.execute(q, params).rowcount
    except psycopg.Error as e:
        raise db_error(e, "비활성(is_active = false)으로 바꾸세요. " if table in ("facilities", "aps") else "")
    if n == 0:
        raise HTTPException(404, "행이 없습니다")
    if table == "users":                         # 지운 계정의 세션도 지움
        for t in session_tokens_of(key):
            session_drop(t)
    return {"ok": True}


# ---------------------------------------------------------------------------
# 지도 — 업로드 · 현재 지도 선택 · 격자 생성 · 삭제
# ---------------------------------------------------------------------------
def slug(text: str) -> str:
    """지도 이름 → 파일 이름에 쓸 수 있는 영문·숫자·_- 문자열 (비면 "map")"""
    s = re.sub(r"[^A-Za-z0-9_-]+", "_", text).strip("_")
    return s or "map"


@app.post("/api/maps/upload")
async def upload_map(request: Request,
                     name: str = Form(...), robot_map_path: str = Form(...), grid_size_m: float = Form(1.0),
                     yaml_file: UploadFile = File(...), pgm_file: UploadFile = File(...)):
    """지도 등록 — yaml(resolution · origin, yaw 0 · negate 0만)과 pgm을 읽어 PNG로 MEDIA_DIR/maps/에 저장 + maps INSERT.
    격자는 만들지 않음(현재 지도로 고를 때 생성). DB 실패면 저장한 PNG를 지움"""
    current_user(request)
    name, robot_map_path = name.strip(), robot_map_path.strip()
    if not name or not robot_map_path:
        raise HTTPException(400, "이름과 제어 PC 지도 경로를 입력하세요")
    if not (grid_size_m > 0):
        raise HTTPException(400, "격자 크기는 0보다 커야 합니다")
    try:
        meta = yaml.safe_load((await yaml_file.read()).decode("utf-8"))
        resolution = float(meta["resolution"])
        ox, oy, oyaw = (float(v) for v in meta["origin"][:3])
    except Exception:
        raise HTTPException(400, "yaml에서 resolution·origin을 읽지 못했습니다")
    if resolution <= 0:
        raise HTTPException(400, "yaml의 resolution이 0보다 커야 합니다")
    if abs(oyaw) > 1e-9:
        raise HTTPException(400, f"yaml origin의 yaw가 0이어야 합니다 (지금 {oyaw})")
    if int(meta.get("negate", 0) or 0) != 0:
        raise HTTPException(400, "negate = 1 지도는 지원하지 않습니다 (negate = 0으로 저장해 주세요)")
    try:
        img = Image.open(pgm_file.file)
        img.load()
        img = img.convert("L")
    except Exception:
        raise HTTPException(400, "pgm 이미지를 읽지 못했습니다")
    fname = f"{slug(name)}_{int(time.time())}.png"
    path = MEDIA_DIR / "maps" / fname
    img.save(path, format="PNG")                        # 크기는 바꾸지 않음
    try:
        with pool.connection() as conn:
            row = conn.execute(
                "INSERT INTO maps (name, robot_map_path, image_key, resolution_m, origin_x, origin_y, grid_size_m) "
                "VALUES (%s, %s, %s, %s, %s, %s, %s) RETURNING map_id",
                (name, robot_map_path, f"maps/{fname}", resolution, ox, oy, grid_size_m)).fetchone()
    except psycopg.Error as e:
        path.unlink(missing_ok=True)
        raise db_error(e)
    return {"ok": True, "map_id": row["map_id"], "width": img.width, "height": img.height}


def split_sectors(cells) -> dict[tuple[int, int], int]:
    """칸 (ix, iy) 목록 → 섹터 번호 (3차 — 임시 규칙, 나중에 이 함수만 바꿔 끼움).
    ix 순으로 칸 수가 절반에 가장 가까운 열에서 나눔: 왼쪽(그 열까지) = 섹터 1, 오른쪽 = 섹터 2.
    같은 차이면 왼쪽 열에서 나눔. 열이 하나뿐이면 모두 섹터 1.
    (DB 01_schema.sql 업그레이드 구간 ⑥ · 더미 make_dummy.py도 같은 규칙)"""
    cells = [(int(a), int(b)) for a, b in cells]
    cols: dict[int, int] = {}
    for ix, _ in cells:                                # 열(ix)별 칸 수
        cols[ix] = cols.get(ix, 0) + 1
    order = sorted(cols)
    total, left, split, best = len(cells), 0, None, None
    for ix in order[:-1]:                              # 마지막 열 전까지 — 왼쪽 누적 칸 수가 절반에 가장 가까운 열
        left += cols[ix]
        diff = abs(2 * left - total)
        if best is None or diff < best:
            best, split = diff, ix
    return {c: (1 if split is None or c[0] <= split else 2) for c in cells}


def zone_sectors(cells, ox: float, oy: float, g: float) -> dict[tuple[int, int], int]:
    """ros 모드 — 섹터를 inspection_manager 와 같은 zones.yaml 폴리곤으로 (Z1 → 1, Z2 → 2). 칸 중심이 어느 폴리곤에도
    없으면(벽 바깥 가장자리 칸) 중심이 가장 가까운 폴리곤의 섹터"""
    cen = [(n, sum(p[0] for p in poly) / len(poly), sum(p[1] for p in poly) / len(poly)) for n, poly in ZONES]
    out = {}
    for ix, iy in [(int(a), int(b)) for a, b in cells]:
        cx, cy = ox + (ix + 0.5) * g, oy + (iy + 0.5) * g
        out[(ix, iy)] = next((n for n, poly in ZONES if _in_poly(cx, cy, poly)), None) or \
            min(cen, key=lambda z: (z[1] - cx) ** 2 + (z[2] - cy) ** 2)[0]
    return out


def _in_poly(x: float, y: float, poly) -> bool:
    """점이 다각형 안인가 (ray casting)"""
    inside, j = False, len(poly) - 1
    for i in range(len(poly)):
        (xi, yi), (xj, yj) = poly[i], poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def generate_cells(conn, m: dict[str, Any]) -> int:
    """지도 PNG의 흰 부분(빈 공간)이 조금이라도 걸치는 칸은 모두 grid_cells에 넣고 섹터를 정함.
    벽 여유(wall_margin_m)로 빼지 않음 — 벽 가까이 좁게 남은 흰 부분도 칸이 생김 (경로 생성에서만 벽 여유를 씀).
    COPY로 넣고 만든 칸 수를 돌려줌 (트랜잭션 · 기존 칸 삭제는 호출한 쪽). GRID_CACHE의 그 지도는 비움"""
    png = MEDIA_DIR / m["image_key"]
    if not png.exists():
        raise HTTPException(500, f"지도 이미지 파일이 없습니다: {m['image_key']}")
    res, ox, oy, g = float(m["resolution_m"]), float(m["origin_x"]), float(m["origin_y"]), float(m["grid_size_m"])

    raw = np.asarray(Image.open(png).convert("L"))
    occ = 1.0 - raw.astype(np.float32) / 255.0             # negate = 0
    safe = (occ < FREE_THRESH) & (raw != UNKNOWN_PIXEL)    # 흰 부분. map_saver의 미탐색 205(회색)는 제외
    rows, cols = np.nonzero(safe)
    if rows.size == 0:
        return 0
    h = raw.shape[0]
    # 흰 픽셀 중심 → 지도 좌표(m, 이미지 행은 위→아래이므로 y 뒤집음) → 칸 (ix, iy), 중복 제거
    x = ox + (cols + 0.5) * res
    y = oy + (h - rows - 0.5) * res
    ix = np.floor((x - ox) / g).astype(np.int64)
    iy = np.floor((y - oy) / g).astype(np.int64)
    cells = np.unique(np.stack([ix, iy], axis=1), axis=0)
    sector = zone_sectors(cells, ox, oy, g) if RUN_OWNER == "ros" else split_sectors(cells)
    GRID_CACHE.pop(m["map_id"], None)                  # 메모리 격자 비움 (다음 grid_of가 DB에서 다시 읽음)
    with conn.cursor().copy("COPY grid_cells (map_id, ix, iy, center_x, center_y, sector) FROM STDIN") as cp:
        for cx, cy in cells:
            cp.write_row((m["map_id"], int(cx), int(cy), ox + (cx + 0.5) * g, oy + (cy + 0.5) * g,
                          sector[(int(cx), int(cy))]))
    return int(len(cells))


def get_map(conn, map_id: int) -> dict[str, Any]:
    """maps 행 하나. 없으면 404"""
    m = conn.execute("SELECT * FROM maps WHERE map_id = %s", (map_id,)).fetchone()
    if not m:
        raise HTTPException(404, "지도가 없습니다")
    return m


@app.post("/api/aps/{ap_id}/target")
def set_target_ap(ap_id: int, request: Request):
    """측정 AP 지정 (3-3차) — 지도마다 하나(aps.is_target), 활성 AP만. 순찰 중인 지도는 잠금.
    같은 지도의 다른 AP는 is_target = false로 (한 트랜잭션)"""
    current_user(request)
    try:
        with pool.connection() as conn:
            with conn.transaction():
                a = conn.execute("SELECT ap_id, map_id, ssid, is_active FROM aps WHERE ap_id = %s", (ap_id,)).fetchone()
                if not a:
                    raise HTTPException(404, "AP가 없습니다")
                if not a["is_active"]:
                    raise HTTPException(400, f"「{a['ssid']}」은 비활성입니다 — client가 이 SSID 값을 받고 있을 때(활성)만 측정 AP로 고를 수 있습니다")
                ensure_no_running_run(conn, a["map_id"], "측정 AP를")
                conn.execute("UPDATE aps SET is_target = false WHERE map_id = %s AND is_target", (a["map_id"],))
                conn.execute("UPDATE aps SET is_target = true WHERE ap_id = %s", (ap_id,))
    except psycopg.Error as e:
        raise db_error(e)
    return {"ok": True, "ssid": a["ssid"]}


@app.post("/api/maps/{map_id}/activate")
def activate_map(map_id: int, request: Request):
    """현재 지도로 선택(maps.is_active 하나만 true). 그 지도에 격자가 없으면 이때 생성 (선택된 지도만).
    다른 지도에서 순찰 중이면 409. 끝나면 연결된 제어 PC에 config 다시 보냄"""
    current_user(request)
    try:
        with pool.connection() as conn:
            with conn.transaction():
                m = get_map(conn, map_id)
                # 순찰(run) 진행 중에는 현재 지도를 다른 지도로 바꾸지 않음 (3-2차) — 같은 지도를 다시 고르는 것은 그대로
                r = conn.execute("SELECT run_id, map_id FROM runs WHERE status = 'running'").fetchone()
                if r and r["map_id"] != map_id:
                    raise HTTPException(409, f"순찰(run {r['run_id']})이 진행 중입니다. 순찰이 끝난 뒤 현재 지도를 바꾸세요")
                conn.execute("UPDATE maps SET is_active = false WHERE is_active AND map_id <> %s", (map_id,))
                conn.execute("UPDATE maps SET is_active = true WHERE map_id = %s", (map_id,))
                n = conn.execute("SELECT count(*) AS n FROM grid_cells WHERE map_id = %s", (map_id,)).fetchone()["n"]
                created = 0
                if n == 0:
                    created = generate_cells(conn, m)
                    n = created
    except psycopg.Error as e:
        raise db_error(e)
    push_config()                                      # 현재 지도가 바뀜 → 제어 PC에 도크 접근 자세 다시 보냄
    return {"ok": True, "cell_count": n, "created": created}


def ensure_no_running_run(conn, map_id: int, what: str = "격자를"):
    """순찰(run)이 진행 중인 지도면 409 — 격자 · AP · 설비 · 도크가 순찰 도중 바뀌지 않도록 (3-2차).
    what = 안내 문구의 대상(기본 「격자를」)"""
    r = conn.execute("SELECT run_id FROM runs WHERE status = 'running' AND map_id = %s", (map_id,)).fetchone()
    if r:
        raise HTTPException(409, f"순찰(run {r['run_id']})이 진행 중인 지도입니다. 순찰이 끝난 뒤 {what} 바꾸세요")


@app.post("/api/maps/{map_id}/grid")
def regenerate_grid(map_id: int, request: Request):
    """격자 다시 생성 — 그 지도 grid_cells 삭제 후 generate_cells (한 트랜잭션, 순찰 중이면 409).
    칸을 참조하는 기록(표본 · 후보 등)이 있으면 외래키로 거부될 수 있음"""
    current_user(request)
    try:
        with pool.connection() as conn:
            with conn.transaction():
                m = get_map(conn, map_id)
                ensure_no_running_run(conn, map_id)
                conn.execute("DELETE FROM grid_cells WHERE map_id = %s", (map_id,))
                n = generate_cells(conn, m)
    except psycopg.Error as e:
        raise db_error(e)
    return {"ok": True, "cell_count": n}


@app.delete("/api/maps/{map_id}/grid")
def delete_grid(map_id: int, request: Request):
    """그 지도의 격자 칸 모두 삭제 + GRID_CACHE 비움 (순찰 중이면 409)"""
    current_user(request)
    try:
        with pool.connection() as conn:
            get_map(conn, map_id)
            ensure_no_running_run(conn, map_id)
            n = conn.execute("DELETE FROM grid_cells WHERE map_id = %s", (map_id,)).rowcount
            GRID_CACHE.pop(map_id, None)
    except psycopg.Error as e:
        raise db_error(e)
    return {"ok": True, "deleted": n}


def delete_map(conn, map_id: int):
    """지도 삭제 — facilities · aps · grid_cells · maps 행을 한 트랜잭션으로 지우고, 커밋 뒤 GRID_CACHE · 지도 PNG 삭제"""
    with conn.transaction():
        m = get_map(conn, map_id)
        # 그 지도에 속한 격자 칸 · AP · 설비를 함께 지움 — 순찰 · 측정 · 판정 기록이 참조하면 외래키로 거부(사용 중)
        for t in ("facilities", "aps", "grid_cells"):
            conn.execute(sql.SQL("DELETE FROM {} WHERE map_id = %s").format(sql.Identifier(t)), (map_id,))
        conn.execute("DELETE FROM maps WHERE map_id = %s", (map_id,))
    GRID_CACHE.pop(map_id, None)
    (MEDIA_DIR / m["image_key"]).unlink(missing_ok=True)
    return {"ok": True}


@app.get("/api/maps/active")
def active_map(request: Request):
    """모니터 · 이력 탭 지도 — 현재 지도 행 + 이미지 크기 · URL, 격자 칸, 섹터 요약, 진행 중 run, AP 설치 위치, 설비. 읽기만"""
    current_user(request)
    with pool.connection() as conn:
        m = conn.execute("SELECT m.*, (SELECT count(*) FROM grid_cells g WHERE g.map_id = m.map_id) AS cell_count "
                         "FROM maps m WHERE m.is_active").fetchone()
    if not m:
        return {"map": None}
    png = MEDIA_DIR / m["image_key"]
    if png.exists():
        with Image.open(png) as im:
            m["width"], m["height"] = im.size
    else:
        m["width"] = m["height"] = None
    m["image_url"] = "/media/" + m["image_key"]
    # 모니터 격자 표시용 — [cell_id, ix, iy, sector]. 칸 범위는 origin + ix·grid_size_m 로 화면에서 계산
    with pool.connection() as conn:
        m["cells"] = [[c["cell_id"], c["ix"], c["iy"], c["sector"]] for c in conn.execute(
            "SELECT cell_id, ix, iy, sector FROM grid_cells WHERE map_id = %s ORDER BY cell_id", (m["map_id"],))]
        run = conn.execute("SELECT run_id, sector_robots FROM runs WHERE status = 'running' AND map_id = %s",
                           (m["map_id"],)).fetchone()
    m["sectors"] = [{"sector": s_, "cells": sum(1 for c in m["cells"] if c[3] == s_),
                     "robot": DEFAULT_SECTOR_ROBOTS.get(str(s_))} for s_ in SECTORS]
    m["running_run"] = run["run_id"] if run else None          # 진행 중이면 격자 다시 생성 금지 (화면 안내용)
    # 지도 위 표시 (3차) — 도크(maps.dock_poses) · AP 설치 위치 · 점검 설비. 음영은 측정 결과로만(이력 탭 히트맵 — /api/runs/{id}의 heat)
    with pool.connection() as conn:
        m["aps"] = conn.execute("SELECT ap_id, ssid, x, y, is_active, is_target FROM aps WHERE map_id = %s AND x IS NOT NULL "
                                "ORDER BY ssid", (m["map_id"],)).fetchall()
        m["facilities"] = conn.execute(
            "SELECT facility_id, facility_type, has_gauge, expected_x, expected_y, view_pose, is_active "
            "FROM facilities WHERE map_id = %s ORDER BY facility_id", (m["map_id"],)).fetchall()
    return {"map": m}


# ---------------------------------------------------------------------------
# 제어 PC(client page · scv_agent) 연결
#   POST /api/client/login — client page 로그인 확인 (제어 PC는 DB에 접속하지 않으므로 관제 PC가 대신 확인)
#   WebSocket /ws/agent   — 첫 메시지 hello { robot_id, active_task_id, … } → robots.is_active = true, 연결이 끊기면 false
#                           관제 PC → welcome · config(+ agent_resync: 배정된 run이 있으면 run_config · task_assign, 하던 task가 DB ACTIVE가 아니면 task_cancel) 를 보낸 뒤
#                           제어 PC → status(1 Hz) · task_progress · task_accepted · task_rejected 를 받음
#                           순찰 명령(run_config · task_assign · task_cancel · run_cancel · run_end)은 send_agent로 보냄
#   메시지 틀: { "type", "msg_id", "ts", "body" } (구현방안 6.2절)
# ---------------------------------------------------------------------------
@app.post("/api/client/login")
async def client_login(request: Request):
    """client page 로그인 확인 — users 비밀번호 확인만(세션 없음), admin · user 모두 → {user: {user_id, username, role}}"""
    body = await request.json()
    username = str(body.get("username", "")).strip()
    password = str(body.get("password", ""))
    if not username or not password:
        raise HTTPException(400, "아이디와 비밀번호를 입력하세요")

    def check():
        """DB 비밀번호 확인 (스레드 풀에서 실행) → 맞으면 users 행, 아니면 None"""
        with pool.connection() as conn:
            return conn.execute(
                "SELECT user_id, username, role FROM users "
                "WHERE username = %s AND password_hash = crypt(%s, password_hash)",
                (username, password)).fetchone()

    user = await run_in_threadpool(check)
    if not user:                                       # admin · user 모두 client page에 들어갈 수 있음
        raise HTTPException(401, "아이디 또는 비밀번호가 맞지 않습니다")
    return {"user": user}


AGENTS: dict[str, WebSocket] = {}                      # robot_id → 지금 연결된 제어 PC (MEM_LOCK 없이 씀)
_msg_seq = 0                                           # 관제 PC가 보내는 메시지 번호 (msg_id "s-000001")


def envelope(type_: str, body: dict[str, Any]) -> dict[str, Any]:
    """관제 PC → 제어 PC 메시지 틀 {type, msg_id, ts(UTC ms, Z), body}"""
    global _msg_seq
    _msg_seq += 1
    return {"type": type_, "msg_id": f"s-{_msg_seq:06d}",
            "ts": datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z"), "body": body}


def agent_config(robot_id: str) -> dict[str, Any]:
    """제어 PC에 보내는 WS config 본문 (3-2차) — {map: {map_id, name, robot_map_path} | None, dock_pose: {x, y, yaw} | None}.
    현재 지도와 그 지도의 이 로봇 도크 접근 자세(maps.dock_poses[robot_id]).
    client는 도크 자세를 받으면 로봇이 도킹 중(실행 중 task 없음)이거나 위치를 모를 때 {ns}/initialpose로 로봇 시작 위치를 지정"""
    with pool.connection() as conn:
        m = conn.execute("SELECT map_id, name, robot_map_path, dock_poses FROM maps WHERE is_active").fetchone()
    if not m:
        return {"map": None, "dock_pose": None}
    return {"map": {"map_id": m["map_id"], "name": m["name"], "robot_map_path": m["robot_map_path"]},
            "dock_pose": (m["dock_poses"] or {}).get(robot_id)}


async def _push_config_async():
    """연결된 제어 PC마다 WS config 보냄 (이벤트 루프에서 실행 — SEND_LOCKS는 쓰지 않음)"""
    for rid, ws in list(AGENTS.items()):
        try:
            await ws.send_json(envelope("config", await run_in_threadpool(agent_config, rid)))
        except Exception as e:                          # 끊긴 연결은 ws_agent가 정리
            print(f"[agent] {rid} config 보내기 실패: {e}", flush=True)


def push_config():
    """연결된 모든 제어 PC에 config를 다시 보냄 (현재 지도 변경 · 도크 수정 때). 동기 API에서 불러도 됨"""
    if MAIN_LOOP is not None and AGENTS:
        asyncio.run_coroutine_threadsafe(_push_config_async(), MAIN_LOOP)


def set_robot_active(robot_id: str, active: bool) -> bool:
    """robots.is_active 변경. 등록된 로봇이면 True."""
    with pool.connection() as conn:
        return conn.execute("UPDATE robots SET is_active = %s WHERE robot_id = %s",
                            (active, robot_id)).rowcount == 1


@app.websocket("/ws/agent")
async def ws_agent(ws: WebSocket):
    """제어 PC 연결 (로봇 1대당 하나) — hello(10 s 안) 확인 → robots.is_active = true · AGENTS 등록 →
    welcome · config · (순찰 중 재접속이면 agent_resync 메시지) 보낸 뒤 받기 반복. 끊기면 AGENTS에서 빼고 비활성.
    인증 없음 — 등록된 robot_id면 받아들임 (등록 안 된 로봇은 error UNKNOWN_ROBOT 후 4404로 닫음)"""
    await ws.accept()
    if RUN_OWNER == "ros":                             # ros 모드: 로봇 명령 · 상태는 ROS 로만 (client_page 를 쓰지 않음)
        await ws.close(code=4409, reason="ROS owns runs; web agent disabled")
        return
    robot_id = None
    try:
        try:                                           # 첫 메시지 = hello 확인
            hello = await asyncio.wait_for(ws.receive_json(), timeout=10)
        except (asyncio.TimeoutError, json.JSONDecodeError, KeyError):
            await ws.close(code=4400, reason="first message must be hello")
            return
        body = hello.get("body") or {}
        if hello.get("type") != "hello" or not body.get("robot_id"):
            await ws.close(code=4400, reason="first message must be hello")
            return
        robot_id = str(body["robot_id"])
        # robots.is_active = true — 등록되지 않은 로봇이면 error 보내고 닫음
        if not await run_in_threadpool(set_robot_active, robot_id, True):
            await ws.send_json(envelope("error", {"reason": "UNKNOWN_ROBOT", "robot_id": robot_id}))
            await ws.close(code=4404, reason="unknown robot")
            robot_id = None
            return
        old = AGENTS.get(robot_id)
        AGENTS[robot_id] = ws
        if old is not None:                            # 같은 로봇의 이전 연결은 닫음 (재접속)
            try:
                await old.close(code=4409, reason="replaced by new connection")
            except Exception:
                pass
        print(f"[agent] {robot_id} 연결 · hello {body} → 활성", flush=True)
        def _welcome():
            """welcome 본문용 — 그 로봇이 배정된 진행 중 run · ACTIVE task_id (스레드 풀에서 실행)"""
            with pool.connection() as conn:
                return run_of_robot(conn, robot_id)
        run, active = await run_in_threadpool(_welcome)
        await ws.send_json(envelope("welcome", {
            "server_time": datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z"),
            "run": run and run["run_id"], "active_task_id": active}))
        await ws.send_json(envelope("config", await run_in_threadpool(agent_config, robot_id)))
        for t, b in await run_in_threadpool(agent_resync, robot_id, body):   # 순찰 중 재접속 — run_config · task 다시
            await ws.send_json(envelope(t, b))
        while True:                                    # 제어 PC → 관제 PC 메시지 처리 (하나가 실패해도 연결은 유지)
            msg = await ws.receive_json()
            kind = msg.get("type")
            b = msg.get("body") or {}
            try:
                if kind == "status":                   # 1 Hz — 메모리 최신값 + robot_status_log · 받은 SSID → AP 활성
                    await run_in_threadpool(store_status, robot_id, b, msg.get("ts"))
                    await run_in_threadpool(mark_aps_seen, robot_id, b.get("ssids") or [])
                elif kind == "task_progress":          # tasks.progress (ACTIVE task만)
                    await run_in_threadpool(on_task_progress, robot_id, b)
                elif kind == "task_accepted":          # 기록 없음 — 로그만
                    print(f"[agent] {robot_id} task 받음 {b.get('task_id')}", flush=True)
                elif kind == "task_rejected":          # 실행 못 함(MAP_MISMATCH 등) → 실패와 같게 (on_task_result ok=False)
                    await run_in_threadpool(on_task_result, robot_id, str(b.get("task_id")), False, b.get("reason") or "REJECTED")
                else:
                    print(f"[agent] {robot_id} ← {kind} (처리하지 않는 메시지)", flush=True)
            except (psycopg.Error, TypeError, ValueError, KeyError) as e:
                print(f"[agent] {robot_id} {kind} 처리 실패: {e}", flush=True)
    except WebSocketDisconnect:
        pass
    finally:
        # 이 연결이 아직 그 로봇의 현재 연결일 때만 정리 (재접속으로 바뀐 이전 연결은 그대로 둠)
        # ROBOT_STATE는 남김 — status_timeout_s 뒤 state_watchdog가 E3 기록
        if robot_id and AGENTS.get(robot_id) is ws:
            AGENTS.pop(robot_id, None)
            await run_in_threadpool(set_robot_active, robot_id, False)
            print(f"[agent] {robot_id} 연결 끊김 → 비활성", flush=True)


# ---------------------------------------------------------------------------
# 로봇 상태 — 메모리 ROBOT_STATE (로봇별 최신 status), 보존 = settings.status_timeout_s(30 s)
#   state_watchdog가 1초마다 확인 → 그동안 status가 없으면 상태를 지우고 E3(응답 없음)를 robot_events에 기록
#   (순찰 중이면 on_robot_lost로 그 로봇 task 실패)
# ---------------------------------------------------------------------------
STATE_FIELDS = ("ts", "run_id", "task_id", "nav_state", "progress", "x", "y", "yaw", "battery_pct", "is_docked", "estop")  # 메모리에 두는 status 필드 (ssids 등 나머지는 버림)
LAST_E3: dict[str, str] = {}                           # robot_id → 마지막 E3 시각 (화면 표시용)
_ttl_cache = {"v": 30, "t": 0.0}                       # status_timeout_s 캐시 — 값 · 읽은 시각(30 s마다 DB에서 다시)


def status_timeout() -> int:
    """settings.status_timeout_s(초, 최소 1, 없거나 틀리면 30). 30초마다 DB에서 다시 읽음"""
    if time.time() - _ttl_cache["t"] > 30:
        with pool.connection() as conn:
            row = conn.execute("SELECT value FROM settings WHERE key = 'status_timeout_s'").fetchone()
        try:
            _ttl_cache["v"] = max(1, int(float(row["value"]))) if row else 30
        except ValueError:
            _ttl_cache["v"] = 30
        _ttl_cache["t"] = time.time()
    return _ttl_cache["v"]


def _num(v):
    """숫자로 바꿈 — None · 빈 문자열 · 형식 오류는 None"""
    try:
        return None if v is None or v == "" else float(v)
    except (TypeError, ValueError):
        return None


def store_status(robot_id: str, body: dict[str, Any], msg_ts: str | None):
    """WS status 한 건 → ROBOT_STATE[robot_id] 덮어씀(문자열로) + 위치·배터리가 있으면 robot_status_log INSERT.
    ts = body.stamp → 메시지 ts → 지금 순. DB 실패는 로그만"""
    ts = body.get("stamp") or msg_ts or datetime.now(timezone.utc).isoformat()
    vals = {"ts": ts, **{k: body.get(k) for k in STATE_FIELDS if k != "ts"}}
    fields = {k: "" if v is None else (str(v).lower() if isinstance(v, bool) else str(v))
              for k, v in {**vals, "received_at": datetime.now(timezone.utc).isoformat()}.items()}
    with MEM_LOCK:
        ROBOT_STATE[robot_id] = {"fields": fields, "rx": time.monotonic()}
    x, y, yaw, bat = _num(vals["x"]), _num(vals["y"]), _num(vals["yaw"]), _num(vals["battery_pct"])
    if None in (x, y, yaw, bat):                       # 위치·배터리를 모르면 이력은 남기지 않음 (메모리만)
        return
    try:
        with pool.connection() as conn:
            conn.execute(
                "INSERT INTO robot_status_log (robot_id, ts, run_id, task_id, x, y, yaw, battery_pct, nav_state, progress) "
                "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s) ON CONFLICT (robot_id, ts) DO NOTHING",
                (robot_id, ts, vals["run_id"], vals["task_id"] or None, x, y, yaw, max(0.0, min(100.0, bat)),
                 str(vals["nav_state"] or "UNKNOWN")[:16], _num(vals["progress"])))
    except psycopg.Error as e:
        print(f"[status] {robot_id} robot_status_log 저장 실패: {e.diag.message_primary}", flush=True)


def record_e3(robot_id: str):
    """E3(응답 없음) — robot_events INSERT(source control, level warn) · RECENT_EVENTS · LAST_E3 → on_robot_lost"""
    ts = datetime.now(timezone.utc)
    detail = {"reason": "NO_STATUS", "timeout_s": status_timeout(), "agent_connected": robot_id in AGENTS}
    try:
        with pool.connection() as conn:
            run, task = run_of_robot(conn, robot_id)
            add_event(conn, robot_id, {"event_uid": f"control-{uuid.uuid4()}", "run_id": run and run["run_id"],
                                       "task_id": task, "code": "E3", "source": "control", "level": "warn",
                                       "detail": detail, "ts": ts})
        LAST_E3[robot_id] = ts.isoformat()
        print(f"[E3] {robot_id} 응답 없음 — {detail['timeout_s']}초 동안 status 없음", flush=True)
    except psycopg.Error as e:
        print(f"[E3] {robot_id} 기록 실패: {e.diag.message_primary}", flush=True)
        return
    try:
        on_robot_lost(robot_id)                        # 순찰 중이면 그 로봇 task 실패 → 그 섹터만 중단
    except psycopg.Error as e:
        print(f"[E3] {robot_id} task 실패 처리 실패: {e.diag.message_primary}", flush=True)


async def state_watchdog():
    """1초마다 — status가 settings.status_timeout_s 동안 없던 로봇은 상태를 지우고 E3 기록(record_e3),
    만료된 세션 정리, SSID 값이 끊긴 AP 비활성(expire_aps). 오류가 나도 반복은 계속"""
    while True:
        try:
            await asyncio.sleep(1.0)
            limit = await run_in_threadpool(status_timeout)
            now = time.monotonic()
            with MEM_LOCK:
                gone = [rid for rid, st in ROBOT_STATE.items() if now - st["rx"] > limit]
                for rid in gone:
                    ROBOT_STATE.pop(rid, None)
                for t in [t for t, s_ in SESSIONS.items() if s_["expires"] < now]:
                    SESSIONS.pop(t, None)
            for rid in gone:                           # ros 모드: E3는 inspection_manager 알림(ros_bridge)으로만
                if RUN_OWNER == "ros":
                    await run_in_threadpool(set_robot_active, rid, False)
                else:
                    await run_in_threadpool(record_e3, rid)
            await run_in_threadpool(expire_aps, limit)     # SSID 값이 끊긴 AP는 비활성
        except asyncio.CancelledError:
            raise
        except Exception as e:                         # 점검은 멈추지 않음
            print(f"[watchdog] 점검 실패: {e}", flush=True)


def robots_snapshot() -> list[dict[str, Any]]:
    """/ws/ui용 로봇 목록 — 등록된 로봇마다 {robot_id, is_active, connected(AGENTS), state(ROBOT_STATE를 형식 변환 + ttl_s · raw), last_e3}"""
    with pool.connection() as conn:
        robots = conn.execute("SELECT robot_id, is_active FROM robots ORDER BY robot_id").fetchall()
    limit = status_timeout()
    out = []
    for r in robots:
        with MEM_LOCK:
            mem = ROBOT_STATE.get(r["robot_id"])
            st = dict(mem["fields"]) if mem else None
            age = time.monotonic() - mem["rx"] if mem else 0.0
        state = None
        if st:                                         # 문자열로 둔 값을 숫자 · bool로 되돌림
            state = {k: st.get(k) or None for k in (*STATE_FIELDS, "received_at")}
            for k in ("x", "y", "yaw", "battery_pct", "progress"):
                state[k] = _num(state[k])
            for k in ("is_docked", "estop"):
                state[k] = state[k] == "true"
            state["ttl_s"] = max(0, math.ceil(limit - age))       # E3까지 남은 초
            state["raw"] = dict(sorted(st.items()))           # 받은 status 그대로 (모니터에 전체 표시)
        connected = r["robot_id"] in AGENTS or (RUN_OWNER == "ros" and st is not None)
        out.append({"robot_id": r["robot_id"], "is_active": r["is_active"], "connected": connected,
                    "state": state, "last_e3": LAST_E3.get(r["robot_id"])})
    return out


# ===========================================================================
# inspection_manager (3-3차) — 순찰(run) · 로봇별 단계 task · 후보 → 정밀측정 → 확정 · 설비 점검 · 복귀
#   로봇마다 단계: SURVEY_SEGMENT(측정) → REINSPECTION(후보 칸 정밀측정) → FACILITY_CHECK(설비 판정 1회) → DOCK
#   그 로봇의 한 단계 task가 모두 끝나면 다음 단계를 만듦. 로봇마다 ACTIVE task 하나(task_id 순서대로 보냄)
#   실패 → scv_fail_task(그 task FAILED + 그 로봇의 남은 task만 CANCELED) + 복귀 DOCK. 남은 task가 없으면 scv_close_run
#   쓰는 표: runs · tasks · candidates · findings (+ 제어 PC 기록 net_samples · robot_events · precise_measures · inspections · media)
#   3차는 로봇 1대 = 섹터 1개. 경로 · 섹터 · 후보 규칙은 임시(함수 하나씩, 4차에 교체)
# ===========================================================================
import collections

IM_LOCK = threading.Lock()                             # task 번호 · 단계 생성이 겹치지 않게 (inspection_manager 전체)
CURRENT_RUN: dict[str, Any] = {"run_id": None}         # 메모리 — 진행 중 run_id (시작할 때 runs에서 복원)
RECENT_EVENTS: collections.deque = collections.deque(maxlen=1000)   # 메모리 — 최근 이벤트 (시작할 때 robot_events에서)
ROBOT_PLAN: dict[str, list] = {}                         # ros 모드 — robot_id → Nav2 전역 경로 점 [(x, y)] (ros_bridge, 모니터 지도)
ESTOP: dict[str, bool] = {"on": False}                   # ros 모드 — 비상정지 중 (ros_bridge, 「재개」 버튼)
LIVE_CELLS: dict[int, dict[int, dict[int, list[float]]]] = {}       # 메모리 — run → cell → ap → [n, sum, sumsq, last] (히트맵)
AP_SEEN: dict[int, float] = {}                         # ap_id → 그 SSID 값을 마지막으로 받은 시각(monotonic)
GRID_CACHE: dict[int, dict[str, Any]] = {}             # map_id → {"cells": {(ix, iy): (cell_id, sector)}, "ox", "oy", "g"}
PHASES = ("SURVEY_SEGMENT", "REINSPECTION", "FACILITY_CHECK", "DOCK")     # 로봇마다 단계 순서
RUN_TYPES = ("patrol",)                 # 순찰 종류 — 순찰 하나 (task: 측정 → 필요하면 정밀측정 → 탐지 → 복귀)
RUN_SETTINGS = ("rssi_suspect_dbm", "min_samples", "precise_duration_s", "max_pose_retry", "battery_return_pct")  # run_config로 제어 PC에 보내는 기준값


def settings_values(conn) -> dict[str, str]:
    """settings 전체 → {key: value(문자열)}"""
    return {r["key"]: r["value"] for r in conn.execute("SELECT key, value FROM settings")}


def fset(st: dict[str, str], key: str, default: float) -> float:
    """설정값을 float로 — 없거나 형식이 틀리면 default"""
    try:
        return float(st.get(key, default))
    except (TypeError, ValueError):
        return default


def grid_of(conn, map_id: int) -> dict[str, Any]:
    """지도의 격자 (칸 번호 · 섹터) — 칸은 순찰 중 바뀌지 않으므로 메모리에 둠 (격자를 다시 만들면 비움)"""
    g = GRID_CACHE.get(map_id)
    if g is None:
        m = conn.execute("SELECT origin_x, origin_y, grid_size_m FROM maps WHERE map_id = %s", (map_id,)).fetchone()
        cells = {(c["ix"], c["iy"]): (c["cell_id"], c["sector"]) for c in conn.execute(
            "SELECT cell_id, ix, iy, sector FROM grid_cells WHERE map_id = %s", (map_id,))}
        g = GRID_CACHE[map_id] = {"cells": cells, "ox": float(m["origin_x"]), "oy": float(m["origin_y"]),
                                  "g": float(m["grid_size_m"])}
    return g


def cell_at(grid: dict[str, Any], x: float, y: float) -> tuple[int, int] | None:
    """좌표 → (cell_id, sector). 격자 밖이면 None"""
    k = (math.floor((x - grid["ox"]) / grid["g"]), math.floor((y - grid["oy"]) / grid["g"]))
    return grid["cells"].get(k)


# ---------------------------------------------------------------- 경로 (임시 — 함수 하나, 4차에 교체)
def _dilate(mask: np.ndarray, r: int) -> np.ndarray:
    """bool 마스크를 반지름 r 픽셀 원으로 팽창 (가장자리 밖은 True로 봄 — 지도 끝도 벽 취급)"""
    if r <= 0:
        return mask.copy()
    h, w = mask.shape
    pad = np.pad(mask, r, constant_values=True)
    out = mask.copy()
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx * dx + dy * dy <= r * r:
                out |= pad[r + dy:r + dy + h, r + dx:r + dx + w]
    return out


def zigzag(cells: list[tuple[int, int, int]]) -> list[tuple[int, int, int]]:
    """(cell_id, ix, iy) → ix 열마다 iy 방향을 번갈아 (지그재그)"""
    cols: dict[int, list] = {}
    for c in cells:
        cols.setdefault(c[1], []).append(c)
    out = []
    for i, ix in enumerate(sorted(cols)):
        out += sorted(cols[ix], key=lambda c: c[2], reverse=bool(i % 2))
    return out


def plan_survey(m: dict[str, Any], cells: list[tuple[int, int, int]], margin_m: float) -> tuple[list[dict], int]:
    """섹터 칸을 지그재그 순서로 — 칸 안에서 벽 여유(margin_m)를 만족하는 점 중 칸 중심에 가장 가까운 점.
    그런 점이 없는 칸은 건너뜀. 반환: (경로점 [{x, y, yaw}], 건너뛴 칸 수)"""
    raw = np.asarray(Image.open(MEDIA_DIR / m["image_key"]).convert("L"))
    h = raw.shape[0]
    res, ox, oy, g = float(m["resolution_m"]), float(m["origin_x"]), float(m["origin_y"]), float(m["grid_size_m"])
    # 갈 수 있는 곳 = 흰 부분에서 벽(흰 부분 아님)을 margin_m만큼 넓혀 뺀 나머지
    free = ((1.0 - raw.astype(np.float32) / 255.0) < FREE_THRESH) & (raw != UNKNOWN_PIXEL)
    safe = ~_dilate(~free, int(math.ceil(margin_m / res)))
    pts, skipped = [], 0
    for _cid, ix, iy in zigzag(cells):                 # 칸 범위의 픽셀 창에서 갈 수 있는 점 중 칸 중심에 가장 가까운 점
        x0, y0 = ox + ix * g, oy + iy * g
        c0, c1 = max(0, int(math.floor((x0 - ox) / res))), min(raw.shape[1], int(math.ceil((x0 + g - ox) / res)))
        r0, r1 = max(0, h - int(math.ceil((y0 + g - oy) / res))), min(h, h - int(math.floor((y0 - oy) / res)))
        rr, cc = np.nonzero(safe[r0:r1, c0:c1])
        if rr.size == 0:
            skipped += 1
            continue
        px = ox + (cc + c0 + 0.5) * res
        py = oy + (h - (rr + r0) - 0.5) * res
        k = int(np.argmin((px - (x0 + g / 2)) ** 2 + (py - (y0 + g / 2)) ** 2))
        pts.append({"x": round(float(px[k]), 3), "y": round(float(py[k]), 3), "yaw": 0.0})
    for i in range(len(pts)):                          # 방향 = 다음 점 쪽 (마지막은 앞 점과 같음)
        j = i + 1 if i + 1 < len(pts) else i
        if j != i:
            pts[i]["yaw"] = round(math.atan2(pts[j]["y"] - pts[i]["y"], pts[j]["x"] - pts[i]["x"]), 4)
        elif i > 0:
            pts[i]["yaw"] = pts[i - 1]["yaw"]
    return pts, skipped


# ---------------------------------------------------------------- 제어 PC로 보내기 (동기 코드에서 불러도 됨)
SEND_LOCKS: dict[str, asyncio.Lock] = {}               # robot_id → 보내기 잠금 (send_agent 메시지 순서용)


async def _send_many(rid: str, msgs: list[dict[str, Any]]):
    """이벤트 루프에서 — 그 로봇의 현재 WS로 메시지들을 차례로 보냄. 연결이 없으면 버림(run_config · task는 재접속 때 agent_resync가 다시 보냄)"""
    ws = AGENTS.get(rid)
    if ws is None:
        return
    lock = SEND_LOCKS.setdefault(rid, asyncio.Lock())
    async with lock:                                   # 한 로봇으로 가는 메시지 순서 유지
        for m in msgs:
            try:
                await ws.send_json(m)
            except Exception as e:
                print(f"[agent] {rid} {m['type']} 보내기 실패: {e}", flush=True)
                return


def send_agent(rid: str, *msgs: tuple[str, dict[str, Any]]):
    """(type, body)들을 envelope로 감싸 그 로봇 제어 PC에 WS로 보냄 — 기다리지 않음(예약만).
    스레드 풀(동기 코드)에서도 · 이벤트 루프 안에서도 부를 수 있음. 연결이 없으면 아무것도 안 함"""
    if MAIN_LOOP is None or rid not in AGENTS:
        return
    envs = [envelope(t, b) for t, b in msgs]
    try:
        asyncio.get_running_loop()
        asyncio.ensure_future(_send_many(rid, envs))      # 이벤트 루프 안에서 부른 경우
    except RuntimeError:
        asyncio.run_coroutine_threadsafe(_send_many(rid, envs), MAIN_LOOP)


# ---------------------------------------------------------------- run · task 조회 도우미
def running_run(conn) -> dict[str, Any] | None:
    """진행 중(status 'running') run 행 하나 — 한 번에 하나만 있다고 봄"""
    return conn.execute("SELECT * FROM runs WHERE status = 'running'").fetchone()


def run_config(conn, run: dict[str, Any], rid: str) -> dict[str, Any]:
    """WS run_config 본문 — 그 로봇이 판단에 쓰는 값: run_id · 지도 · 격자 · 자기 섹터 · 섹터 칸 [cell_id, ix, iy] ·
    측정 대상 SSID(ssids · target_ap) · 섹터 안 활성 설비 · 기준값(RUN_SETTINGS) · 도크 접근 자세. 읽기만"""
    m = conn.execute("SELECT * FROM maps WHERE map_id = %s", (run["map_id"],)).fetchone()
    sectors = [int(s) for s, r in run["sector_robots"].items() if r == rid]
    cells = [[c["cell_id"], c["ix"], c["iy"]] for c in conn.execute(
        "SELECT cell_id, ix, iy FROM grid_cells WHERE map_id = %s AND sector = ANY(%s) ORDER BY cell_id",
        (run["map_id"], sectors))]
    st = settings_values(conn)
    target = conn.execute("SELECT ssid, x, y FROM aps WHERE map_id = %s AND is_target", (m["map_id"],)).fetchall()
    grid = grid_of(conn, m["map_id"])
    facs = []                                          # 그 로봇 섹터의 설비 (client 지도에 순찰 동안 표시)
    for f in conn.execute("SELECT facility_id, facility_type, expected_x, expected_y, view_pose FROM facilities "
                          "WHERE map_id = %s AND is_active ORDER BY facility_id", (m["map_id"],)):
        v = f["view_pose"] or {}
        c = cell_at(grid, float(v.get("x", 0)), float(v.get("y", 0)))
        if c and c[1] in sectors:
            facs.append(f)
    return {"run_id": run["run_id"], "map": {"map_id": m["map_id"], "name": m["name"], "robot_map_path": m["robot_map_path"]},
            "grid": {"origin_x": m["origin_x"], "origin_y": m["origin_y"], "size": m["grid_size_m"]},
            "sectors": sectors, "sector_cells": cells,
            "ssids": [a["ssid"] for a in target],              # 측정 AP 하나 (3-3차) — 이 SSID 표본만
            "target_ap": target[0] if target else None,
            "facilities": facs,
            "settings": {k: st.get(k) for k in RUN_SETTINGS},
            "dock_pose": (m["dock_poses"] or {}).get(rid)}


def task_message(conn, t: dict[str, Any]) -> dict[str, Any]:
    """WS task_assign 본문 {run_id, task_id, type, target_ref, waypoints, map_path} — 설비 점검은 facility, 정밀측정은 candidate를 함께"""
    run = conn.execute("SELECT r.map_id, m.robot_map_path FROM runs r JOIN maps m USING (map_id) WHERE r.run_id = %s",
                       (t["run_id"],)).fetchone()
    body = {"run_id": t["run_id"], "task_id": t["task_id"], "type": t["type"], "target_ref": t["target_ref"],
            "waypoints": t["waypoints"], "map_path": run["robot_map_path"]}
    if t["type"] == "FACILITY_CHECK":
        f = conn.execute("SELECT facility_id, facility_type, has_gauge, expected_x, expected_y, view_pose FROM facilities "
                         "WHERE map_id = %s AND facility_id = %s", (run["map_id"], t["target_ref"])).fetchone()
        body["facility"] = f
    elif t["type"] == "REINSPECTION":
        c = conn.execute("SELECT candidate_id, source, kind, ref, target_x, target_y, score FROM candidates "
                         "WHERE candidate_id = %s", (t["target_ref"],)).fetchone()
        body["candidate"] = c
    return body


def new_task(conn, run_id: int, rid: str, type_: str, target_ref: str | None, waypoints: list[dict],
             reason: str | None = None) -> str:
    """tasks INSERT(status PENDING) → task_id "r{run}-t{번호 3자리}" (번호 = 그 run의 task 수 + 1, IM_LOCK 안에서 부름)"""
    n = conn.execute("SELECT count(*) AS n FROM tasks WHERE run_id = %s", (run_id,)).fetchone()["n"]
    tid = f"r{run_id}-t{n + 1:03d}"
    conn.execute("INSERT INTO tasks (task_id, run_id, robot_id, type, target_ref, waypoints, status, reason) "
                 "VALUES (%s, %s, %s, %s, %s, %s, 'PENDING', %s)",
                 (tid, run_id, rid, type_, target_ref, Jsonb(waypoints), reason))
    return tid


def robot_sectors(run: dict[str, Any], rid: str) -> list[int]:
    """runs.sector_robots {"섹터": robot_id}에서 그 로봇의 섹터 번호들"""
    return [int(s) for s, r in run["sector_robots"].items() if r == rid]


def create_phase(conn, run: dict[str, Any], rid: str, phase: str) -> int:
    """그 로봇의 한 단계 task를 tasks에 만듦(PENDING) → 만든 개수 (0이면 advance가 다음 단계로)"""
    m = conn.execute("SELECT * FROM maps WHERE map_id = %s", (run["map_id"],)).fetchone()
    sectors = robot_sectors(run, rid)
    if phase == "SURVEY_SEGMENT":                      # 섹터 칸 지그재그 경로 → segment_points(20)점씩 끊어 task 여러 개
        cells = [(c["cell_id"], c["ix"], c["iy"]) for c in conn.execute(
            "SELECT cell_id, ix, iy FROM grid_cells WHERE map_id = %s AND sector = ANY(%s)", (m["map_id"], sectors))]
        st = settings_values(conn)
        pts, skipped = plan_survey(m, cells, fset(st, "wall_margin_m", 0.3))
        seg = max(1, int(fset(st, "segment_points", 20)))
        for k in range(0, len(pts), seg):              # target_ref "seg:a-b" = 섹터 경로의 경로점 번호 범위, 건너뛴 칸 수는 첫 task reason에
            part = pts[k:k + seg]
            new_task(conn, run["run_id"], rid, phase, f"seg:{k}-{k + len(part) - 1}", part,
                     f"SKIPPED_CELLS:{skipped}" if k == 0 and skipped else None)
        return math.ceil(len(pts) / seg)
    if phase == "REINSPECTION":                        # 그 로봇이 낸 열린 후보마다 — 같은 로봇이 정밀측정 (후보 → REINSPECTING)
        cands = conn.execute("SELECT candidate_id, target_x, target_y FROM candidates WHERE run_id = %s AND detected_by = %s "
                             "AND status = 'OPEN' ORDER BY candidate_id", (run["run_id"], rid)).fetchall()
        for c in cands:
            tid = new_task(conn, run["run_id"], rid, phase, c["candidate_id"],
                           [{"x": c["target_x"], "y": c["target_y"], "yaw": 0.0}])
            conn.execute("UPDATE candidates SET reinspect_task_id = %s, status = 'REINSPECTING' WHERE candidate_id = %s",
                         (tid, c["candidate_id"]))
        return len(cands)
    if phase == "FACILITY_CHECK":                      # 섹터 안 설비(관측 자세가 속한 칸 기준)마다 판정 1회
        grid = grid_of(conn, m["map_id"])
        # 설비 순서 = 관측 자세가 속한 칸의 지그재그 순서 (지도 전체 격자 기준)
        order = {c[0]: i for i, c in enumerate(zigzag([(cid, ix, iy) for (ix, iy), (cid, _s) in grid["cells"].items()]))}
        fac = []
        for f in conn.execute("SELECT facility_id, view_pose FROM facilities WHERE map_id = %s AND is_active", (m["map_id"],)):
            v = f["view_pose"] or {}
            c = cell_at(grid, float(v.get("x", 0)), float(v.get("y", 0)))
            if c and c[1] in sectors:
                fac.append((order.get(c[0], 0), f["facility_id"], v))
        for _o, fid, v in sorted(fac):
            new_task(conn, run["run_id"], rid, phase, fid, [{"x": v["x"], "y": v["y"], "yaw": v.get("yaw", 0.0)}])
        return len(fac)
    if phase == "DOCK":                                # 복귀 — 경로점 = 그 로봇 도크 접근 자세 하나 (없으면 빈 목록)
        dock = (m["dock_poses"] or {}).get(rid)
        new_task(conn, run["run_id"], rid, phase, None, [dock] if dock else [])
        return 1
    return 0


def advance(conn, run: dict[str, Any], rid: str, after: str | None):
    """after 단계 다음부터 task가 생기는 단계까지 만듦 (예: 후보가 없으면 정밀측정을 건너뛰고 설비 점검)
    → 만든 단계 이름, DOCK 뒤면 None. after=None이면 처음(SURVEY_SEGMENT)부터"""
    start = 0 if after is None else PHASES.index(after) + 1
    for ph in PHASES[start:]:
        if create_phase(conn, run, rid, ph):
            return ph
    return None


def assign_next(conn, run_id: int, rid: str) -> dict[str, Any] | None:
    """그 로봇에 ACTIVE가 없으면 다음 PENDING(task_id 순)을 ACTIVE로(assigned_at) → 보낼 task_assign 본문.
    이미 ACTIVE가 있거나 PENDING이 없으면 None. 보내기는 호출한 쪽(커밋 뒤 send_agent)"""
    if conn.execute("SELECT 1 FROM tasks WHERE run_id = %s AND robot_id = %s AND status = 'ACTIVE'", (run_id, rid)).fetchone():
        return None
    t = conn.execute("SELECT * FROM tasks WHERE run_id = %s AND robot_id = %s AND status = 'PENDING' "
                     "ORDER BY task_id LIMIT 1", (run_id, rid)).fetchone()
    if not t:
        return None
    conn.execute("UPDATE tasks SET status = 'ACTIVE', assigned_at = now() WHERE task_id = %s", (t["task_id"],))
    return task_message(conn, t)


def close_if_done(conn, run_id: int) -> str | None:
    """남은 task(PENDING · ACTIVE)가 없으면 scv_close_run → done · partial · aborted (남아 있으면 None).
    닫히면 CURRENT_RUN 비움 · 배정 로봇 제어 PC에 WS run_end · done이면 mark_baseline.
    호출한 쪽 트랜잭션 안에서 불림 — run_end는 커밋 전에 예약됨"""
    st = conn.execute("SELECT scv_close_run(%s) AS s", (run_id,)).fetchone()["s"]
    if st:
        CURRENT_RUN["run_id"] = None
        sr = conn.execute("SELECT sector_robots FROM runs WHERE run_id = %s", (run_id,)).fetchone()["sector_robots"]
        for rid in dict.fromkeys(sr.values()):         # 제어 PC — 순찰 표시(지도 · 정보)를 끝냄
            send_agent(rid, ("run_end", {"run_id": run_id, "status": st}))
        ok = mark_baseline(conn, run_id) if st == "done" else False
        print(f"[run] run {run_id} 종료 — {st}" + (" · 기준선 재료" if ok else ""), flush=True)
    return st


def mark_baseline(conn, run_id: int) -> bool:
    """기준선 재료 회차 표시 (runs.is_baseline_ok) — done · 정기 순찰(patrol) · RSSI 확정 이상 없음이면 true.
    partial · aborted는 scv_close_run · scv_abort_run이 false로 둠. 기준선 계산은 보고서 차수(3-5)"""
    return bool(conn.execute(
        "UPDATE runs r SET is_baseline_ok = true WHERE r.run_id = %s AND r.status = 'done' AND r.run_type = 'patrol' "
        "AND NOT EXISTS (SELECT 1 FROM findings f JOIN candidates c USING (candidate_id) "
        "WHERE c.run_id = r.run_id AND c.source = 'NETWORK') RETURNING r.run_id", (run_id,)).fetchone())


# ---------------------------------------------------------------- 순찰 시작 · 중단
def robot_ready(rid: str) -> tuple[bool, bool]:
    """(정상, 도킹) — 정상 = 제어 PC 연결 + status를 받고 있음, 도킹 = 마지막 status의 is_docked"""
    with MEM_LOCK:
        st = ROBOT_STATE.get(rid)
        f = dict(st["fields"]) if st else {}
    ok = (rid in AGENTS or RUN_OWNER == "ros") and bool(f.get("ts"))
    return ok, f.get("is_docked") == "true"


def start_form(conn) -> dict[str, Any]:
    """순찰 시작 조건 (3-3차) — 현재 지도 · 격자 · 활성 측정 AP, 로봇마다 정상 · 도킹 · 도크 접근 자세.
    → {map, sectors, robots[{robot_id, ready, why}], target_ap, problems, can_start, default(섹터→기본 로봇, 준비된 것만), run_types, running}"""
    m = conn.execute("SELECT map_id, name, dock_poses FROM maps WHERE is_active").fetchone()
    robots = conn.execute("SELECT robot_id FROM robots ORDER BY robot_id").fetchall()
    sectors = [] if not m else [{"sector": r["sector"], "cells": r["n"]} for r in conn.execute(
        "SELECT sector, count(*) AS n FROM grid_cells WHERE map_id = %s GROUP BY sector ORDER BY sector", (m["map_id"],))]
    target = m and conn.execute("SELECT ap_id, ssid, is_active FROM aps WHERE map_id = %s AND is_target", (m["map_id"],)).fetchone()
    problems = []                                      # 순찰 전체를 막는 것
    if not m:
        problems.append("현재 지도 없음")
    elif not sectors:
        problems.append("현재 지도에 격자 없음")
    if m and not target:
        problems.append("측정 AP 없음 (정보 › 측정 대상 AP에서 지정)")
    elif target and not target["is_active"] and RUN_OWNER == "web":   # ros: 표본은 순찰 중에만 와서 시작 전엔 늘 비활성
        problems.append(f"측정 AP {target['ssid']} 비활성 (client가 그 SSID 값을 받지 못함)")
    out = []
    for r in robots:
        rid = r["robot_id"]
        ok, docked = robot_ready(rid)
        why = []
        if not ok:
            why.append("비정상 (제어 PC 연결 · 상태 없음)")
        elif not docked and RUN_OWNER == "web":   # ros: mission_executor 가 켜지며 undock 하고 IDLE 로 기다림, 판단은 manager
            why.append("도킹 아님")
        if not (m and (m["dock_poses"] or {}).get(rid)):
            why.append("도크 접근 자세 없음")
        out.append({"robot_id": rid, "ready": not why and not problems, "why": why})
    ready = [x["robot_id"] for x in out if x["ready"]]
    default = {str(sc["sector"]): (DEFAULT_SECTOR_ROBOTS.get(str(sc["sector"])) if DEFAULT_SECTOR_ROBOTS.get(str(sc["sector"])) in ready
                                   else None) for sc in sectors}
    run = running_run(conn)
    return {"map": m and {"map_id": m["map_id"], "name": m["name"]}, "sectors": sectors, "robots": out,
            "target_ap": target and {"ssid": target["ssid"], "active": target["is_active"]}, "problems": problems,
            "can_start": bool(ready) and not run, "default": default, "run_types": RUN_TYPES, "running": run and run["run_id"]}


def start_run(user: dict[str, Any], body: dict[str, Any]) -> dict[str, Any]:
    """순찰 시작 — body.sector_robots {"섹터": robot_id} 확인(준비된 로봇 · 로봇 1대 = 섹터 1개) → runs INSERT(running) ·
    로봇마다 첫 단계 task 생성 · 첫 task ACTIVE (한 트랜잭션, IM_LOCK). 커밋 뒤 CURRENT_RUN 설정 ·
    제어 PC마다 WS run_config + task_assign"""
    req = {str(k): (str(v) if v else None) for k, v in (body.get("sector_robots") or {}).items()}
    run_type = str(body.get("run_type") or "patrol")
    if run_type not in RUN_TYPES:
        raise HTTPException(400, "순찰 종류는 순찰(patrol) 하나입니다")
    sends: list[tuple[str, dict, dict | None]] = []
    with IM_LOCK, pool.connection() as conn:
        with conn.transaction():
            r = running_run(conn)
            if r:
                raise HTTPException(409, f"순찰(run {r['run_id']})이 이미 진행 중입니다")
            form = start_form(conn)
            if not form["map"]:
                raise HTTPException(400, "현재 지도가 없습니다")
            if form["problems"]:
                raise HTTPException(400, "순찰을 시작할 수 없습니다 — " + " · ".join(form["problems"]))
            sec_ok = {str(s["sector"]) for s in form["sectors"] if s["cells"]}
            robots = {x["robot_id"]: x for x in form["robots"]}
            sr = {}
            for s, rid in req.items():                 # 배정 확인 — 빈 섹터는 건너뜀
                if not rid:
                    continue
                if s not in sec_ok:
                    raise HTTPException(400, f"섹터 {s}에 격자 칸이 없습니다")
                if rid not in robots:
                    raise HTTPException(400, f"{rid}는 등록된 로봇이 아닙니다")
                if not robots[rid]["ready"]:
                    raise HTTPException(400, f"{rid}로 순찰을 시작할 수 없습니다 — " + " · ".join(robots[rid]["why"]))
                if rid in sr.values():
                    raise HTTPException(400, f"{rid}에 섹터가 둘 배정되었습니다 — 로봇 1대 = 섹터 1개")
                sr[s] = rid
            if not sr:
                raise HTTPException(400, "활성 로봇을 배정한 섹터가 없습니다")
            run = conn.execute("INSERT INTO runs (map_id, run_type, sector_robots, status, started_by, started_at) "
                               "VALUES (%s, %s, %s, 'running', %s, now()) RETURNING *",
                               (form["map"]["map_id"], run_type, Jsonb(sr), user["user_id"])).fetchone()
            for rid in dict.fromkeys(sr.values()):     # 로봇마다 첫 단계 task 생성 · 보낼 run_config · 첫 task 준비
                advance(conn, run, rid, None)
                cfg = run_config(conn, run, rid)
                sends.append((rid, cfg, assign_next(conn, run["run_id"], rid)))
    CURRENT_RUN["run_id"] = run["run_id"]
    for rid, cfg, t in sends:
        send_agent(rid, ("run_config", cfg), *([("task_assign", t)] if t else []))
    print(f"[run] run {run['run_id']} 시작 — {sr}", flush=True)
    return {"ok": True, "run_id": run["run_id"]}


def abort_run(run_id: int, reason: str) -> dict[str, Any]:
    """순찰 중단 — scv_abort_run(남은 task 모두 CANCELED · run aborted). 커밋 뒤 CURRENT_RUN 비움 ·
    배정 로봇 제어 PC에 WS run_cancel(go_dock true)"""
    with IM_LOCK, pool.connection() as conn:
        with conn.transaction():
            run = conn.execute("SELECT * FROM runs WHERE run_id = %s", (run_id,)).fetchone()
            if not run or run["status"] != "running":
                raise HTTPException(409, "진행 중인 순찰이 아닙니다")
            n = conn.execute("SELECT scv_abort_run(%s, %s) AS n", (run_id, reason)).fetchone()["n"]
    CURRENT_RUN["run_id"] = None
    for rid in dict.fromkeys(run["sector_robots"].values()):   # 제어 PC는 하던 task를 멈추고 도크로 복귀
        send_agent(rid, ("run_cancel", {"run_id": run_id, "reason": reason, "go_dock": True}))
    print(f"[run] run {run_id} 중단 — task {n}개 취소 ({reason})", flush=True)
    return {"ok": True, "canceled": n}


# ---------------------------------------------------------------- task 진행 · 결과
def on_task_progress(rid: str, body: dict[str, Any]):
    """WS task_progress {task_id, progress 0~1} → tasks.progress (그 로봇의 ACTIVE task만)"""
    with pool.connection() as conn:
        conn.execute("UPDATE tasks SET progress = %s WHERE task_id = %s AND robot_id = %s AND status = 'ACTIVE'",
                     (max(0.0, min(1.0, float(body.get("progress") or 0))), body.get("task_id"), rid))


def on_task_result(rid: str, task_id: str, ok: bool, reason: str | None) -> dict[str, Any]:
    """task 결과 (REST /api/agent/tasks/{id}/result 또는 WS task_rejected). 그 로봇의 ACTIVE task가 아니면 무시.
    성공: DONE → 그 로봇 PENDING이 없으면 다음 단계 생성 / 실패: scv_fail_task → (DOCK이 아니면) 복귀 DOCK 생성.
    → 다음 task ACTIVE(커밋 뒤 WS task_assign), 없으면 close_if_done. IM_LOCK · 한 트랜잭션"""
    nxt, closed = None, None
    with IM_LOCK, pool.connection() as conn:
        with conn.transaction():
            t = conn.execute("SELECT t.*, r.status AS run_status FROM tasks t JOIN runs r USING (run_id) "
                             "WHERE t.task_id = %s FOR UPDATE OF t", (task_id,)).fetchone()
            if not t or t["robot_id"] != rid or t["status"] != "ACTIVE":
                return {"ok": True, "ignored": "ACTIVE task가 아님 (이미 끝났거나 취소됨)"}
            run = conn.execute("SELECT * FROM runs WHERE run_id = %s", (t["run_id"],)).fetchone()
            if ok:                                     # 성공 — 이 단계 task가 모두 끝났으면 다음 단계 생성
                conn.execute("UPDATE tasks SET status = 'DONE', result_ok = true, progress = 1, ended_at = now(), "
                             "reason = COALESCE(%s, reason) WHERE task_id = %s", (reason, task_id))
                if not conn.execute("SELECT 1 FROM tasks WHERE run_id = %s AND robot_id = %s AND status = 'PENDING'",
                                    (t["run_id"], rid)).fetchone():
                    advance(conn, run, rid, t["type"])
            else:
                conn.execute("SELECT scv_fail_task(%s, %s)", (task_id, (reason or "FAILED")[:200]))
                if t["type"] != "DOCK":                # 실패한 로봇은 복귀 (그 섹터만 중단)
                    create_phase(conn, run, rid, "DOCK")
            nxt = assign_next(conn, t["run_id"], rid)
            if nxt is None:                            # 이 로봇은 끝 — 다른 로봇도 끝났으면 run 종료
                closed = close_if_done(conn, t["run_id"])
    if nxt:
        send_agent(rid, ("task_assign", nxt))
    print(f"[task] {rid} {task_id} {'DONE' if ok else 'FAILED ' + str(reason)}"
          + (f" → 다음 {nxt['task_id']} {nxt['type']}" if nxt else "") + (f" · run {closed}" if closed else ""), flush=True)
    return {"ok": True, "next": nxt and nxt["task_id"], "run_status": closed}


def on_robot_lost(rid: str):
    """E3 — 순찰 중이면 그 로봇의 task 실패(NO_RESPONSE, ACTIVE 우선 · 없으면 첫 PENDING) → scv_fail_task로 남은 task 취소
    → 그 섹터만 중단, close_if_done. 응답이 없으므로 복귀 task는 만들지 않음 (task_cancel도 없음 — run이 닫히면 close_if_done이 run_end만)"""
    with IM_LOCK, pool.connection() as conn:
        with conn.transaction():
            run = running_run(conn)
            if not run:
                return
            t = conn.execute("SELECT task_id FROM tasks WHERE run_id = %s AND robot_id = %s AND status IN ('ACTIVE', 'PENDING') "
                             "ORDER BY (status = 'ACTIVE') DESC, task_id LIMIT 1", (run["run_id"], rid)).fetchone()
            if not t:
                return
            conn.execute("SELECT scv_fail_task(%s, 'NO_RESPONSE')", (t["task_id"],))
            closed = close_if_done(conn, run["run_id"])
    print(f"[task] {rid} 응답 없음 → {t['task_id']} FAILED(NO_RESPONSE)" + (f" · run {closed}" if closed else ""), flush=True)


def agent_resync(rid: str, hello: dict[str, Any]) -> list[tuple[str, dict[str, Any]]]:
    """제어 PC가 다시 연결했을 때 보낼 WS 메시지 목록 [(type, body)] — ws_agent가 welcome · config 뒤에 보냄.
    그 로봇이 진행 중 run에 배정됐으면 run_config, DB의 ACTIVE task가 hello.active_task_id와 다르면 task_assign 다시.
    제어 PC가 하던 task가 DB에서 ACTIVE가 아니면 맨 앞에 task_cancel(NOT_ACTIVE, DB ACTIVE가 없으면 go_dock)"""
    out = []
    with pool.connection() as conn:
        run = running_run(conn)
        db_active = None
        if run and rid in run["sector_robots"].values():
            out.append(("run_config", run_config(conn, run, rid)))
            t = conn.execute("SELECT * FROM tasks WHERE run_id = %s AND robot_id = %s AND status = 'ACTIVE'",
                             (run["run_id"], rid)).fetchone()
            db_active = t and t["task_id"]
            if t and hello.get("active_task_id") != t["task_id"]:
                out.append(("task_assign", task_message(conn, t)))
        mine = hello.get("active_task_id")
        if mine and mine != db_active:
            out.insert(0, ("task_cancel", {"task_id": mine, "reason": "NOT_ACTIVE", "go_dock": db_active is None}))
    return out


def run_of_robot(conn, rid: str) -> tuple[dict | None, str | None]:
    """그 로봇이 배정된 진행 중 run과 그 로봇의 ACTIVE task — 배정되지 않았으면 (None, None)"""
    run = running_run(conn)
    if not run or rid not in run["sector_robots"].values():
        return None, None
    t = conn.execute("SELECT task_id FROM tasks WHERE run_id = %s AND robot_id = %s AND status = 'ACTIVE'",
                     (run["run_id"], rid)).fetchone()
    return run, t and t["task_id"]


# ---------------------------------------------------------------- 제어 PC 기록 (REST — 아웃박스에서 옴)
def add_event(conn, rid: str, ev: dict[str, Any]) -> dict[str, Any] | None:
    """robot_events INSERT (event_uid로 중복 무시) → 새로 들어간 이벤트(+ RECENT_EVENTS에 추가), 중복이면 None.
    커밋 전에 메모리에 들어감 — 호출한 쪽 트랜잭션이 롤백돼도 RECENT_EVENTS에는 남음"""
    row = conn.execute(
        "INSERT INTO robot_events (event_uid, robot_id, run_id, task_id, code, source, level, detail, ts) "
        "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s) ON CONFLICT (event_uid) DO NOTHING "
        "RETURNING event_id, event_uid, robot_id, run_id, task_id, code, source, level, detail, ts",
        (ev["event_uid"], rid, ev.get("run_id"), ev.get("task_id"), str(ev["code"])[:16], ev.get("source", "robot"),
         ev.get("level", "info"), Jsonb(ev.get("detail")) if ev.get("detail") is not None else None,
         ev.get("ts") or datetime.now(timezone.utc))).fetchone()
    if row:
        remember_event(row)
    return row


def remember_event(row: dict[str, Any]):
    """robot_events 행 → RECENT_EVENTS(최근 1000개, ts는 ISO 문자열로)"""
    e = dict(row)
    e["ts"] = e["ts"].isoformat() if hasattr(e["ts"], "isoformat") else e["ts"]
    with MEM_LOCK:
        RECENT_EVENTS.append(e)


def on_events(rid: str, events: list[dict[str, Any]]) -> dict[str, Any]:
    """REST /api/agent/events — 이벤트마다 한 트랜잭션: robot_events INSERT → 새 이벤트면 handle_event(candidates · findings).
    DB 규칙 위반은 rejected로 돌려주고 200 (아웃박스가 다시 보내지 않게) → {ok, stored, rejected}"""
    done, rejected = 0, []
    for ev in events:
        try:
            with IM_LOCK, pool.connection() as conn:
                with conn.transaction():
                    row = add_event(conn, rid, ev)
                    if row:
                        handle_event(conn, rid, row, ev.get("detail") or {})
            done += 1
        except psycopg.Error as e:                     # 규칙 위반 등 — 다시 보내도 같으므로 받은 것으로 하고 넘김
            rejected.append({"event_uid": ev.get("event_uid"), "error": e.diag.message_primary})
            print(f"[event] {rid} {ev.get('code')} 거부: {e.diag.message_primary}", flush=True)
    return {"ok": True, "stored": done, "rejected": rejected}


def handle_event(conn, rid: str, ev: dict[str, Any], d: dict[str, Any]):
    """이상 알림 이벤트 → candidates · findings (inspection_manager). d = 이벤트 detail
    CANDIDATE: candidates INSERT(OPEN) / CONFIRMED · CLEARED · UNCONFIRMED: 후보 결론(CLOSED), CONFIRMED면 findings INSERT"""
    code = ev["code"]
    if code == "CANDIDATE":                            # detail {candidate_id, source, kind, ref, target [x, y], score} — 낸 로봇 · task 기록
        tgt = d.get("target") or [None, None]
        conn.execute(
            "INSERT INTO candidates (candidate_id, run_id, source, kind, ref, target_x, target_y, score, detect_task_id, "
            "detected_by, status, detected_at) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, 'OPEN', %s) "
            "ON CONFLICT DO NOTHING",
            (d["candidate_id"], ev["run_id"], d.get("source", "NETWORK"), d.get("kind", "NET_COVERAGE"), str(d["ref"]),
             tgt[0], tgt[1], d.get("score"), ev["task_id"], rid, ev["ts"]))
    elif code in ("CONFIRMED", "CLEARED", "UNCONFIRMED"):   # 결론은 한 번만 (outcome이 비었을 때)
        c = conn.execute("UPDATE candidates SET outcome = %s, decided_at = %s, status = 'CLOSED' "
                         "WHERE candidate_id = %s AND outcome IS NULL RETURNING candidate_id, source, reinspect_task_id",
                         (code, ev["ts"], d.get("candidate_id"))).fetchone()
        if c and code == "CONFIRMED":                  # 확정 이상 — category: NETWORK 후보 = coverage, 그 밖 = facility
            conn.execute("INSERT INTO findings (candidate_id, task_id, category, status, confirmed_at) "
                         "VALUES (%s, %s, %s, 'open', %s) ON CONFLICT (candidate_id) DO NOTHING",
                         (c["candidate_id"], c["reinspect_task_id"], "coverage" if c["source"] == "NETWORK" else "facility",
                          ev["ts"]))


def on_samples(rid: str, run_id: int, samples: list[dict[str, Any]]) -> dict[str, Any]:
    """REST /api/agent/samples — RSSI 표본 묶음 → net_samples INSERT(한 트랜잭션, (robot, run, seq …) 중복 무시).
    SSID → 그 run 지도의 ap_id(없으면 버림), pose_ok면 좌표 → cell_id. 새로 들어간 것은 LIVE_CELLS에도 누적
    → {ok, stored, dropped}"""
    with pool.connection() as conn:
        run = conn.execute("SELECT map_id FROM runs WHERE run_id = %s", (run_id,)).fetchone()
        if not run:
            return {"ok": True, "stored": 0, "dropped": len(samples), "reason": "없는 run"}
        aps = {a["ssid"]: a["ap_id"] for a in conn.execute("SELECT ap_id, ssid FROM aps WHERE map_id = %s", (run["map_id"],))}
        grid = grid_of(conn, run["map_id"])
        # 표본의 측정 task — 이 로봇 · run의 SURVEY_SEGMENT만 (DB 트리거와 같은 조건. 아니면 비워서 묶음 전체가 거부되지 않게)
        survey = {t["task_id"] for t in conn.execute("SELECT task_id FROM tasks WHERE run_id = %s AND robot_id = %s "
                                                     "AND type = 'SURVEY_SEGMENT'", (run_id, rid))}
        stored, dropped, inc = 0, 0, []
        with conn.transaction():
            for s in samples:
                ap = aps.get(s.get("ssid"))
                if ap is None:                         # 등록되지 않은 SSID — 저장하지 않음
                    dropped += 1
                    continue
                c = cell_at(grid, float(s["x"]), float(s["y"])) if s.get("pose_ok") else None
                tid = s.get("task_id") if s.get("task_id") in survey else None
                row = conn.execute(
                    "INSERT INTO net_samples (robot_id, run_id, seq, stamp, x, y, yaw, pose_ok, ap_id, rssi_dbm, cell_id, task_id, detected) "
                    "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s) ON CONFLICT DO NOTHING RETURNING seq",
                    (rid, run_id, int(s["seq"]), s["stamp"], s["x"], s["y"], s["yaw"], bool(s.get("pose_ok")), ap,
                     int(s["rssi_dbm"]) if s.get("detected", True) else None,
                     c and c[0], tid, bool(s.get("detected", True)))).fetchone()
                if row:
                    stored += 1
                    if c and s.get("detected", True):
                        inc.append((c[0], ap, float(s["rssi_dbm"])))
    with MEM_LOCK:                                     # 실시간 히트맵 누적 (이미 만든 run만 — 없으면 처음 볼 때 DB에서)
        live = LIVE_CELLS.get(run_id)
        if live is not None:
            for cid, ap, v in inc:
                a = live.setdefault(cid, {}).setdefault(ap, [0, 0.0, 0.0, v])
                a[0] += 1; a[1] += v; a[2] += v * v; a[3] = v
    return {"ok": True, "stored": stored, "dropped": dropped}


def live_cells(conn, run_id: int) -> dict[int, dict[int, list[float]]]:
    """그 run의 칸별 · AP별 누적 [n, sum, sumsq, last] — LIVE_CELLS에 없으면 net_samples에서 집계해 넣음 (히트맵용)"""
    with MEM_LOCK:
        live = LIVE_CELLS.get(run_id)
    if live is None:                                   # 처음이거나 server_page 재시작 — net_samples에서 다시 집계
        live = {}
        for r in conn.execute("SELECT cell_id, ap_id, count(*) AS n, sum(rssi_dbm) AS s, sum(rssi_dbm * rssi_dbm) AS q "
                              "FROM net_samples WHERE run_id = %s AND pose_ok AND detected AND cell_id IS NOT NULL "
                              "GROUP BY cell_id, ap_id", (run_id,)):
            live.setdefault(r["cell_id"], {})[r["ap_id"]] = [int(r["n"]), float(r["s"]), float(r["q"]), None]
        with MEM_LOCK:
            live = LIVE_CELLS.setdefault(run_id, live)
            for old in [k for k in LIVE_CELLS if k != run_id and k != CURRENT_RUN["run_id"]][:-3]:
                LIVE_CELLS.pop(old, None)              # 지난 run은 몇 개만 남김 (이 run · 진행 중 run 말고 나중에 넣은 3개)
    return live


def on_precise(rid: str, body: dict[str, Any]) -> dict[str, Any]:
    """REST /api/agent/precise-measures — {task_id, cell_id, measures[{ssid, duration_s, n_samples, rssi_median, rssi_iqr,
    measured_at}]} → precise_measures INSERT(AP마다, 중복 무시). 등록 안 된 SSID는 건너뜀 → {ok, stored}"""
    with pool.connection() as conn:
        t = conn.execute("SELECT t.run_id, r.map_id FROM tasks t JOIN runs r USING (run_id) WHERE t.task_id = %s",
                         (body.get("task_id"),)).fetchone()
        if not t:
            return {"ok": True, "stored": 0, "reason": "없는 task"}
        aps = {a["ssid"]: a["ap_id"] for a in conn.execute("SELECT ap_id, ssid FROM aps WHERE map_id = %s", (t["map_id"],))}
        n = 0
        with conn.transaction():
            for m_ in body.get("measures") or []:
                ap = aps.get(m_.get("ssid"))
                if ap is None:
                    continue
                n += conn.execute(
                    "INSERT INTO precise_measures (task_id, robot_id, ap_id, cell_id, duration_s, n_samples, rssi_median, "
                    "rssi_iqr, measured_at) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s) ON CONFLICT DO NOTHING",
                    (body["task_id"], rid, ap, int(body["cell_id"]), int(m_["duration_s"]), int(m_["n_samples"]),
                     float(m_["rssi_median"]), float(m_["rssi_iqr"]), m_["measured_at"])).rowcount
    return {"ok": True, "stored": n}


def on_inspection(rid: str, b: dict[str, Any]) -> dict[str, Any]:
    """REST /api/agent/inspections — 설비 판정 한 건 → inspections INSERT (run_id · map_id는 task에서, inspection_uid 중복 무시)
    → {ok, inspection_id(중복이면 None)}"""
    with pool.connection() as conn:
        t = conn.execute("SELECT t.run_id, r.map_id FROM tasks t JOIN runs r USING (run_id) WHERE t.task_id = %s",
                         (b.get("task_id"),)).fetchone()
        if not t:
            return {"ok": True, "stored": 0, "reason": "없는 task"}
        row = conn.execute(
            "INSERT INTO inspections (inspection_uid, task_id, run_id, robot_id, map_id, facility_id, detail, final, frontal, "
            "pose_retry, gauge_state, confidence, bbox, model_version, inspected_at, aruco) "
            "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s) ON CONFLICT DO NOTHING RETURNING inspection_id",
            (b["inspection_uid"], b["task_id"], t["run_id"], rid, t["map_id"], b["facility_id"], b["detail"], b["final"],
             bool(b["frontal"]), int(b.get("pose_retry", 0)), b.get("gauge_state"), b.get("confidence"),
             Jsonb(b["bbox"]) if b.get("bbox") is not None else None, str(b.get("model_version", "unknown"))[:32],
             b["inspected_at"], Jsonb(b["aruco"]) if b.get("aruco") else None)).fetchone()
    return {"ok": True, "inspection_id": row and row["inspection_id"]}


async def on_media(rid: str, meta: dict[str, Any], data: bytes) -> dict[str, Any]:
    """REST /api/agent/media — 판정 사진 한 장 → MEDIA_DIR/inspections/YYYY/MM/DD/r{run}/{설비}_{uid 8자}_{n}.jpg 저장 +
    media INSERT. 판정(inspection_uid)이 아직 없으면 409 (아웃박스가 다시 보냄)"""
    import hashlib

    def work():
        """스레드 풀에서 — 판정 찾기 · 파일 쓰기 · media 행 (object_key 중복 무시, 파일은 덮어씀)"""
        with pool.connection() as conn:
            ins = conn.execute("SELECT inspection_id, run_id, facility_id, inspected_at FROM inspections WHERE inspection_uid = %s",
                               (meta.get("inspection_uid"),)).fetchone()
            if not ins:
                return {"ok": False, "retry": True, "reason": "판정이 아직 없음"}
            d = ins["inspected_at"]
            key = (f"inspections/{d:%Y/%m/%d}/r{ins['run_id']}/{ins['facility_id']}_"
                   f"{str(meta['inspection_uid'])[:8]}_{int(meta.get('index', 1))}.jpg")
            path = MEDIA_DIR / key
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
            conn.execute("INSERT INTO media (inspection_id, robot_id, kind, object_key, sha256, size_bytes, captured_at) "
                         "VALUES (%s, %s, %s, %s, %s, %s, %s) ON CONFLICT (object_key) DO NOTHING",
                         (ins["inspection_id"], rid, meta.get("kind", "rgb"), key, hashlib.sha256(data).hexdigest(),
                          len(data), meta.get("captured_at") or datetime.now(timezone.utc)))
            return {"ok": True, "object_key": key}
    r = await run_in_threadpool(work)
    if not r["ok"]:
        raise HTTPException(409, r["reason"])          # 판정보다 사진이 먼저 오면 제어 PC가 다시 보냄
    return r


def mark_aps_seen(rid: str, ssids: list[str]):
    """status의 ssids — client가 최근에 받은 SSID → 현재 지도의 그 AP aps.is_active = true · AP_SEEN 갱신
    (값이 끊기면 1초 점검 expire_aps가 비활성)"""
    if not ssids:
        return
    with pool.connection() as conn:
        rows = conn.execute("SELECT a.ap_id, a.is_active FROM aps a JOIN maps m USING (map_id) WHERE m.is_active "
                            "AND a.ssid = ANY(%s)", (list(ssids),)).fetchall()
        off = [r["ap_id"] for r in rows if not r["is_active"]]
        if off:
            conn.execute("UPDATE aps SET is_active = true WHERE ap_id = ANY(%s)", (off,))
    now = time.monotonic()
    with MEM_LOCK:
        for r in rows:
            AP_SEEN[r["ap_id"]] = now


def expire_aps(limit: float):
    """limit초 동안 SSID 값을 못 받은 AP → AP_SEEN에서 빼고 aps.is_active = false (state_watchdog가 1초마다)"""
    now = time.monotonic()
    with MEM_LOCK:
        gone = [a for a, t in AP_SEEN.items() if now - t > limit]
        for a in gone:
            AP_SEEN.pop(a, None)
    if gone:
        with pool.connection() as conn:
            conn.execute("UPDATE aps SET is_active = false WHERE ap_id = ANY(%s)", (gone,))


def restore_memory():
    """server_page 시작 때 — CURRENT_RUN(runs) · RECENT_EVENTS(robot_events 최근 1000개) 복원,
    순찰 중 로봇(PENDING · ACTIVE task가 있는)은 ROBOT_STATE에 「방금 받은 것」으로 두어 status가 끝내 안 오면 E3가 나게.
    LIVE_CELLS는 처음 볼 때 live_cells가, GRID_CACHE는 grid_of가 다시 채움"""
    with pool.connection() as conn:
        run = running_run(conn)
        CURRENT_RUN["run_id"] = run and run["run_id"]
        for e in reversed(conn.execute("SELECT event_id, event_uid, robot_id, run_id, task_id, code, source, level, detail, ts "
                                       "FROM robot_events ORDER BY event_id DESC LIMIT 1000").fetchall()):
            remember_event(e)
        busy = [] if not run else [r["robot_id"] for r in conn.execute(
            "SELECT DISTINCT robot_id FROM tasks WHERE run_id = %s AND status IN ('PENDING', 'ACTIVE')", (run["run_id"],))]
    now = time.monotonic()
    with MEM_LOCK:
        for rid in busy:
            ROBOT_STATE.setdefault(rid, {"fields": {"note": "server_page 재시작 — status 기다림 (순찰 중 로봇)"}, "rx": now})
    if run:
        print(f"[run] 진행 중 run {run['run_id']} 복원 — 대기 로봇 {busy or '없음'}", flush=True)


# ---------------------------------------------------------------- REST — 제어 PC (scv_agent 아웃박스)
#   body에 robot_id를 함께 보냄 (로그인 확인 없음). 2xx면 아웃박스에서 지움, 409 · 5xx는 다시 보냄, 그 밖 4xx는 failed/
@app.post("/api/agent/tasks/{task_id}/result")
async def api_task_result(task_id: str, request: Request):
    """task 결과 {robot_id, ok, reason, ended_at(쓰지 않음)} → on_task_result (다음 task는 WS task_assign으로)"""
    b = await request.json()
    return await run_in_threadpool(on_task_result, str(b["robot_id"]), task_id, bool(b.get("ok")), b.get("reason"))


@app.get("/api/agent/last-seq")
def api_last_seq(robot_id: str, run_id: int):
    """그 로봇 · run의 net_samples 마지막 seq (없으면 0) — 제어 PC가 표본 번호를 이어 붙일 때"""
    with pool.connection() as conn:
        r = conn.execute("SELECT max(seq) AS s FROM net_samples WHERE robot_id = %s AND run_id = %s", (robot_id, run_id)).fetchone()
    return {"last_seq": r["s"] or 0}


@app.post("/api/agent/samples")
async def api_samples(request: Request):
    """RSSI 표본 묶음 {robot_id, run_id, samples[{seq, stamp, x, y, yaw, pose_ok, ssid, rssi_dbm, task_id}]} → on_samples"""
    b = await request.json()
    return await run_in_threadpool(on_samples, str(b["robot_id"]), int(b["run_id"]), b.get("samples") or [])


@app.post("/api/agent/events")
async def api_events(request: Request):
    """이벤트 {robot_id, events[{event_uid, run_id, task_id, code, source, level, detail, ts}]} → on_events"""
    b = await request.json()
    return await run_in_threadpool(on_events, str(b["robot_id"]), b.get("events") or [])


@app.post("/api/agent/precise-measures")
async def api_precise(request: Request):
    """정밀측정 결과 {robot_id, task_id, cell_id, measures[…]} → on_precise"""
    b = await request.json()
    return await run_in_threadpool(on_precise, str(b["robot_id"]), b)


@app.post("/api/agent/inspections")
async def api_inspections(request: Request):
    """설비 판정 {robot_id, inspection_uid, task_id, facility_id, detail, final, frontal, …, inspected_at} → on_inspection"""
    b = await request.json()
    return await run_in_threadpool(on_inspection, str(b["robot_id"]), b)


@app.post("/api/agent/media")
async def api_media(meta: str = Form(...), file: UploadFile = File(...)):
    """판정 사진 — multipart: meta(JSON {robot_id, inspection_uid, index, kind, captured_at}) + file(jpg) → on_media"""
    m = json.loads(meta)
    return await on_media(str(m["robot_id"]), m, await file.read())


# ---------------------------------------------------------------- REST — server page 순찰 (모니터 시작 · 중단 / 이력 탭 · 「자세히」)
@app.get("/api/runs/form")
def api_run_form(request: Request):
    """순찰 시작 창 — start_form (시작 조건 · 로봇별 준비 · 기본 배정)"""
    current_user(request)
    with pool.connection() as conn:
        return start_form(conn)


@app.post("/api/runs")
async def api_run_start(request: Request):
    """순찰 시작 {sector_robots, run_type?} → start_run → {ok, run_id}"""
    user = current_user(request)
    body = await request.json()
    if RUN_OWNER == "ros":                             # 섹터 · 로봇은 inspection_manager(zones.yaml)가 정함
        return await run_in_threadpool(ros_call, "start")
    return await run_in_threadpool(start_run, user, body)


def ros_call(name: str) -> dict[str, Any]:
    """ros 모드 — /inspection/{name} 호출 → {ok, message, run_id}. 실패는 409 (화면에 문구)"""
    r = BRIDGE.call(name)
    if not r["ok"]:
        raise HTTPException(409, r["message"])
    return {**r, "run_id": CURRENT_RUN["run_id"]}


@app.post("/api/runs/resume")
def api_run_resume(request: Request):
    """순찰 재개 (ros 모드) — 비상정지 뒤 /inspection/resume"""
    current_user(request)
    if RUN_OWNER != "ros":
        raise HTTPException(409, "재개는 ROS 관제(SCV_RUN_OWNER=ros)에서만 씁니다")
    return ros_call("resume")


@app.post("/api/runs/{run_id}/stop")
def api_run_stop(run_id: int, request: Request):
    """순찰 종료 (ros 모드, 10/15) — /inspection/abort: 작업 취소 · 회차 aborted"""
    current_user(request)
    if RUN_OWNER != "ros":
        return abort_run(run_id, "ADMIN_STOP")
    return ros_call("abort")


@app.post("/api/runs/{run_id}/abort")
def api_run_abort(run_id: int, request: Request):
    """순찰 중단 (관리자) → abort_run(reason ADMIN_STOP) → {ok, canceled}"""
    current_user(request)
    if RUN_OWNER == "ros":                             # 비상정지 — 두 로봇 goal 취소 · 정지, 재개 대기
        return ros_call("estop")
    return abort_run(run_id, "ADMIN_STOP")


@app.get("/api/runs")
def api_runs(request: Request):
    """이력 탭 — 현재 지도에서 돈 순찰 (최근 50회) + 진행 중 run_id(CURRENT_RUN) · 현재 지도"""
    current_user(request)
    with pool.connection() as conn:
        m = conn.execute("SELECT map_id, name FROM maps WHERE is_active").fetchone()
        rows = [] if not m else conn.execute(
            "SELECT r.run_id, r.status, r.run_type, r.sector_robots, r.started_at, r.ended_at, r.is_baseline_ok, "
            "m.name AS map_name FROM runs r JOIN maps m USING (map_id) WHERE r.map_id = %s "
            "ORDER BY r.run_id DESC LIMIT 50", (m["map_id"],)).fetchall()
    return {"runs": rows, "current": CURRENT_RUN["run_id"], "map": m}


@app.get("/api/runs/{run_id}/events")
def api_run_events(run_id: int, request: Request):
    """이력 탭 — 그 run의 robot_events 전체 (event_id 순)"""
    current_user(request)
    with pool.connection() as conn:
        rows = conn.execute("SELECT event_id, event_uid, robot_id, run_id, task_id, code, source, level, detail, ts "
                            "FROM robot_events WHERE run_id = %s ORDER BY event_id", (run_id,)).fetchall()
    return {"events": rows}


@app.get("/api/runs/{run_id}/recording")
def api_run_recording(run_id: int, request: Request):
    """이력 탭 블랙박스 — 로봇별 [촬영 시각 ms, /media 경로] (ros_bridge 가 주행 중 1 Hz 로 저장한 프레임)"""
    current_user(request)
    base = MEDIA_DIR / "recordings" / f"r{run_id}"
    out = {}
    for d in sorted(base.iterdir()) if base.is_dir() else []:
        frames = sorted(int(f.stem) for f in d.glob("*.jpg") if f.stem.isdigit())
        out[d.name] = [[t, f"/media/recordings/r{run_id}/{d.name}/{t}.jpg"] for t in frames]
    return out


@app.get("/api/runs/{run_id}")
def api_run_detail(run_id: int, request: Request, lite: bool = False):
    """이력 탭 run 하나 — run · tasks · candidates(칸 · 정밀측정 최고 중앙값 · finding) · inspections(설비 · 사진 키) · aps ·
    heat(칸 → AP → [평균 RSSI, 표본 수], LIVE_CELLS) · 표본 수 · 의심 기준값(thr). 읽기만 (LIVE_CELLS는 없으면 채움)"""
    current_user(request)
    with pool.connection() as conn:
        run = conn.execute("SELECT r.*, m.name AS map_name, u.username AS started_by_name FROM runs r JOIN maps m USING (map_id) "
                           "LEFT JOIN users u ON u.user_id = r.started_by WHERE r.run_id = %s", (run_id,)).fetchone()
        if not run:
            raise HTTPException(404, "없는 순찰입니다")
        tasks = conn.execute("SELECT task_id, robot_id, type, target_ref, waypoints, status, progress, result_ok, reason, "
                             "assigned_at, ended_at FROM tasks WHERE run_id = %s ORDER BY task_id", (run_id,)).fetchall()
        cands = conn.execute(
            "SELECT c.*, g.ix, g.iy, (SELECT max(p.rssi_median) FROM precise_measures p WHERE p.task_id = c.reinspect_task_id) "
            "AS precise_best, f.finding_id FROM candidates c LEFT JOIN grid_cells g ON g.cell_id::text = c.ref "
            "LEFT JOIN findings f USING (candidate_id) WHERE c.run_id = %s ORDER BY c.candidate_id", (run_id,)).fetchall()
        ins = conn.execute(
            "SELECT i.inspection_id, i.task_id, i.robot_id, i.facility_id, i.detail, i.final, i.frontal, i.pose_retry, "
            "i.gauge_state, i.confidence, i.inspected_at, i.aruco, f.expected_x, f.expected_y, f.facility_type, "
            "(SELECT array_agg(md.object_key ORDER BY md.media_id) FROM media md WHERE md.inspection_id = i.inspection_id) AS photos "
            "FROM inspections i JOIN facilities f ON f.map_id = i.map_id AND f.facility_id = i.facility_id "
            "WHERE i.run_id = %s ORDER BY i.inspection_id", (run_id,)).fetchall()
        aps = conn.execute("SELECT ap_id, ssid FROM aps WHERE map_id = %s ORDER BY ssid", (run["map_id"],)).fetchall()
        live = live_cells(conn, run_id)
        n_samples = conn.execute("SELECT count(*) AS n FROM net_samples WHERE run_id = %s", (run_id,)).fetchone()["n"]
        thr = fset(settings_values(conn), "rssi_suspect_dbm", -72)
        # 10/15 UI 시안: 방문 궤적(robot_status_log 1 Hz) · NetSample 점 (lite=1 이면 표본 점은 뺌 — 모니터 2초 갱신용)
        tracks: dict[str, list] = {}
        # 도크 위 · undock 중(DOCKED) · 도킹 중(DOCKING) 위치는 빼서 출발한 순간부터 그림 (끊긴 곳은 화면이 잇지 않음)
        for r in conn.execute("SELECT robot_id, extract(epoch FROM ts) AS t, x, y FROM robot_status_log "
                              "WHERE run_id = %s AND x IS NOT NULL AND coalesce(nav_state, '') NOT IN ('DOCKED', 'DOCKING') "
                              "ORDER BY ts", (run_id,)):
            tracks.setdefault(r["robot_id"], []).append([round(float(r["t"]), 1), round(r["x"], 3), round(r["y"], 3)])
        points = [] if lite else conn.execute(
            "SELECT robot_id, seq, stamp, x, y, pose_ok, ap_id, detected, rssi_dbm, cell_id, task_id FROM net_samples "
            "WHERE run_id = %s ORDER BY stamp LIMIT 5000", (run_id,)).fetchall()
    with MEM_LOCK:                                     # 히트맵 = 칸 · AP별 평균(sum / n)
        heat = {cid: {ap: [round(v[1] / v[0], 1), int(v[0])] for ap, v in d.items() if v[0]} for cid, d in live.items()}
    return {"run": run, "tasks": tasks, "candidates": cands, "inspections": ins, "aps": aps, "heat": heat,
            "samples": n_samples, "thr": thr, "current": CURRENT_RUN["run_id"], "tracks": tracks, "points": points}


def cell_key(grid: dict[str, Any], x: float, y: float) -> dict[str, Any] | None:
    """좌표 → {cell_id, ix, iy} (격자 밖이면 None)"""
    ix, iy = math.floor((x - grid["ox"]) / grid["g"]), math.floor((y - grid["oy"]) / grid["g"])
    c = grid["cells"].get((ix, iy))
    return {"cell_id": c[0], "ix": ix, "iy": iy} if c else None


def cell_visits(points) -> list[dict[str, Any]]:
    """시각순 (시각, 칸) → 칸이 바뀔 때마다 끊은 실제 순서 [{cell_id, ix, iy, enter, leave, n}]. 다시 지나면 따로 남음"""
    out: list[dict[str, Any]] = []
    for ts, c in points:
        if c is None:
            continue
        if out and out[-1]["cell_id"] == c["cell_id"]:
            out[-1]["leave"], out[-1]["n"] = ts, out[-1]["n"] + 1
        else:
            out.append({**c, "enter": ts, "leave": ts, "n": 1})
    return out


@app.get("/api/tasks/{task_id}")
def api_task_detail(task_id: str, request: Request):
    """이력 탭 task 「자세히」 — 측정: 거친 칸 · 칸별 측정값 · 이 구간 후보 / 정밀측정: 후보 · 정밀측정 · 결론 /
    탐지: 설비 · 판정 · 사진 / 공통: 경로점 · 실제로 지나간 길 · 배터리(robot_status_log) · 이 task의 이벤트
    칸 순서: plan_cells = 경로점마다 칸(계획 순서), visits = 실제로 지난 칸 순서(측정은 표본, 그 밖은 status 1 Hz)"""
    current_user(request)
    with pool.connection() as conn:
        t = conn.execute("SELECT t.*, r.map_id, r.status AS run_status FROM tasks t JOIN runs r USING (run_id) "
                         "WHERE t.task_id = %s", (task_id,)).fetchone()
        if not t:
            raise HTTPException(404, "없는 task입니다")
        aps = conn.execute("SELECT ap_id, ssid, is_target FROM aps WHERE map_id = %s ORDER BY ssid", (t["map_id"],)).fetchall()
        events = conn.execute("SELECT event_id, code, source, level, detail, ts FROM robot_events WHERE task_id = %s "
                              "ORDER BY ts, event_id", (task_id,)).fetchall()
        # 실제로 지나간 길 — 제어 PC status(1 Hz, robot_status_log)의 그 task 동안 위치 · 배터리
        track = conn.execute("SELECT ts, x, y, battery_pct, nav_state FROM robot_status_log WHERE robot_id = %s "
                             "AND task_id = %s ORDER BY ts", (t["robot_id"], task_id)).fetchall()
        grid = grid_of(conn, t["map_id"])
        m_ = re.match(r"seg:(\d+)-", t["target_ref"] or "")
        plan = []
        for i, w in enumerate(t["waypoints"] or []):   # 계획 순서 — 경로점 i(이 task 안 1부터)가 속한 칸
            c = cell_key(grid, float(w["x"]), float(w["y"]))
            plan.append({"i": i + 1, "path_i": (int(m_.group(1)) + i) if m_ else None, **(c or {"cell_id": None})})
        out: dict[str, Any] = {"task": t, "aps": aps, "events": events, "track": track, "plan_cells": plan,
                               "visits": [], "visits_source": None,
                               "thr": fset(settings_values(conn), "rssi_suspect_dbm", -72)}
        cell_cols = "g.ix, g.iy, g.sector, g.center_x, g.center_y"
        if t["type"] == "SURVEY_SEGMENT":               # 측정 — 표본 합계 · 칸별 AP 값 · 실제 순서 · 이 구간 후보
            # 표본 — net_samples.task_id(3-3차). 그 전 표본은 같은 로봇 · run에서 이 task 수행 시간 안의 것
            where, args, basis = "n.task_id = %s", [task_id], "task_id"
            if not conn.execute("SELECT 1 FROM net_samples WHERE task_id = %s LIMIT 1", (task_id,)).fetchone() \
                    and t["assigned_at"]:
                where = "n.robot_id = %s AND n.run_id = %s AND n.stamp >= %s AND n.stamp <= COALESCE(%s, now())"
                args, basis = [t["robot_id"], t["run_id"], t["assigned_at"], t["ended_at"]], "time"
            tot = conn.execute(f"SELECT count(*) AS n, count(*) FILTER (WHERE NOT n.pose_ok) AS pose_bad, "
                               f"count(*) FILTER (WHERE n.pose_ok AND n.cell_id IS NULL) AS off_grid, "
                               f"min(n.stamp) AS first, max(n.stamp) AS last FROM net_samples n WHERE {where}", args).fetchone()
            if not tot["n"]:                           # 수행 시간으로 찾아도 없음 — 표본이 없는 task
                basis = "none"
            rows = conn.execute(
                f"SELECT n.cell_id, {cell_cols}, n.ap_id, count(*) AS n, "
                f"count(*) FILTER (WHERE n.detected) AS n_detected, count(*) FILTER (WHERE NOT n.detected) AS n_missed, min(n.stamp) AS first, "
                f"percentile_cont(0.5) WITHIN GROUP (ORDER BY n.rssi_dbm)::float8 AS median, "
                f"round(avg(n.rssi_dbm), 1)::float8 AS mean, min(n.rssi_dbm) AS min, max(n.rssi_dbm) AS max "
                f"FROM net_samples n JOIN grid_cells g USING (cell_id) WHERE {where} AND n.pose_ok "
                f"GROUP BY n.cell_id, {cell_cols}, n.ap_id ORDER BY min(n.stamp)", args).fetchall()
            cells: dict[int, dict[str, Any]] = {}
            for r in rows:                             # 칸마다 AP별 값 · 처음 지난 시각 순서
                c = cells.setdefault(r["cell_id"], {k: r[k] for k in ("cell_id", "ix", "iy", "sector", "center_x", "center_y")}
                                     | {"first": r["first"], "by_ap": {}})
                c["first"] = min(c["first"], r["first"])
                c["by_ap"][r["ap_id"]] = {k: r[k] for k in ("n", "n_detected", "n_missed", "median", "mean", "min", "max")}
            out["survey"] = {"basis": basis, "totals": tot, "cells": sorted(cells.values(), key=lambda c: c["first"])}
            # 실제 순서 — 표본(rssi 주기)을 시각순으로. 같은 시각의 여러 AP 표본은 한 번으로 셈
            pts = conn.execute(f"SELECT DISTINCT n.stamp, n.cell_id, g.ix, g.iy FROM net_samples n JOIN grid_cells g USING (cell_id) "
                               f"WHERE {where} AND n.pose_ok ORDER BY n.stamp", args).fetchall()
            out["visits"] = cell_visits((r["stamp"], {k: r[k] for k in ("cell_id", "ix", "iy")}) for r in pts)
            out["visits_source"] = "samples" if out["visits"] else None
            out["candidates"] = conn.execute(
                f"SELECT c.candidate_id, c.ref, c.score, c.status, c.outcome, c.reinspect_task_id, c.target_x, c.target_y, "
                f"f.finding_id, {cell_cols} FROM candidates c LEFT JOIN grid_cells g ON g.cell_id::text = c.ref "
                f"LEFT JOIN findings f USING (candidate_id) WHERE c.detect_task_id = %s ORDER BY c.candidate_id",
                (task_id,)).fetchall()
        elif t["type"] == "REINSPECTION":               # 정밀측정 — 후보 · 결론(finding) · AP별 정밀측정 · 측정 때 그 칸 값
            c = conn.execute(
                f"SELECT c.*, f.finding_id, f.status AS finding_status, {cell_cols} FROM candidates c "
                f"LEFT JOIN grid_cells g ON g.cell_id::text = c.ref LEFT JOIN findings f USING (candidate_id) "
                f"WHERE c.candidate_id = %s", (t["target_ref"],)).fetchone()
            out["candidate"] = c
            out["precise"] = conn.execute(
                "SELECT p.ap_id, a.ssid, p.duration_s, p.n_samples, p.rssi_median, p.rssi_iqr, p.measured_at "
                "FROM precise_measures p JOIN aps a USING (ap_id) WHERE p.task_id = %s ORDER BY a.ssid", (task_id,)).fetchall()
            out["survey_cell"] = conn.execute(           # 후보가 된 측정 때 그 칸의 값 (비교용)
                "SELECT n.ap_id, count(*) AS n, percentile_cont(0.5) WITHIN GROUP (ORDER BY n.rssi_dbm)::float8 AS median "
                "FROM net_samples n WHERE n.run_id = %s AND n.cell_id::text = %s AND n.pose_ok GROUP BY n.ap_id",
                (t["run_id"], c["ref"])).fetchall() if c and c["source"] == "NETWORK" else []
        elif t["type"] == "FACILITY_CHECK":             # 탐지 — 설비 · 판정 · 판정마다 사진(media)
            out["facility"] = conn.execute(
                "SELECT facility_id, facility_type, has_gauge, expected_x, expected_y, view_pose, is_active "
                "FROM facilities WHERE map_id = %s AND facility_id = %s", (t["map_id"], t["target_ref"])).fetchone()
            ins = conn.execute(
                "SELECT inspection_id, detail, final, frontal, pose_retry, gauge_state, confidence, bbox, model_version, "
                "inspected_at FROM inspections WHERE task_id = %s ORDER BY inspection_id", (task_id,)).fetchall()
            for i in ins:
                i["media"] = conn.execute("SELECT object_key, kind, size_bytes, captured_at FROM media "
                                          "WHERE inspection_id = %s ORDER BY media_id", (i["inspection_id"],)).fetchall()
            out["inspections"] = ins
        if not out["visits"] and track:                # 측정 표본이 없거나 측정이 아닌 task — status(1 Hz) 위치로
            out["visits"] = cell_visits((p["ts"], cell_key(grid, float(p["x"]), float(p["y"]))) for p in track)
            out["visits_source"] = "status" if out["visits"] else None
    return out


def ui_extra() -> dict[str, Any]:
    """/ws/ui에 함께 — 현재 지도 AP 활성 · 측정 AP(정보 탭 실시간), 순찰 시작 조건(모니터 버튼) → {aps, start}"""
    with pool.connection() as conn:
        aps = conn.execute("SELECT a.ap_id, a.ssid, a.is_active, a.is_target FROM aps a JOIN maps m USING (map_id) "
                           "WHERE m.is_active ORDER BY a.ap_id").fetchall()
        f = start_form(conn)
    return {"aps": aps, "start": {k: f[k] for k in ("can_start", "problems", "running", "target_ap")}
            | {"robots": f["robots"]}, "run_owner": RUN_OWNER, "estopped": ESTOP["on"] and bool(f["running"])}


@app.websocket("/ws/ui")
async def ws_ui(ws: WebSocket):
    """server page(브라우저) 실시간 — 1초마다 {type "robots", ts, timeout_s, robots(robots_snapshot), run_id,
    events(진행 중 run의 최근 30개), aps, start}를 보냄. 받는 메시지는 없음. 로그인 세션 쿠키는 연결 때만 확인"""
    await ws.accept()
    if not await run_in_threadpool(session_get, ws.cookies.get(SESSION_COOKIE)):
        await ws.close(code=4401, reason="login required")
        return
    try:
        while True:                                    # 끊길 때까지 1초마다 보냄 (DB 조회는 스레드 풀에서)
            snap = await run_in_threadpool(robots_snapshot)
            extra = await run_in_threadpool(ui_extra)
            cur = CURRENT_RUN["run_id"]
            with MEM_LOCK:                             # 진행 중 순찰의 이벤트만 (순찰이 없으면 비움)
                events = [e for e in RECENT_EVENTS if cur and e.get("run_id") == cur][-30:]
                plans = {k: list(v) for k, v in ROBOT_PLAN.items()}
            await ws.send_json({"type": "robots", "ts": datetime.now(timezone.utc).isoformat(),
                                "timeout_s": status_timeout(), "robots": snap, "run_id": cur,
                                "events": json.loads(json.dumps(events, default=str)), "plans": plans, **extra})
            await asyncio.sleep(1.0)
    except (WebSocketDisconnect, RuntimeError):
        pass


@app.get("/api/agents")
def agents(request: Request):
    """지금 연결된 제어 PC 목록 (server page 확인용) — AGENTS의 robot_id"""
    current_user(request)
    return {"robots": sorted(AGENTS)}


if __name__ == "__main__":
    uvicorn.run(app, host=HOST, port=PORT, workers=1)
