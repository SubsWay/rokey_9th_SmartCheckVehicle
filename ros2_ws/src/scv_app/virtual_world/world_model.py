"""
SCV 가상 세계 모델 (3-3차) — world.yaml 읽기 · 지도(벽) · RSSI 모델

  robot3_virtual.py(--world) · make_assets.py가 함께 씁니다. register_world.py는 yaml만 직접 읽습니다(map · register).
  world.yaml 구역: map(지도 파일 · 격자) · register(관제 PC 등록 값) · truth(실제 세계의 정답 — 가상 로봇 · make_assets.py만)

  RSSI (AP마다)
    p1m_dbm − 10·n·log10(d) − wall_loss_db · min(벽 수, max_walls) − (음영 구역 loss_db 합) + 잡음(σ = noise_db)
    → floor_dbm ~ ceil_dbm로 자르고 정수 dBm. d = AP 원점까지 거리(m, 0.3 m 미만은 0.3)
    벽 수 = AP 원점 → 로봇 직선이 지나는 지도 점유 칸(pgm 검은 부분) 덩어리 수. 원점이 벽 안이면 그 덩어리는 세지 않음
"""
from __future__ import annotations

import math
import random
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

OCC_THRESH = 0.65                      # map_server 기본 occupied_thresh — yaml에 있으면 그 값


class World:
    """world.yaml 한 개 — 지도 벽 격자 · register · truth 값과 RSSI 계산 (읽기 전용, 파일을 쓰지 않음)"""

    def __init__(self, path: str | Path):
        """world.yaml과 그 map.yaml(+ pgm)을 읽어 벽 격자를 만듦. map.yaml이 없으면 ValueError"""
        self.path = Path(path).resolve()
        self.base = self.path.parent
        d = yaml.safe_load(self.path.read_text(encoding="utf-8")) or {}
        self.map_cfg = d.get("map") or {}
        self.register = d.get("register") or {}
        self.truth = d.get("truth") or {}
        if "yaml" not in self.map_cfg:
            raise ValueError(f"{self.path}: map.yaml(지도 yaml 경로)이 없습니다")
        # 지도 — map_server 형식 yaml(image · resolution · origin)
        self.map_yaml = self.file(self.map_cfg["yaml"])
        meta = yaml.safe_load(self.map_yaml.read_text(encoding="utf-8"))
        self.map_pgm = (self.map_yaml.parent / meta["image"]).resolve()
        self.res = float(meta["resolution"])
        self.ox, self.oy = float(meta["origin"][0]), float(meta["origin"][1])
        # 벽 = 점유 확률 > occupied_thresh 이고 미탐색(205, 회색)이 아닌 픽셀
        raw = np.asarray(Image.open(self.map_pgm).convert("L"))
        occ = 1.0 - raw.astype(np.float32) / 255.0             # negate = 0 (server page 지도 등록 규칙과 같음)
        self.wall = (occ > float(meta.get("occupied_thresh", OCC_THRESH))) & (raw != 205)
        self.h, self.w = self.wall.shape
        # 신호 공통 값 (truth.signal) — 없으면 기본값
        sig = self.truth.get("signal") or {}
        self.wall_loss = float(sig.get("wall_loss_db", 0.0))
        self.max_walls = int(sig.get("max_walls", 3))
        self.floor = float(sig.get("floor_dbm", -100))
        self.ceil = float(sig.get("ceil_dbm", -20))

    def file(self, rel: str | None) -> Path | None:
        """world.yaml 기준 상대 경로 → 절대 경로 (없으면 None)"""
        return None if not rel else (self.base / rel).resolve()

    # ---------------------------------------------------------------- 지도
    def px(self, x: float, y: float) -> tuple[int, int]:
        """map 좌표 → (행, 열). 이미지 위쪽이 y가 큰 쪽"""
        return self.h - 1 - int(math.floor((y - self.oy) / self.res)), int(math.floor((x - self.ox) / self.res))

    def is_wall(self, x: float, y: float) -> bool:
        """그 점이 벽인지 — 지도 밖도 벽으로 봄"""
        r, c = self.px(x, y)
        return not (0 <= r < self.h and 0 <= c < self.w) or bool(self.wall[r, c])

    def walls_between(self, x0: float, y0: float, x1: float, y1: float) -> int:
        """직선이 지나는 벽 덩어리 수 (빈 곳 → 벽으로 들어갈 때마다 1)"""
        # 반 칸(res/2) 간격으로 표본 — 지도 밖 표본은 벽이 아닌 것으로 봄 (is_wall과 다름)
        n = max(2, int(math.hypot(x1 - x0, y1 - y0) / (self.res * 0.5)) + 1)
        t = np.linspace(0.0, 1.0, n)
        rows = self.h - 1 - np.floor((y0 + (y1 - y0) * t - self.oy) / self.res).astype(int)
        cols = np.floor((x0 + (x1 - x0) * t - self.ox) / self.res).astype(int)
        ok = (rows >= 0) & (rows < self.h) & (cols >= 0) & (cols < self.w)
        hit = np.zeros(n, bool)
        hit[ok] = self.wall[rows[ok], cols[ok]]
        # 빈 곳 → 벽으로 바뀌는 곳 수 — 시작점이 벽 안이면 그 덩어리는 세지 않음
        return int(np.count_nonzero(hit[1:] & ~hit[:-1]))

    # ---------------------------------------------------------------- RSSI
    def hole_loss(self, x: float, y: float) -> float:
        """그 점을 포함하는 음영 구역(truth.holes, 경계 포함)의 loss_db 합 (dB, 겹치면 더함)"""
        return sum(float(h.get("loss_db", 0)) for h in (self.truth.get("holes") or [])
                   if h["x0"] <= x <= h["x1"] and h["y0"] <= y <= h["y1"])

    def rssi_mean(self, ssid: str, x: float, y: float) -> float:
        """잡음 없는 기대값 (dBm) — truth.ap_models[ssid] 기준, 자르기(floor · ceil) 전. 없는 SSID면 KeyError"""
        m = self.truth["ap_models"][ssid]
        ax, ay = float(m["x"]), float(m["y"])
        d = max(0.3, math.hypot(x - ax, y - ay))                 # 원점 바로 옆에서 log10이 튀지 않게 최소 0.3 m
        walls = min(self.max_walls, self.walls_between(ax, ay, x, y))
        return (float(m.get("p1m_dbm", -35)) - 10 * float(m.get("n", 2.5)) * math.log10(d)
                - self.wall_loss * walls - self.hole_loss(x, y))

    def rssi_at(self, x: float, y: float, rng: random.Random | None = None) -> list[dict]:
        """rssi 토픽 scans — [{ssid, rssi_dbm}] (잡음 포함, floor ~ ceil로 자른 정수 dBm). AP는 truth.ap_models 전부"""
        rng = rng or random
        out = []
        for ssid, m in (self.truth.get("ap_models") or {}).items():
            v = self.rssi_mean(ssid, x, y) + rng.gauss(0, float(m.get("noise_db", 2.0)))
            out.append({"ssid": ssid, "rssi_dbm": int(round(max(self.floor, min(self.ceil, v))))})
        return out

    # ---------------------------------------------------------------- 설비
    def facility_reg(self, fid: str) -> dict | None:
        """register.facilities에서 그 설비 ID의 항목 (종류 · 압력계 · 위치 · 관측 자세). 없으면 None"""
        return next((f for f in (self.register.get("facilities") or []) if str(f.get("id")) == fid), None)

    def facility_truth(self, fid: str) -> dict | None:
        """truth.facilities[설비 ID] — 실제 상태(truth) · photo · moved_to. 없으면 None"""
        return (self.truth.get("facilities") or {}).get(fid)
