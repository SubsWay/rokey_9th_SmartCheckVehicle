#!/usr/bin/env python3
"""
가상 세계 사진 · 영상 만들기 (3-3차) — world.yaml의 truth가 가리키는 파일을 합성으로 만듦

  판정 사진  truth.facilities.<설비 ID>.photo   설비 종류(register) × 실제 상태(truth)로 그린 640 × 360 JPEG
             normal · abnormal(압력계 바늘 빨강 구역 · 유도등 꺼짐 · 셔터 휨 · 랙 선반 기울어짐) · not_detected(빈 자리) · occluded(상자에 가림)
             · position(옆으로 옮겨짐 + 원래 자리 점선)
  순찰 영상  truth.camera.video                 지도(pgm)를 1인칭으로 그린 순찰 영상 (mp4, 640 × 360, 10 fps, 기본 45초)
             가상 로봇이 움직일 때 차례로 내보냄(멈추면 그 장면) — 로봇 위치와 맞추지는 않음

  묶음에 이미 만들어 둔 파일이 있으므로 다시 만들 때만 실행합니다. 같은 이름의 실제 사진 · 영상으로 바꿔 끼워도 됩니다.
  필요 패키지: numpy · pillow · pyyaml (사진), opencv(python3-opencv — 영상)

  사용
    python3 make_assets.py testbed/world.yaml                  # 사진 + 영상
    python3 make_assets.py testbed/world.yaml --photos         # 사진만
    python3 make_assets.py testbed/world.yaml --video --seconds 60
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from world_model import World

W, H = 640, 360                                                # 사진 · 영상 크기 (px)
FPS = 10                                                       # 영상 초당 장 수
STATES = ("normal", "abnormal", "not_detected", "occluded", "position")   # truth.facilities.<ID>.truth 허용 값


def font(size: int):
    """기본 글꼴 — Pillow 10.1+는 크기 지정, 이전 버전은 고정 크기"""
    try:
        return ImageFont.load_default(size=size)              # Pillow 10.1+
    except TypeError:                                          # pragma: no cover — 이전 Pillow
        return ImageFont.load_default()


# =====================================================================
# 판정 사진
# =====================================================================
def background() -> Image.Image:
    """벽(위 72 %) + 바닥 타일(원근)"""
    a = np.zeros((H, W, 3), np.uint8)
    wall_h = int(H * 0.72)
    g = np.linspace(0.9, 1.0, wall_h)[:, None, None]                                # 위가 조금 어두움
    a[:wall_h] = (np.array([226, 222, 210])[None, None, :] * g).astype(np.uint8)
    a[wall_h:] = (150, 152, 156)
    img = Image.fromarray(a)
    d = ImageDraw.Draw(img)
    d.rectangle([0, wall_h - 8, W, wall_h], fill=(120, 116, 108))                     # 걸레받이
    for i in range(-8, 9):                                                            # 바닥 타일 줄 (소실점)
        d.line([(W / 2 + i * 26, wall_h), (W / 2 + i * 150, H)], fill=(132, 134, 138), width=1)
    for yy in (wall_h + 10, wall_h + 24, wall_h + 44, wall_h + 72):                   # 가로 줄 — 멀수록 촘촘
        d.line([(0, yy), (W, yy)], fill=(132, 134, 138), width=1)
    return img


def dashed_rect(d: ImageDraw.ImageDraw, box, color, dash=8):
    """점선 사각형 — box = [x0, y0, x1, y1], dash px 그리고 dash px 띄움"""
    x0, y0, x1, y1 = box
    for x in range(int(x0), int(x1), dash * 2):                                       # 위 · 아래 변
        d.line([(x, y0), (min(x + dash, x1), y0)], fill=color, width=2)
        d.line([(x, y1), (min(x + dash, x1), y1)], fill=color, width=2)
    for y in range(int(y0), int(y1), dash * 2):                                       # 왼쪽 · 오른쪽 변
        d.line([(x0, y), (x0, min(y + dash, y1))], fill=color, width=2)
        d.line([(x1, y), (x1, min(y + dash, y1))], fill=color, width=2)


def draw_extinguisher(d, cx, floor_y, gauge: bool, bad: bool):
    """소화기 — 빨간 몸통 · 손잡이 · 호스 · 압력계(바늘이 초록 구역 = 정상, 빨강 구역 · 빨간 바늘 = 이상) · 받침.
    압력계는 gauge일 때만 — 압력계 없는 소화기는 bad여도 겉모습이 같음. 반환: 설비가 차지한 상자 [x0, y0, x1, y1]"""
    bw, bh = 70, 170
    x0, y0 = cx - bw // 2, floor_y - bh
    d.rounded_rectangle([x0 + 6, floor_y - 6, x0 + bw + 14, floor_y + 4], 4, fill=(110, 110, 110))   # 그림자 · 받침
    d.rounded_rectangle([x0, y0, x0 + bw, floor_y], 22, fill=(200, 28, 32), outline=(120, 10, 14), width=3)
    d.rectangle([x0 + 8, y0 + 60, x0 + bw - 8, y0 + 110], fill=(240, 236, 228))                    # 라벨
    for i in range(4):                                                                               # 라벨 글자 줄
        d.line([(x0 + 14, y0 + 70 + i * 10), (x0 + bw - 14, y0 + 70 + i * 10)], fill=(150, 30, 30), width=2)
    d.rectangle([cx - 10, y0 - 22, cx + 10, y0 + 4], fill=(60, 60, 64))                             # 밸브
    d.polygon([(cx - 10, y0 - 22), (cx + 34, y0 - 30), (cx + 34, y0 - 22), (cx - 10, y0 - 14)], fill=(40, 40, 44))
    d.line([(cx + 10, y0 - 10), (x0 + bw + 18, y0 + 30), (x0 + bw + 14, floor_y - 40)], fill=(25, 25, 25), width=6)  # 호스
    if gauge:                                                                                        # 압력계
        # 눈금 호 — 200~250° 빨강(이상) · 255~320° 초록(정상). 바늘 각도 215°(이상) / 290°(정상)
        gx, gy, r = cx - 26, y0 - 4, 17
        d.ellipse([gx - r, gy - r, gx + r, gy + r], fill=(250, 250, 250), outline=(40, 40, 40), width=3)
        d.arc([gx - r + 4, gy - r + 4, gx + r - 4, gy + r - 4], 200, 250, fill=(220, 40, 40), width=4)
        d.arc([gx - r + 4, gy - r + 4, gx + r - 4, gy + r - 4], 255, 320, fill=(30, 170, 60), width=4)
        ang = math.radians(215 if bad else 290)
        d.line([(gx, gy), (gx + math.cos(ang) * (r - 5), gy + math.sin(ang) * (r - 5))],
               fill=(200, 20, 20) if bad else (20, 20, 20), width=3)
    return [x0 - 30, y0 - 34, x0 + bw + 26, floor_y + 4]


def draw_shutter(d, cx, floor_y, bad: bool):
    """방화 셔터 — 금속 틀 + 가로 판. 이상 = 아래가 휘어 틈이 벌어짐. 반환: 설비 상자 [x0, y0, x1, y1]"""
    sw, sh = 360, 230
    x0, y0 = cx - sw // 2, floor_y - sh
    d.rectangle([x0 - 14, y0 - 22, x0 + sw + 14, floor_y], fill=(90, 94, 100))                       # 틀
    d.rectangle([x0 - 10, y0 - 22, x0 + sw + 10, y0], fill=(70, 74, 80))                             # 셔터 상자
    for i in range(0, sh, 14):                                                                       # 가로 판 (14 px마다)
        top = y0 + i
        sag = 0
        if bad and i > sh - 70:                                    # 이상 — 아래 70 px의 판일수록 더 처짐
            sag = int((i - (sh - 70)) * 0.5)
        d.rectangle([x0, top, x0 + sw, top + 12], fill=(178, 184, 190))
        d.line([(x0, top + 12), (x0 + sw, top + 12)], fill=(120, 126, 132), width=2)
        if sag:                                                    # 가운데 아래로 벌어진 틈 (어두운 사다리꼴)
            d.polygon([(x0 + 120, top), (x0 + 240, top), (x0 + 200, top + 12 + sag), (x0 + 160, top + 12 + sag)],
                      fill=(60, 60, 60))
    d.rectangle([cx - 30, floor_y - 18, cx + 30, floor_y - 8], fill=(220, 180, 40))                  # 손잡이 띠
    return [x0 - 14, y0 - 22, x0 + sw + 14, floor_y]


def draw_exit_light(d, cx, wall_y, bad: bool):
    """유도등 — 초록 상자 + 흰 비상구 그림. 이상 = 꺼짐(어두움). wall_y = 상자 위 끝. 반환: 설비 상자 [x0, y0, x1, y1]"""
    bw, bh = 190, 80
    x0, y0 = cx - bw // 2, wall_y
    d.rectangle([x0 - 4, y0 - 4, x0 + bw + 4, y0 + bh + 4], fill=(60, 60, 60))                       # 테두리
    body = (40, 60, 44) if bad else (24, 190, 88)
    d.rectangle([x0, y0, x0 + bw, y0 + bh], fill=body)
    fg = (110, 120, 112) if bad else (250, 255, 250)
    d.rectangle([x0 + bw - 62, y0 + 12, x0 + bw - 22, y0 + bh - 12], outline=fg, width=4)             # 문
    hx, hy = x0 + 62, y0 + 22                                                                        # 달리는 사람
    d.ellipse([hx - 8, hy - 8, hx + 8, hy + 8], fill=fg)
    d.line([(hx, hy + 8), (hx - 6, hy + 30)], fill=fg, width=6)
    d.line([(hx - 6, hy + 30), (hx - 24, hy + 44)], fill=fg, width=5)
    d.line([(hx - 6, hy + 30), (hx + 10, hy + 46)], fill=fg, width=5)
    d.line([(hx - 2, hy + 14), (hx + 22, hy + 22)], fill=fg, width=5)
    d.line([(hx - 2, hy + 14), (hx - 22, hy + 18)], fill=fg, width=5)
    d.polygon([(x0 + 100, y0 + 40), (x0 + 118, y0 + 30), (x0 + 118, y0 + 50)], fill=fg)             # 화살표
    if not bad:                                                                                       # 빛 번짐
        d.rectangle([x0 - 8, y0 + bh + 4, x0 + bw + 8, y0 + bh + 10], fill=(150, 225, 170))
    return [x0 - 4, y0 - 4, x0 + bw + 4, y0 + bh + 4]


def draw_rack(d, cx, floor_y, bad: bool):
    """랙(3-3 보완) — 금속 기둥 4개 + 선반 3단, 선반마다 상자. 이상 = 가운데 선반이 한쪽으로 기울고 상자 하나가 바닥에 떨어짐.
    반환: 설비 상자 [x0, y0, x1, y1]"""
    rw, rh = 260, 240
    x0, y0 = cx - rw // 2, floor_y - rh
    post = (70, 78, 90)
    for px in (x0, x0 + rw - 10):                                                                    # 앞 기둥 2개 (뒤 기둥은 안쪽에 옅게)
        d.rectangle([px, y0, px + 10, floor_y], fill=post)
        d.rectangle([px + 18, y0 + 8, px + 24, floor_y - 6], fill=(120, 126, 136))
    shelves = [floor_y - 12, floor_y - 92, floor_y - 172]                                            # 선반 높이 (아래 → 위)
    for k, sy in enumerate(shelves):
        tilt = 22 if bad and k == 1 else 0                                       # 이상 — 가운데 선반의 오른쪽이 22 px 내려감
        d.polygon([(x0 + 10, sy), (x0 + rw - 10, sy + tilt), (x0 + rw - 10, sy + tilt + 8), (x0 + 10, sy + 8)],
                  fill=(150, 156, 166))
        for j, (bx, bw, bh) in enumerate([(20, 70, 50), (100, 60, 62), (170, 70, 44)]):             # 선반 위 상자 3개
            if bad and k == 1 and j == 2:                                        # 기운 쪽 끝 상자는 바닥으로 떨어짐
                continue
            drop = int(tilt * (bx + bw / 2) / rw)                                # 기운 선반 위 상자는 그만큼 내려앉음
            d.rectangle([x0 + bx, sy - bh + drop, x0 + bx + bw, sy + drop], fill=(176, 132, 82), outline=(110, 78, 44), width=2)
    if bad:                                                                                          # 바닥에 떨어진 상자 (비스듬히)
        fx, fy = x0 + rw + 20, floor_y + 6
        d.polygon([(fx, fy), (fx + 66, fy - 14), (fx + 76, fy - 56), (fx + 10, fy - 42)], fill=(176, 132, 82), outline=(110, 78, 44))
    return [x0, y0, x0 + rw + (100 if bad else 0), floor_y + 6]


def draw_facility(d, ftype, cx, gauge, bad):
    """종류별로 설비 그리기 — 가운데 x = cx. extinguisher · shutter · rack은 바닥에, 그 밖(exit_light)은 벽 위(y 70)에.
    반환: 설비 상자 [x0, y0, x1, y1]"""
    floor_y = int(H * 0.72) + 30
    if ftype == "extinguisher":
        return draw_extinguisher(d, cx, floor_y, gauge, bad)
    if ftype == "shutter":
        return draw_shutter(d, cx, floor_y, bad)
    if ftype == "rack":
        return draw_rack(d, cx, floor_y, bad)
    return draw_exit_light(d, cx, 70, bad)                     # 알 수 없는 종류도 유도등으로 그림


def draw_boxes(d, box):
    """가림 — 설비 앞에 쌓인 상자 3개 (아래 2개 + 위 1개). box = 가릴 설비 상자"""
    x0, y0, x1, y1 = box
    floor_y = int(H * 0.72) + 34
    w = max(150, (x1 - x0) * 0.9)
    cx = (x0 + x1) / 2
    # (왼쪽 x, 너비, 높이) — 세 번째(i == 2)는 130 px 위에 얹음
    for i, (bx, bw, bh) in enumerate([(cx - w / 2 - 20, w * 0.62, 150), (cx - w / 2 + w * 0.45, w * 0.6, 120),
                                       (cx - w / 2 + 10, w * 0.5, 110)]):
        by = floor_y - bh - (130 if i == 2 else 0)
        d.rectangle([bx, by, bx + bw, by + bh], fill=(176, 132, 82), outline=(110, 78, 44), width=3)
        d.line([(bx, by + bh * 0.45), (bx + bw, by + bh * 0.45)], fill=(206, 176, 120), width=8)       # 테이프


def make_photo(fid: str, ftype: str, gauge: bool, state: str, out: Path):
    """판정 사진 한 장 — 종류(ftype · gauge) × 실제 상태(state)로 그려 out에 JPEG(품질 88)로 저장 (폴더도 만듦)"""
    img = background()
    d = ImageDraw.Draw(img)
    cx = W // 2
    if state == "not_detected":                        # 빈 자리 — 걸이 · 받침만
        floor_y = int(H * 0.72) + 30
        if ftype == "exit_light":                      # 유도등 — 벽 걸이만
            d.rectangle([cx - 30, 70, cx + 30, 80], fill=(120, 120, 120))
        elif ftype == "rack":                          # 랙 — 바닥에 기둥 자국 4개만 (치워진 자리)
            for px in (cx - 130, cx - 112, cx + 112, cx + 130):
                d.rectangle([px - 5, floor_y - 4, px + 5, floor_y + 2], fill=(110, 110, 110))
        else:                                          # 소화기 · 셔터 — 바닥 받침 + 벽 걸이
            d.rectangle([cx - 40, floor_y - 6, cx + 40, floor_y + 4], fill=(120, 120, 120))
            d.rectangle([cx - 6, floor_y - 150, cx + 6, floor_y - 120], fill=(130, 130, 130))
        dashed_rect(d, [cx - 70, 50, cx + 70, floor_y], (200, 60, 60))
    elif state == "position":                          # 옆으로 옮겨짐 + 원래 자리 점선
        box = draw_facility(d, ftype, cx + 170, gauge, False)          # 170 px 오른쪽에 정상 모습으로
        w_ = box[2] - box[0]
        dashed_rect(d, [cx - w_ / 2, box[1], cx + w_ / 2, box[3]], (230, 140, 20))
        # 원래 자리 오른쪽 끝 → 옮겨진 설비 왼쪽 끝을 잇는 가로선 (화살촉 없음)
        d.line([(cx + w_ / 2 + 6, (box[1] + box[3]) / 2), (box[0] - 6, (box[1] + box[3]) / 2)], fill=(230, 140, 20), width=3)
    else:                                              # normal · abnormal · occluded — 제자리에 (occluded는 상자를 덧그림)
        box = draw_facility(d, ftype, cx, gauge, state == "abnormal")
        if state == "occluded":
            draw_boxes(d, box)
    # 아래 글자 띠 — 설비 ID · 종류 · 정답 상태
    d.rectangle([0, H - 26, W, H], fill=(20, 20, 20))
    d.text((10, H - 21), f"SCV virtual camera   {fid}  {ftype}{' (gauge)' if gauge else ''}   truth: {state}",
           fill=(235, 235, 235), font=font(14))
    out.parent.mkdir(parents=True, exist_ok=True)
    img.save(out, format="JPEG", quality=88)


def make_photos(w: World) -> int:
    """truth.facilities마다 판정 사진을 그 photo 경로에 만듦 — 종류 · 압력계는 register.facilities(같은 설비 ID)에서.
    photo가 없거나 register에 없는 설비는 건너뜀, truth가 STATES 밖이면 종료. 반환: 만든 장 수"""
    n = 0
    for fid, t in (w.truth.get("facilities") or {}).items():
        reg = w.facility_reg(fid)
        out = w.file(t.get("photo"))
        if not out:
            print(f"  {fid}: photo 경로가 없어 건너뜀")
            continue
        if not reg:
            print(f"  {fid}: register.facilities에 없어 종류를 몰라 건너뜀")
            continue
        state = t.get("truth", "normal")
        if state not in STATES:
            sys.exit(f"{fid}: truth '{state}'는 {STATES} 중 하나여야 합니다")
        make_photo(fid, reg["type"], bool(reg.get("gauge")), state, out)
        print(f"  사진 {out.relative_to(w.base)} — {fid} {reg['type']} {state}")
        n += 1
    return n


# =====================================================================
# 순찰 영상 — 지도 1인칭 (레이캐스팅)
# =====================================================================
PATROL = [(1.0, 0.9), (1.0, 7.0), (3.0, 7.2), (5.0, 5.8), (5.0, 3.0), (5.2, 1.0), (4.6, 5.6), (7.0, 6.6),
          (7.2, 1.2), (11.0, 1.0), (11.2, 4.4), (10.0, 4.6), (10.0, 7.0), (8.6, 6.6), (10.0, 4.4), (7.6, 3.6),
          (5.6, 6.2), (1.6, 6.0), (1.0, 0.9)]                      # testbed 한 바퀴 (작은 방 문 · 칸막이 끝을 지남)


def patrol_poses(seconds: float, speed=0.6, turn=1.6):
    """경로점을 차례로 (제자리 회전 → 직진) — FPS마다 자세 (x, y, yaw).
    speed m/s · turn rad/s, seconds × FPS개를 채울 때까지 PATROL을 되풀이 (한 바퀴는 약 113초)"""
    poses, x, y = [], *PATROL[0]
    yaw = math.atan2(PATROL[1][1] - y, PATROL[1][0] - x)
    dt = 1.0 / FPS
    i = 1
    while len(poses) < seconds * FPS:
        tx, ty = PATROL[i % len(PATROL)]
        head = math.atan2(ty - y, tx - x)
        err = math.atan2(math.sin(head - yaw), math.cos(head - yaw))      # 목표 방향과의 각도 차 (−π ~ π)
        if abs(err) > 0.03:                                              # 아직 안 맞음 — 한 장에 turn·dt까지 회전
            yaw += math.copysign(min(abs(err), turn * dt), err)
        else:                                                            # 맞음 — 한 장에 speed·dt까지 직진, 닿으면 다음 경로점
            dist = math.hypot(tx - x, ty - y)
            step = min(dist, speed * dt)
            x, y = x + math.cos(head) * step, y + math.sin(head) * step
            if dist - step < 1e-3:
                i += 1
        poses.append((x, y, yaw))
    return poses


def render_view(w: World, x, y, yaw, rw=320, rh=180, fov=math.radians(72)):
    """레이캐스팅 — 열마다 벽까지 거리 → 벽 높이 · 밝기, 바닥은 1 m 타일.
    (x, y, yaw)에서 본 rw × rh RGB 화면(uint8 배열)을 돌려줌 — 파일은 쓰지 않음"""
    # 광선 — 화면 왼쪽(+fov/2) → 오른쪽, 1 cm 간격으로 14 m까지 표본. 지도 밖은 벽으로 봄
    ang = yaw + np.linspace(fov / 2, -fov / 2, rw)
    t = np.arange(0.05, 14.0, 0.01)
    xs = x + np.cos(ang)[:, None] * t[None, :]
    ys = y + np.sin(ang)[:, None] * t[None, :]
    rows = w.h - 1 - np.floor((ys - w.oy) / w.res).astype(int)
    cols = np.floor((xs - w.ox) / w.res).astype(int)
    inside = (rows >= 0) & (rows < w.h) & (cols >= 0) & (cols < w.w)
    hit = np.ones_like(inside)
    hit[inside] = w.wall[rows[inside], cols[inside]]
    k = hit.argmax(axis=1)                                           # 열마다 처음 벽에 닿은 표본 번호
    dist = t[k] * np.cos(ang - yaw)                                  # 어안 보정
    hx, hy = xs[np.arange(rw), k], ys[np.arange(rw), k]
    # 벽 면 방향 — 마지막 한 걸음에서 행(y)이 바뀌었으면 가로 벽(y면), 열(x)만 바뀌었으면 세로 벽
    kp = np.maximum(k - 1, 0)
    ri = np.arange(rw)
    r0, c0, r1, c1 = rows[ri, kp], cols[ri, kp], rows[ri, k], cols[ri, k]
    both = (r0 != r1) & (c0 != c1)                                   # 대각선으로 넘어감 — 사이 칸 (r1, c0)이 벽이면 가로 벽
    mid = w.wall[np.clip(r1, 0, w.h - 1), np.clip(c0, 0, w.w - 1)]
    face_y = np.where(both, mid, r0 != r1)
    u = np.where(face_y, hx, hy)                                     # 벽 면을 따라간 좌표(m) — 벽판 이음 위치에 씀
    img = np.zeros((rh, rw, 3), np.float32)
    horizon = rh / 2
    yy = np.arange(rh)[:, None]
    # 천장(지평선 위, 아래로 갈수록 조금 어두움) · 바닥(지평선 아래, 멀수록 어두움)
    img[: int(horizon)] = np.array([205, 208, 214])[None, None, :] * (0.75 + 0.25 * (1 - yy[: int(horizon)] / horizon))[:, :, None]
    fy = yy[int(horizon):] - horizon + 0.5
    fd = (0.5 * rh * 0.55) / fy                                       # 카메라 높이 0.55 · 화면 줄마다 바닥 거리 (벽 아래 끝 원근과 정확히 맞춘 값은 아님 — 보기용 근사)
    fdx = x + np.cos(ang)[None, :] * fd / np.cos(ang - yaw)[None, :]
    fdy = y + np.sin(ang)[None, :] * fd / np.cos(ang - yaw)[None, :]
    tile = ((np.floor(fdx) + np.floor(fdy)) % 2)[:, :, None]                    # 1 m 바닥 타일 (옅은 체크)
    floor = np.where(tile > 0, np.array([150, 152, 156]), np.array([160, 162, 166])) * np.clip(1.2 - fd[:, :, None] / 12, 0.45, 1.0)
    img[int(horizon):] = floor
    # 벽 — 위 밝은 벽 · 아래 1/3 징두리(어두운 띠, wh × 0.33) · 1 m마다 벽판 이음(가까울수록 진하게, 5 m 넘으면 사라짐)
    #   벽 높이 wh ∝ 1/거리, 지평선 기준 위 1/2 · 아래 0.95 × 1/2. 세로 벽(x면)은 0.84배 어둡게
    wh = np.clip(rh * 1.1 / np.maximum(dist, 0.05), 0, rh * 4)
    top, bot = horizon - wh / 2, horizon + wh / 2 * 0.95
    shade = np.clip(1.15 - dist / 11, 0.35, 1.0) * np.where(face_y, 1.0, 0.84)
    frac = np.abs(u - np.round(u))
    du = np.abs(np.gradient(u))                                       # 화면 한 열이 덮는 벽 길이 — 크면 이음이 깨져 보이므로 흐리게
    seam = np.clip(1 - frac / 0.025, 0, 1) * np.clip(1 - dist / 5, 0, 1) * np.clip(1 - du / 0.08, 0, 1)
    upper = np.array([226, 222, 210])[None, :] * (1 - 0.35 * seam[:, None])
    lower = np.array([168, 176, 184])[None, :] * (1 - 0.35 * seam[:, None])
    mask = (yy >= top[None, :]) & (yy <= bot[None, :])
    band = yy >= (bot - wh * 0.33)[None, :]
    wall = np.where(band[:, :, None], lower[None, :, :], upper[None, :, :]) * shade[None, :, None]
    img = np.where(mask[:, :, None], wall, img)
    skirt = (yy <= bot[None, :]) & (yy >= bot[None, :] - wh[None, :] * 0.05)       # 걸레받이
    img = np.where(skirt[:, :, None], (np.array([96, 92, 86]) * shade[:, None])[None, :, :], img)
    return np.clip(img, 0, 255).astype(np.uint8)


def make_video(w: World, out: Path, seconds: float) -> int:
    """순찰 영상 — patrol_poses 자세마다 render_view(320 × 180)를 640 × 360으로 키워 out에 mp4(mp4v, FPS)로 씀.
    OpenCV가 없거나 파일을 못 열면 종료. 반환: 장 수"""
    try:
        import cv2
    except ImportError:
        sys.exit("영상을 만들려면 OpenCV가 필요합니다 (sudo apt install python3-opencv 또는 pip install opencv-python)")
    out.parent.mkdir(parents=True, exist_ok=True)
    vw = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))
    if not vw.isOpened():
        sys.exit(f"영상 파일을 열지 못했습니다: {out}")
    poses = patrol_poses(seconds)
    for i, (x, y, yaw) in enumerate(poses):                          # 한 장씩 그림 → BGR → 키움 → 아래에 좌표 글자
        rgb = render_view(w, x, y, yaw)
        frame = cv2.resize(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), (W, H), interpolation=cv2.INTER_LINEAR)
        cv2.putText(frame, f"SCV virtual patrol  x {x:5.2f}  y {y:5.2f}", (10, H - 12), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (40, 40, 40), 1, cv2.LINE_AA)
        vw.write(frame)
    vw.release()
    return len(poses)


def main():
    """명령줄 — world.yaml을 읽어 사진 · 영상(둘 다 또는 --photos / --video 하나)을 만들고 결과를 출력"""
    ap = argparse.ArgumentParser(description="가상 세계 사진 · 영상 만들기")
    ap.add_argument("world", help="world.yaml (예: testbed/world.yaml)")
    ap.add_argument("--photos", action="store_true", help="사진만")
    ap.add_argument("--video", action="store_true", help="영상만")
    # 주의: 도움말의 「한 바퀴」는 맞지 않음 — PATROL 한 바퀴는 약 113초, 기본 45초는 그 앞부분(약 40 %)만
    ap.add_argument("--seconds", type=float, default=45.0, help="영상 길이(초, 기본 45 — 한 바퀴)")
    a = ap.parse_args()
    w = World(a.world)
    both = not a.photos and not a.video                              # 둘 다 안 고르면 둘 다 만듦
    if a.photos or both:
        print(f"판정 사진 — {w.path}")
        print(f"  {make_photos(w)}장")
    if a.video or both:
        out = w.file((w.truth.get("camera") or {}).get("video"))
        if not out:
            sys.exit("truth.camera.video 경로가 없습니다")
        n = make_video(w, out, a.seconds)
        print(f"순찰 영상 {out.relative_to(w.base)} — {n}장 ({n / FPS:g}초, {out.stat().st_size // 1024} KB)")


if __name__ == "__main__":
    main()
