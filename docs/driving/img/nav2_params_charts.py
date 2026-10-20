#!/usr/bin/env python3
# Nav2 설정값 정리(docs/driving/NAV2_PARAMS.md)용 그림 4장 → docs/driving/img/nav2_params_*.png
#   python3 nav2_params_charts.py            (저장소 루트를 스스로 찾음)
# 원본 데이터: docs/driving/data/2026-10-11_robot3_amcl*.csv, 2026-10-12_track_check.csv, 2026-10-12_tol_check.csv
import csv, sys, os, statistics as st
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm

ROOT = sys.argv[1] if len(sys.argv) > 1 else os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
DATA = f"{ROOT}/docs/driving/data"
OUT = f"{ROOT}/docs/driving/img"
fm.fontManager.addfont("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
_fp = fm.FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
plt.rcParams["font.family"] = _fp.get_name()
plt.rcParams["axes.unicode_minus"] = False
BLUE, ORANGE, GREEN, GRAY, RED = "#3b6ea8", "#e07b39", "#4a9a5b", "#888888", "#c9403c"

def rows(name):
    with open(f"{DATA}/{name}", newline="") as f:
        return list(csv.DictReader(f))

# ── 1. 3-2 track_check: 설정별 경로 이탈 ───────────────────────────────
groups = [  # (tag prefix, label, exclude)
    ("mppi_",     "MPPI\n지움 0.7 m\n(robot4)", None),
    ("rpp_",      "RPP la0.6\n지움 0.7 m\n(robot4)", None),
    ("mppi_bt_",  "MPPI\n지움 0.35 m\n(robot3)", None),
    ("rpp_bt_",   "RPP la0.6\n지움 0.35 m\n(robot3)", "2026-10-12 14:24:48"),
    ("rpp_la04_", "RPP la0.4\n지움 0.35 m\n(robot3) ★채택", None),
    ("keepout_empty_", "채택값 +\n빈 keepout\n(robot3)", None),
]
tc = rows("2026-10-12_track_check.csv")
means, maxs, ns = [], [], []
for pre, _, ex in groups:
    sel = [r for r in tc if r["tag"].startswith(pre) and r["tag"][len(pre):].isdigit() and r["time"] != ex]
    means.append(st.mean(float(r["mean_cm"]) for r in sel))
    maxs.append(max(float(r["max_cm"]) for r in sel))
    ns.append(len(sel))
fig, ax = plt.subplots(figsize=(10, 4.8))
x = range(len(groups)); w = 0.38
b1 = ax.bar([i - w/2 for i in x], means, w, color=BLUE, label="이탈 평균 (회차 평균)")
b2 = ax.bar([i + w/2 for i in x], maxs, w, color=ORANGE, label="이탈 최대 (회차 중 최대)")
ax.axhline(5, color=BLUE, ls="--", lw=1); ax.text(len(groups)-0.45, 5.4, "목표 평균 ≤ 5 cm", color=BLUE, ha="right", fontsize=9, bbox=dict(fc="white", ec="none", pad=1))
ax.axhline(10, color=ORANGE, ls="--", lw=1); ax.text(len(groups)-0.45, 10.4, "목표 최대 ≤ 10 cm", color=ORANGE, ha="right", fontsize=9, bbox=dict(fc="white", ec="none", pad=1))
for b, v in zip(list(b1)+list(b2), means+maxs):
    ax.text(b.get_x()+b.get_width()/2, v+0.6, f"{v:.1f}", ha="center", fontsize=9)
ax.set_xticks(list(x)); ax.set_xticklabels([f"{g[1]}\nn={n}" for g, n in zip(groups, ns)], fontsize=9)
ax.set_ylabel("cross-track 이탈 (cm)"); ax.set_ylim(0, max(maxs)+8)
ax.set_title("3-2 컨트롤러·lookahead·waypoint 지움 거리별 경로 이탈 (10/12, 코스 약 7.2 m, data/2026-10-12_track_check.csv)", fontsize=11)
ax.legend(loc="upper right"); ax.grid(axis="y", alpha=0.3)
fig.tight_layout(); fig.savefig(f"{OUT}/nav2_params_track_check.png", dpi=130); plt.close(fig)

# ── 2. 3-4 tol_check: 허용 오차 단계별 도착 오차 ────────────────────────
tl = rows("2026-10-12_tol_check.csv")
# 그룹: (tol_xy, tol_yaw, 회전 속도) — 마지막 6행(16:50 이후)이 0.5 rad/s
def grp(r):
    rot = 0.5 if r["time"] >= "2026-10-12 16:50" else 1.0
    return (float(r["tol_xy_m"]), float(r["tol_yaw_deg"]), rot)
order = [(0.10, 10.0, 1.0), (0.08, 8.0, 1.0), (0.05, 5.0, 1.0), (0.03, 3.0, 1.0), (0.03, 3.0, 0.5)]
labels = ["0.10 m / 10°\n회전 1.0", "0.08 m / 8°\n회전 1.0", "0.05 m / 5°\n회전 1.0\n★일반", "0.03 m / 3°\n회전 1.0", "0.03 m / 3°\n회전 0.5\n★정밀·회전 채택"]
fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4.6))
for i, g in enumerate(order):
    sel = [r for r in tl if grp(r) == g]
    xy = [float(r["err_xy_cm"]) for r in sel]; yaw = [float(r["err_yaw_deg"]) for r in sel]
    c = GREEN if i >= 2 else GRAY
    a1.scatter([i]*len(xy), xy, color=c, zorder=3, s=28)
    a1.hlines(g[0]*100, i-0.3, i+0.3, color=RED, lw=2)
    a2.scatter([i]*len(yaw), yaw, color=c, zorder=3, s=28)
    a2.hlines(g[1], i-0.3, i+0.3, color=RED, lw=2)
    a2.text(i, max(yaw)+0.6, f"평균 {st.mean(yaw):.1f}°", ha="center", fontsize=8.5)
    a1.text(i, max(xy)+0.3, f"평균 {st.mean(xy):.1f}", ha="center", fontsize=8.5)
for a, t, yl in ((a1, "위치 오차 (Nav2 feedback, AMCL 기준)", "cm"), (a2, "방향 오차", "°")):
    a.set_xticks(range(len(order))); a.set_xticklabels(labels, fontsize=8.5)
    a.set_title(t, fontsize=10.5); a.set_ylabel(yl); a.grid(axis="y", alpha=0.3)
a1.plot([], [], color=RED, lw=2, label="허용 오차 설정값"); a1.legend(loc="upper right", fontsize=9)
fig.suptitle("3-4 goal checker 허용 오차 단계 시험 (10/12, robot3, A↔B 각 6회, data/2026-10-12_tol_check.csv) — 위치는 경계 안에서 멈추고 방향은 회전 속도가 좌우", fontsize=10.5)
fig.tight_layout(); fig.savefig(f"{OUT}/nav2_params_tol_check.png", dpi=130); plt.close(fig)

# ── 3. 2-2 AMCL 튜닝: 재방문 오차·std ───────────────────────────────────
import math
cfg = [("기본값\n(update_min_d 0.25 · a 0.2 · beams 60)", [5.3, 4.8, 6.2], "2026-10-11_robot3_amcl.csv"),
       ("튜닝 ①\n(0.10 · 0.1 · 60)", [2.3, 5.6, 2.5], "2026-10-11_robot3_amcl_t1.csv"),
       ("튜닝 ①+② ★채택\n(0.10 · 0.1 · 120)", [2.0, 4.1, 0.6], "2026-10-11_robot3_amcl_t2.csv")]
fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4.4))
w = 0.25
for j, pt in enumerate("ABC"):
    a1.bar([i + (j-1)*w for i in range(3)], [c[1][j] for c in cfg], w, label=f"점 {pt}", color=[BLUE, ORANGE, GREEN][j])
for i, c in enumerate(cfg):
    a1.text(i, max(c[1])+0.3, f"평균 {st.mean(c[1]):.1f} cm", ha="center", fontsize=9)
a1.axhline(5, color=RED, ls="--", lw=1); a1.text(1.0, 5.12, "제안 기준 5 cm", color=RED, fontsize=8.5, ha="center", bbox=dict(fc="white", ec="none", pad=1))
a1.set_ylim(0, 7.4)
a1.set_xticks(range(3)); a1.set_xticklabels([c[0] for c in cfg], fontsize=8.5)
a1.set_ylabel("재방문 오차 (cm, 줄자)"); a1.set_title("재방문 오차 (점당 1회)", fontsize=10.5); a1.legend(fontsize=8.5); a1.grid(axis="y", alpha=0.3)
for i, c in enumerate(cfg):
    rs = rows(c[2])
    if i == 0:
        rs = [r for r in rs if r["label"] != "robot3_A_0"]
    sx = [math.hypot(float(r["std_x_cm"]), float(r["std_y_cm"]))/math.sqrt(2) for r in rs]
    sy = [float(r["std_yaw_deg"]) for r in rs]
    a2.scatter([i-0.12]*len(sx), sx, color=BLUE, s=28, zorder=3)
    a2.scatter([i+0.12]*len(sy), sy, color=ORANGE, s=28, marker="s", zorder=3)
a2.scatter([], [], color=BLUE, label="위치 std (x·y 평균, cm)"); a2.scatter([], [], color=ORANGE, marker="s", label="yaw std (°)")
a2.set_xticks(range(3)); a2.set_xticklabels([c[0] for c in cfg], fontsize=8.5); a2.legend(fontsize=8.5)
a2.set_title("기록 시점 AMCL 표준편차 (갱신이 잦아져 수렴)", fontsize=10.5); a2.grid(axis="y", alpha=0.3)
fig.suptitle("2-2 AMCL 튜닝 전후 (10/11, robot3, data/2026-10-11_robot3_amcl*.csv)", fontsize=11)
fig.tight_layout(); fig.savefig(f"{OUT}/nav2_params_amcl.png", dpi=130); plt.close(fig)

# ── 4. 변경 시간순 타임라인 ─────────────────────────────────────────────
days = [
 ("10/11\n5e3c287", [
   ("amcl", "update_min_d 0.25→0.10 · update_min_a 0.2→0.1 · max_beams 60→120", "2-2 재방문 5.4→2.2 cm"),
   ("controller", "required_movement_radius 0.5→0.15", "저속 Failed to make progress"),
   ("controller", "MPPI vx_min −0.35→0.0 (nav2.yaml 만)", "Create 3 후진 거부"),
   ("controller", "MPPI vx_max 0.5→0.2 · smoother max_velocity 0.26→0.2", "측정 속도 0.2 m/s"),
   ("controller", "xy/yaw tol 0.25→0.10 / 0.17 (1차)", "목표치 0-1"),
   ("costmap", "keepout_layer 추가 (enabled false 로 시작)", "IF-10 준비")]),
 ("10/12\na7018f1 · f8aaffb\n26bab7c · 8034dfd · fd23581", [
   ("controller", "progress_checker Simple→PoseProgressChecker (angle 0.5)", "제자리 회전 중 끊김 0회"),
   ("controller", "FollowPath MPPI→RPP · lookahead 0.6→0.4", "3-2 이탈 10.3→4.5 cm"),
   ("bt", "RemovePassedGoals radius 0.7→0.35", "코너 최대 이탈 38→17 cm"),
   ("controller", "xy/yaw tol 0.10→0.05 / 0.087 · rotate_to_heading 1.0→0.5", "3-4 방향 오차 3.9→2.9°"),
   ("controller", "max_robot_pose_search_dist 10→1.0", "4-2 waypoint 건너뜀"),
   ("costmap", "keepout enabled true · map_topic {ns}/ · use_maximum costmap 단위", "4-1 재계획 3.9·1.1 s"),
   ("costmap", "voxel origin_z 0.0→−0.1", "origin out of bounds 경고")]),
 ("10/13\nb839d82", [
   ("smoother", "velocity_smoother min_velocity x −0.2→0.0 (TB4 원본 −0.26)", "후진 0건 (322 s bag)"),
   ("bt", "BackUp 제거 (두 BT) · behavior_server backup 삭제", "PM step3 리뷰")]),
]
color = {"amcl": "#7b4fa0", "controller": BLUE, "bt": GREEN, "costmap": ORANGE, "smoother": RED}
fig, ax = plt.subplots(figsize=(13, 6.2))
ax.set_xlim(0, 3); ax.set_ylim(0, 8.2); ax.axis("off")
for i, (day, items) in enumerate(days):
    ax.text(i + 0.5, 7.9, day, ha="center", va="top", fontsize=10.5, fontweight="bold")
    ax.plot([i+0.5, i+0.5], [0.2, 7.0], color=GRAY, lw=1, ls=":")
    for k, (node, what, why) in enumerate(items):
        y = 6.6 - k*0.9
        ax.add_patch(plt.Rectangle((i+0.03, y-0.36), 0.94, 0.78, fc="white", ec=color[node], lw=1.6, zorder=2))
        ax.text(i+0.06, y+0.13, what, fontsize=8.3, va="center", zorder=3, wrap=True)
        ax.text(i+0.06, y-0.2, f"근거: {why}", fontsize=7.6, va="center", color="#444444", zorder=3)
for n, c in color.items():
    ax.plot([], [], color=c, lw=3, label={"amcl": "amcl (localization.yaml)", "controller": "controller_server", "bt": "bt_navigator / BT xml / behavior_server", "costmap": "costmap", "smoother": "velocity_smoother"}[n])
ax.legend(loc="lower center", ncol=5, fontsize=8.5, frameon=False, bbox_to_anchor=(0.5, -0.02))
ax.set_title("Nav2 설정 변경 시간순 (날짜 · 커밋) — 한 칸 = 변경 한 묶음, 아래 줄 = 시험 근거", fontsize=11.5)
fig.tight_layout(); fig.savefig(f"{OUT}/nav2_params_timeline.png", dpi=130); plt.close(fig)
print("ok")
