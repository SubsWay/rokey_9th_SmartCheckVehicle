"""RSSI 샘플을 격자 칸으로 모으고 색을 정한다 (ROS 와 무관, pytest 대상).

- 칸 번호는 지도 원점(origin) 기준: i = floor((x - ox) / cell), j = floor((y - oy) / cell)
- 칸 대표값·약한 신호 판정은 검출된 RSSI 의 중앙값 (1차 · 재측정 같은 기준, 10/15 PM). 평균은 기록용. 미검출 샘플은 통계에 넣지 않고 따로 센다
- (robot_id, seq) 가 같은 샘플은 한 번만, pose_ok 가 False 인 샘플은 버린다
- 회피 마스크: 미검출·음영 칸(0.5 m 칸 그대로, margin 을 주면 그만큼 넓혀)은 100, 나머지(정상·측정 안 한 칸) 0 인 OccupancyGrid 값
"""

import colorsys
import math
import statistics
from dataclasses import dataclass, field
from typing import Dict, List, NamedTuple, Optional, Tuple

Color = Tuple[float, float, float]

# 신호 세기 → 색 (NetSpot 식 그라데이션): 강할수록 빨강, 약할수록(음영) 파랑
#   STRONG_DBM 이상 빨강 → 주황 → 노랑 → 초록 → 하늘 → WEAK_DBM 이하 파랑
STRONG_DBM = -35
WEAK_DBM = -75
MISS_COLOR: Color = (0.4, 0.4, 0.4)    # 회색 (미검출 — 파랑(약함)과 구분)

# 칸 상태 (회피 마스크용). 음영 기준은 BR-T04 의 −72 dBm
SHADOW_DBM = -72
NORMAL, SHADOW, MISS = 'normal', 'shadow', 'miss'
UNKNOWN = 'unmeasured'
MIN_SAMPLES = 3
# Keepout filter값
# ROS지도(OccupancyGrid)에서 100은 "벽처럼 막힘"을 뜻함. Nav2가 이 값을 보고 그 칸을 피함.
KEEPOUT = 100   


class Sample(NamedTuple):
    robot_id: str        #어떤 로봇이 쟀는가?
    seq: int             #그 로봇의 몇 번째 측정인가?
    x: float             # 위치(미터단위)
    y: float
    pose_ok: bool        #위치를 믿을 수 있는가?
    detected: bool       #와이파이가 잡혔는가?
    rssi_dbm: int        #잡혔다면 세기는 어느정도인가?

# 신호 세기를 0~1 사이 점수 t로 바꾸는 함수. 약할수록 0, 강할수록 1
# -55 dbm 예시) (-55-(-75)) / (-35-(-75)) = 20/40=0.5
def rssi_to_color(rssi: float, strong: float = STRONG_DBM, weak: float = WEAK_DBM) -> Color:
    t = (rssi - weak) / (strong - weak)
    # 0보다 작으면0, 1보다 크면 1로 자름. 이상치가 와도 값을 제한하기 위함
    t = min(1.0, max(0.0, t))
    # HSV 색상환으로 변환. 0도에 가까울 수록 빨간색, 240도에 가까울 수록 파란색
    # t=1(강함)이면 0 * 2/3 = 0 즉, 빨강. t=0이면 2/3(240도) 즉 파랑
    return colorsys.hsv_to_rgb((1.0 - t) * 2.0 / 3.0, 1.0, 1.0)

# ox, oy는 지도의 왼쪽 아래 모서리(원점)
def cell_index(x: float, y: float, ox: float, oy: float, cell: float) -> Tuple[int, int]:
    # x - ox는 원점에서 오른쪽으로 몇 m 떨어졌나이고, 칸 크기(0.5m)로 나누면 몇 칸째 인지 알 수 있음.
    # floor를 사용하여 int에서 생기는 오차를 제거.
    return (math.floor((x - ox) / cell), math.floor((y - oy) / cell))


# cell_index의 반대 방향으로, +0.5로 칸의 한가운데 좌표를 얻음.
# 화면에 색 네모와 글자를 그릴 때 사용함.
def cell_center(i: int, j: int, ox: float, oy: float, cell: float) -> Tuple[float, float]:
    return (ox + (i + 0.5) * cell, oy + (j + 0.5) * cell)

# __init__같은 귀찮은 코드 생략용
@dataclass
class CellStats:
    # 이 칸에서 잡힌 신호 세기 목록
    # = []를 쓰면 모든 칸이 같은 한 목록을 공유하므로, 에러를 방지하기위한 방어코드
    rssi: List[int] = field(default_factory=list)
    # 신호를 못 잡은 횟수
    n_missed: int = 0

    @property # 괄호 없이 값을 읽을 수 있게 @property를 붙임
    def n_detected(self) -> int: # 값이 잡힌 횟수
        return len(self.rssi)

    @property
    def n_total(self) -> int: # 잡힌 횟수 + 못 잡은 횟수
        return self.n_detected + self.n_missed

    @property
    def median(self) -> Optional[float]: # 검출 rssi 중앙값. 값이 없으면 None
        return statistics.median(self.rssi) if self.rssi else None

    @property
    def mean(self) -> Optional[float]:
        return statistics.fmean(self.rssi) if self.rssi else None

    @property
    def miss_ratio(self) -> float: # 신호 못 잡은 비율 (0으로 나누기 방지)
        return self.n_missed / self.n_total if self.n_total else 0.0


# 이 위치를 써도 되는지 bool값과 이유로 알려주는 함수
def pose_usable(age_s: Optional[float], cov_xy: Optional[float],
                max_age: float = 0.5, max_cov: float = 0.25,
                lag_s: Optional[float] = None) -> Tuple[bool, str]:
    """TF 로 얻은 위치를 지도에 써도 되는지. (쓸 수 있는지, 안 되면 이유)

    age_s: 측정 시각과 붙인 TF 시각의 차 [s] (None 이면 TF 를 못 찾음)
    lag_s: 지금 시각 - 가장 최근 TF 시각 [s]. 통신 지연이거나 노트북·로봇 시계 차.
           측정 시각의 TF 를 찾았어도(age 0) 시계가 어긋나 있으면 엉뚱한 과거 위치일 수 있어 따로 본다
    max_age <= 0 이면 age·lag 검사를 하지 않는다
    cov_xy: AMCL 공분산(자신없음정도) xx + yy [m^2] (None 이면 amcl_pose 를 아직 못 받음 → 검사 안 함)
    """
    if age_s is None:
        return False, 'TF 없음'
    # 시간이 앞서든 뒤쳐지든 문제라 절대값 사용
    if max_age > 0 and lag_s is not None and abs(lag_s) > max_age:
        return False, f'최근 TF 가 지금보다 {lag_s:+.2f} s (통신 지연 또는 노트북·로봇 시계 차)'
    if max_age > 0 and abs(age_s) > max_age:
        return False, f'TF 시각 차 {age_s:.2f} s > {max_age} s'
    if cov_xy is not None and cov_xy > max_cov:
        return False, f'AMCL 공분산 {cov_xy:.3f} > {max_cov} m^2'
    # 모두 통과
    return True, ''

# 칸의 색상 정하는 함수. 다른 함수에서 계산된 값 받아옴.
# 신호가 안잡히거나 절반 이상 못잡으면 회색. 검출 중앙값으로 색상 정함
def cell_color(stats: CellStats, miss_ratio_thresh: float = 0.5,
               strong: float = STRONG_DBM, weak: float = WEAK_DBM) -> Color:
    if stats.n_detected == 0 or stats.miss_ratio >= miss_ratio_thresh:
        return MISS_COLOR
    return rssi_to_color(stats.median, strong, weak)

# 약한 신호: 검출 RSSI 중앙값 < -72. 미측정과 미검출을 구분한다.
def cell_status(stats: CellStats, shadow_dbm: float = SHADOW_DBM,
                miss_ratio_thresh: float = 0.5) -> str:
    """미측정 / 미검출 비율 이상 / 약한 신호 / 정상. 신뢰도(최소 표본)는 호출자가 검사."""
    return rf_status(stats.median, stats.n_detected, stats.n_missed, shadow_dbm, miss_ratio_thresh)


def rf_status(rssi, n_detected, n_missed, threshold=SHADOW_DBM, miss_ratio=.5):
    # rssi: 검출 RSSI 의 대표값(중앙값)
    total = n_detected + n_missed
    if total == 0:
        return UNKNOWN
    if n_detected == 0 or n_missed / total >= miss_ratio:
        return MISS
    return SHADOW if rssi < threshold else NORMAL

# Nav2가 피해야할 칸 목록
# 최소 전체 표본 수 이상인 칸만 판정함
def keepout_cells(cells: Dict[Tuple[int, int], CellStats], shadow_dbm: float = SHADOW_DBM,
                  miss_ratio_thresh: float = 0.5, min_samples: int = MIN_SAMPLES) -> Dict[Tuple[int, int], str]:
    """회피할 칸 → 상태(MISS·SHADOW). 샘플이 min_samples 개보다 적은 칸은 한 번 튄 값일 수 있어 뺀다"""
    # [GRID-2] 표본 수·미검출·RSSI 기준으로 회피 셀 선택
    out = {}
    for idx, st in cells.items():
        if st.n_total < min_samples:
            continue
        status = cell_status(st, shadow_dbm, miss_ratio_thresh)
        if status != NORMAL:
            out[idx] = status
    return out

# 회피 마스크 그리는 함수
# 지도는 5cm 픽셀로 되어, 0.5m칸 하나가 10x10 픽셀이 됨.
# 나쁜 칸에 해당하는 픽셀을 100으로 칠한 1차원 목록을 만듦.
def keepout_mask(cells, cell_size: float, origin: Tuple[float, float], resolution: float,
                 width: int, height: int, margin: float = 0.0) -> List[int]:
    """OccupancyGrid.data (아래 행부터, 행 우선). 칸 사각형에서 margin [m] 안의 픽셀은 KEEPOUT, 나머지 0.

    지도(/map)와 원점·해상도·크기가 같다고 본다 (회전 없는 지도). 측정 안 한 칸은 0.
    KeepoutFilter 는 inflation 뒤에 적용돼 회피 칸 주변이 팽창되지 않는다. 기본(margin 0)은 칸 그대로라
    로봇 중심은 칸에 못 들어가지만 몸체(반경 0.19 m)는 칸 가장자리에 걸칠 수 있다.
    """
    # [GRID-3] 0.5 m 셀을 지도 해상도의 OccupancyGrid 픽셀로 변환
    ox, oy = origin
    # 마스크 값 전부 0으로 초기화
    data = [0] * (width * height)
    # 여유거리 제곱하여 딱 경계에 있는 픽셀이 빠지지 않게 하는 안전장치
    m2 = margin * margin + 1e-9
    # 나쁜 칸 마다
    for i, j in cells:
        # 칸의 왼쪽 아래
        x0, y0 = ox + i * cell_size, oy + j * cell_size
        # 칸의 오른쪽 위
        x1, y1 = x0 + cell_size, y0 + cell_size
        # 검사할 픽셀
        c0 = max(0, math.floor((x0 - margin - ox) / resolution))
        c1 = min(width - 1, math.floor((x1 + margin - ox) / resolution))
        # 범위(열/행)
        r0 = max(0, math.floor((y0 - margin - oy) / resolution))
        r1 = min(height - 1, math.floor((y1 + margin - oy) / resolution))
        #칸에 여유만큼 더한 사각형 범위 안 픽셀만 보고, 지도 밖의 번호(음수, 너무 큰 수)가 나오지 않게 자름

        for r in range(r0, r1 + 1):
            # 픽셀 가운데 y
            py = oy + (r + 0.5) * resolution
            # 칸 까지의 세로거리
            dy = max(y0 - py, 0.0, py - y1)
            for c in range(c0, c1 + 1):
                #픽셀 가운데 x
                px = ox + (c + 0.5) * resolution
                # 칸까지의 가로 거리
                dx = max(x0 - px, 0.0, px - x1)
                # 픽셀에서 칸 까지의 거리가 margin이하인지 확인하는 피타고라스 식
                # 거리를 원 모양으로 재기 때문에 넓힌 영역의 모서리가 둥글게 됨.
                # margin이 0이면 가운데가 칸 안에 있는 픽셀만 칠해짐.
                if dx * dx + dy * dy <= m2:     # 모서리는 둥글게
                    data[r * width + c] = KEEPOUT
    return data

# heatmap.py 전체 manager코드
class SurveyGrid:
    """받은 샘플을 보관하고, 요청할 때 칸별 통계를 계산한다.

    지도의 원점·범위는 /map 을 받은 뒤 바뀔 수 있어서 샘플 자체를 보관하고 칸은 매번 다시 계산한다.
    """

    # 칸 크기, 원점, 범위, 샘플 목록, 이미 본 샘플 집합
    def __init__(self, cell_size: float = 0.5, origin: Tuple[float, float] = (0.0, 0.0)):
        self.cell_size = cell_size
        self.origin = origin
        self.bounds: Optional[Tuple[float, float, float, float]] = None  # xmin, ymin, xmax, ymax
        self._samples: List[Sample] = []
        self._seen = set()

    # 지도를 받으면 원점과 지도 범위를 저장
    def set_map(self, ox: float, oy: float, width_m: float, height_m: float) -> None:
        self.origin = (ox, oy)
        self.bounds = (ox, oy, ox + width_m, oy + height_m)

    # 새 측정 시작. 샘플과 중복 기록만 비우고 지도 정보만 유지
    def clear(self) -> None:
        self._samples = []
        self._seen = set()

    # 좌표가 지도 안인지 판단하는 함수
    def in_bounds(self, x: float, y: float) -> bool:
        if self.bounds is None:
            return True
        xmin, ymin, xmax, ymax = self.bounds
        # 오른쪽, 위쪽 경계를 포함하지 않게 범위 설정
        return xmin <= x < xmax and ymin <= y < ymax

    # 로봇, 번호를 이미 봤으면 거절, or 봤다고 기록, or 위치를 못믿으면 거절, or 저장하는 함수
    def add(self, s: Sample) -> bool:
        """지도에 쓸 샘플이면 보관하고 True. 중복·pose_ok False 는 False."""
        # [GRID-4] 중복·위치 불확실 표본 제외
        key = (s.robot_id, s.seq)
        if key in self._seen:
            return False
        self._seen.add(key)
        if not s.pose_ok:
            return False
        self._samples.append(s)
        return True

    # 지도에 쓴 샘플 복사본을 돌려줌. 원본은 그대로 두도록 하는 함수.
    @property
    def samples(self) -> List[Sample]:
        """지도에 쓴 샘플 (범위 밖 포함)"""
        return list(self._samples)

    # 매번 새로 계산, 지도 밖 샘플은 건너뛰고, 칸 번호를 구해서 그 칸의 성적표에 기록
    def cells(self) -> Dict[Tuple[int, int], CellStats]:
        # [GRID-5] 누적 표본을 셀별 통계로 집계
        ox, oy = self.origin
        out: Dict[Tuple[int, int], CellStats] = {}
        for s in self._samples:
            if not self.in_bounds(s.x, s.y):
                continue
            # 칸 번호가 있으면 꺼내고, 없으면 새 빈 성적표를 넣고 꺼낸다.
            # if문 없이 setdefault로 처리
            st = out.setdefault(cell_index(s.x, s.y, ox, oy, self.cell_size), CellStats())
            if s.detected:
                st.rssi.append(s.rssi_dbm)
            else:
                st.n_missed += 1
        return out

    # 칸 번호의 가운데 좌표
    def center(self, idx: Tuple[int, int]) -> Tuple[float, float]:
        return cell_center(idx[0], idx[1], self.origin[0], self.origin[1], self.cell_size)
