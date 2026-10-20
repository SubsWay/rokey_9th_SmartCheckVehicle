"""정지 재측정의 유효 표본 창과 판정. ROS 없이 경계 조건을 시험한다."""
import math
from dataclasses import dataclass, field

from scv_survey.heatmap import MIN_SAMPLES, CellStats, rf_status

DURATION_S = 15


def classify(rssi, n_detected, n_missed, *, valid=True, min_samples=MIN_SAMPLES,
             threshold=-72.0, miss_ratio=.5):
    """rssi 는 검출 중앙값. None은 판정 불가. 무표본을 미검출 이상으로 바꾸지 않는다."""
    total = n_detected + n_missed
    if not valid or min(n_detected, n_missed) < 0 or total < min_samples:
        return None
    if n_detected and (rssi is None or not math.isfinite(rssi) or not -127 <= rssi <= 0):
        return None
    return rf_status(rssi, n_detected, n_missed, threshold, miss_ratio)


@dataclass
class MeasurementWindow:
    start_ns: int
    duration_s: int
    robot_id: str
    stats: CellStats = field(default_factory=CellStats)
    seen: set = field(default_factory=set)

    def add(self, stamp_ns, robot_id, detected, rssi, now_ns):
        # latched 과거 표본, 시간 창 밖, 중복 스캔, 다른 로봇을 배제한다.
        if (robot_id != self.robot_id or stamp_ns in self.seen
                or not self.start_ns <= stamp_ns <= self.start_ns + self.duration_s * 10**9
                or not 0 <= now_ns - stamp_ns <= 2 * 10**9):
            return False
        if detected and not -127 <= rssi <= 0:
            return False
        self.seen.add(stamp_ns)
        if detected:
            self.stats.rssi.append(rssi)
        else:
            self.stats.n_missed += 1
        return True
