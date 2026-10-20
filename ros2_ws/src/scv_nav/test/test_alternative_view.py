"""설비 관측 자리가 막혔을 때 대체 자리 고르기 (mission_executor.alternative_views) — Nav2 없이 메서드만 꺼내 시험."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS

source = Path(__file__).parents[1] / 'scripts' / 'mission_executor.py'
tree = ast.parse(source.read_text())
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'alternative_views')
consts = {t.id: ast.literal_eval(n.value) for n in tree.body if isinstance(n, ast.Assign)
          for t in n.targets if isinstance(t, ast.Name) and t.id in ('ALT_RINGS', 'ALT_MAX_COST')}


class Pose:   # PoseStamped 대역 (header · pose.position · pose.orientation)
    def __init__(self, header=None):
        self.header = header
        self.pose = NS(position=NS(x=0.0, y=0.0), orientation=NS(z=0.0, w=1.0))


namespace = {'math': math, 'PoseStamped': Pose, 'xy': lambda p: (p.pose.position.x, p.pose.position.y), **consts}
exec(compile(ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[])), str(source), 'exec'), namespace)
alternative_views = namespace['alternative_views']


def view(x, y, yaw):
    p = Pose('map')
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation = NS(z=math.sin(yaw / 2), w=math.cos(yaw / 2))
    return p


def robot(blocked):
    """blocked(x, y) 가 참이면 costmap 장애물(254), 아니면 0. 지도 장애물은 없음"""
    return NS(cost=lambda x, y: 254 if blocked(x, y) else 0, on_static_map=lambda x, y: False)


def test_nearest_free_ring_and_behind_first_keeping_yaw():
    v = view(1.0, 1.0, 0.0)                                    # +x 를 봄
    r = robot(lambda x, y: math.dist((x, y), (1.0, 1.0)) < 0.2)   # 관측 자리 둘레 0.2 m 막힘
    alts = alternative_views(r, v)
    assert len(alts) == 3
    first = alts[0]
    assert math.dist((first.pose.position.x, first.pose.position.y), (1.0, 1.0)) == __import__('pytest').approx(0.25)
    assert first.pose.position.x < 1.0                         # 보는 방향의 뒤쪽 (설비에서 멀어지는 쪽) 먼저
    assert first.pose.orientation is v.pose.orientation        # 방향은 원래 그대로


def test_all_blocked_returns_nothing():
    assert alternative_views(robot(lambda x, y: True), view(0.0, 0.0, 1.0)) == []


def test_static_map_obstacles_are_skipped():
    v = view(0.0, 0.0, 0.0)
    r = robot(lambda x, y: math.dist((x, y), (0, 0)) < 0.1)
    r.on_static_map = lambda x, y: x < 0                       # 뒤쪽은 벽
    alts = alternative_views(r, v)
    assert alts and all(a.pose.position.x >= -1e-9 for a in alts)
