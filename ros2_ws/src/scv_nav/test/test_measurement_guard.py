"""Nav2 없는 PC에서 실제 메서드를 추출해 정지·칸 조건을 검증. 주행 통합 시험은 아님."""
import ast
import math
import time
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

source = Path(__file__).parents[1] / 'scripts' / 'mission_executor.py'
cls = next(n for n in ast.parse(source.read_text()).body if isinstance(n, ast.ClassDef))
method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'measurement_guard')
namespace = {'math': math, 'time': time, 'DOCK_PCT': 20}
exec(compile(ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[])), str(source), 'exec'), namespace)
guard = namespace['measurement_guard']


def robot():
    return NS(battery=80, pose=(.25,.25,0,.05), pose_std_max=.15, cell_size=.5,
              static_map=NS(info=NS(origin=NS(position=NS(x=0,y=0)))),
              odom=(time.monotonic(),0,0))


def test_stopped_in_target_cell_is_valid():
    assert guard(robot(),(0,0),(.25,.25,0)) == ''


@pytest.mark.parametrize('attr,value,reason', [
    ('battery',10,'LOW_BATTERY'), ('pose',None,'POSE_UNCERTAIN'),
    ('pose',(.25,.25,0,.2),'POSE_UNCERTAIN'), ('static_map',None,'NO_MAP'),
    ('pose',(.51,.25,0,.05),'OUTSIDE_TARGET_CELL'),
    ('pose',(.31,.25,0,.05),'MOVED_DURING_MEASUREMENT'),
    ('pose',(.25,.25,.2,.05),'MOVED_DURING_MEASUREMENT'),
    ('odom',None,'NO_FRESH_ODOM'), ('odom',(0,0,0),'NO_FRESH_ODOM'),
])
def test_bad_measurement_cannot_clear_cell(attr,value,reason):
    r = robot()
    setattr(r,attr,value)
    assert guard(r,(0,0),(.25,.25,0)) == reason


def test_nonzero_measured_velocity_is_rejected():
    r = robot()
    r.odom = (time.monotonic(),.03,0)
    assert guard(r,(0,0),(.25,.25,0)) == 'NOT_STATIONARY'
