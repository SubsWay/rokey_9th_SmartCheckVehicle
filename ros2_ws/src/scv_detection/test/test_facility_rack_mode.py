"""IF-05 설비 점검 — 랙은 yolo 모드의 랙 적재 판정, 최종 판정은 마지막 확정 판정 · 최소 관측 3 s (10/16). cv2 · rclpy 없이 표와 함수만 꺼내 시험."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS

source = Path(__file__).parents[1] / 'scv_detection' / 'scv_detection_manager.py'
tree = ast.parse(source.read_text())
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'ScvDetectionManager')
methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in ('_facility_status', '_facility_decision')]
for m in methods:
    m.decorator_list = []
KIND = NS(KIND_EXTINGUISHER=1, KIND_SHUTTER=2, KIND_EXIT_LIGHT=3, KIND_RACK=4)
namespace = {'InspectionResult': KIND, 'VERDICTS': {'NORMAL': 0, 'ABNORMAL': 1}}
modes = next(n for n in tree.body if isinstance(n, ast.Assign) and n.targets[0].id == 'FACILITY_MODES')
exec(compile(ast.fix_missing_locations(ast.Module(body=[modes, *methods], type_ignores=[])), str(source), 'exec'), namespace)
FACILITY_MODES, status, decision = namespace['FACILITY_MODES'], namespace['_facility_status'], namespace['_facility_decision']


def rack_outcome(*statuses):
    return {'success': True, 'rack_loading': {'racks': [{'status': s} for s in statuses], 'boxes': []}}


def test_rack_uses_yolo_mode_not_old_opencv_mode():
    assert FACILITY_MODES[KIND.KIND_RACK] == 'yolo'


def test_rack_verdict_from_yolo_rack_loading():
    assert status(KIND.KIND_RACK, rack_outcome('NORMAL')) == 'NORMAL'
    assert status(KIND.KIND_RACK, rack_outcome('NORMAL', 'ABNORMAL')) == 'ABNORMAL'
    assert status(KIND.KIND_RACK, rack_outcome('UNCONFIRMED')) is None      # 확인 중 → 판정 아님
    assert status(KIND.KIND_RACK, rack_outcome()) is None                   # 랙을 못 봄
    assert status(KIND.KIND_RACK, {'success': True, 'rack_loading': {'racks': []}, 'pressures': []}) is None
    # 추론 성공(success=True)만으로 NORMAL 이 되면 안 된다 (이전 코드의 랙 판정 방식)
    assert status(KIND.KIND_RACK, {'success': True}) is None


def test_other_kinds_unchanged():
    assert status(KIND.KIND_EXTINGUISHER, {'pressures': [{'status': 'ABNORMAL'}]}) == 'ABNORMAL'
    assert status(KIND.KIND_EXIT_LIGHT, {'exit_signs': [{'status': 'NORMAL'}]}) == 'NORMAL'
    assert status(KIND.KIND_SHUTTER, {'shutter': {'status': 'UNCONFIRMED'}}) is None
    assert FACILITY_MODES[KIND.KIND_SHUTTER] == 'shutter'


def test_decision_follows_last_confirmed_status():
    """10/16: 한두 프레임 이상 뒤 정상으로 확정되면 NORMAL, 확정 판정이 없을 때만 발행한 불량으로 ABNORMAL"""
    assert decision('NORMAL', True) == 'NORMAL'
    assert decision('ABNORMAL', False) == 'ABNORMAL'
    assert decision(None, True) == 'ABNORMAL'
    assert decision(None, False) is None


def test_minimum_observation_time_declared():
    text = source.read_text()
    assert "declare_parameter('facility_min_s', 3.0)" in text
    assert 'time.monotonic() >= min_end' in text


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
            print('ok', name)
