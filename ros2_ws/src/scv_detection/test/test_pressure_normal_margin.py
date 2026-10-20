"""압력계 정상 구간 여유각 (10/16) — 바늘 시차 흡수. cv2 없이 widen_sector 만 꺼내 시험."""
import ast
from pathlib import Path

package = Path(__file__).parents[1] / 'scv_detection'
source = package / 'extinguisher_pressure.py'
tree = ast.parse(source.read_text())
func = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'widen_sector')
namespace = {}
exec(compile(ast.fix_missing_locations(ast.Module(body=[func], type_ignores=[])), str(source), 'exec'), namespace)
widen_sector = namespace['widen_sector']


def test_widen_both_sides():
    assert widen_sector((80.0, 100.0), 10.0) == (70.0, 110.0)


def test_wraps_around_zero():
    assert widen_sector((5.0, 30.0), 10.0) == (355.0, 40.0)


def test_zero_margin_keeps_sector():
    assert widen_sector((80.0, 100.0), 0.0) == (80.0, 100.0)


def test_full_circle_not_inverted():
    assert widen_sector((0.0, 345.0), 10.0) == (0.0, 345.0)


def test_band_uses_strict_yellow_only():
    text = source.read_text()
    assert 'BAND_NEEDLE_LOWER = (18, 80, 70)' in text
    assert 'cv2.bitwise_and(strict, band)' in text


def test_defaults():
    """모듈 기본값은 0(이전 동작 그대로), 노드는 10° 로 켬"""
    assert 'normal_margin_deg=0.0' in source.read_text()
    assert "declare_parameter('pressure_normal_margin_deg', 10.0)" in (package / 'scv_detection_manager.py').read_text()


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
            print('ok', name)
