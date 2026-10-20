import pytest
from scv_survey.heatmap import CellStats, cell_status
from scv_survey.remeasurement import MeasurementWindow, classify


@pytest.mark.parametrize('values,missed,expected', [
    ([-90,-65,-65], 0, 'normal'),  # 중앙값 -65 로 판정 (10/15 PM: 1차·재측정 모두 중앙값). 평균이면 -73.33
    ([-72]*3, 0, 'normal'), ([-73]*3, 0, 'shadow'),
    ([-60]*3, 3, 'miss'), ([-60]*4, 2, 'normal'),
    ([], 3, 'miss'), ([-60], 29, 'miss'),
])
def test_same_policy_in_survey_and_reinspection(values, missed, expected):
    st = CellStats(values, missed)
    assert classify(st.median, st.n_detected, st.n_missed) == expected
    assert cell_status(st) == expected


@pytest.mark.parametrize('mean,det,miss,valid', [
    (None,0,0,True), (-60,1,0,True), (-60,2,0,True),
    (-60,3,0,False), (float('nan'),3,0,True), (None,3,0,True),
])
def test_inconclusive_does_not_clear_or_confirm(mean, det, miss, valid):
    assert classify(mean, det, miss, valid=valid) is None


def test_no_messages_differs_from_all_missed_and_rejects_stale_duplicates():
    w = MeasurementWindow(10**10, 15, 'robot4')
    assert classify(w.stats.mean, w.stats.n_detected, w.stats.n_missed) is None
    assert not w.add(9*10**9, 'robot4', True, -60, 10**10)
    assert not w.add(11*10**9, 'robot3', True, -60, 11*10**9)
    assert not w.add(11*10**9, 'robot4', True, -60, 14*10**9)
    for t in (11,12,13):
        assert w.add(t*10**9,'robot4',False,0,t*10**9)
        assert not w.add(t*10**9,'robot4',False,0,t*10**9)
    assert w.stats.n_total == 3 and w.stats.n_detected == 0
    assert classify(w.stats.mean, 0, 3) == 'miss'
