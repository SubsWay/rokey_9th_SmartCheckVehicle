"""관제 결과 콜백 시험. ROS 노드/실제 로봇을 실행하지 않는다."""
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest
from scv_msgs.action import ExecuteTask
from scv_manager.inspection_manager import InspectionManager
from scv_manager.manager_core import ManagerCore, TaskState


class Manager:
    remeasure_state = InspectionManager.remeasure_state
    apply_remeasure = InspectionManager.apply_remeasure
    retry_remeasure_results = InspectionManager.retry_remeasure_results
    on_task_result = InspectionManager.on_task_result

    def __init__(self, saved=True):
        self.core = ManagerCore(['robot4'])
        rid = self.core.start_run()
        self.core.create_task(task_id='reinspect', run_id=rid, task_type=2,
                              priority=1, target_ref='cell:1,2', preferred_robot_id='robot4')
        self.core.assign_task('reinspect', 'robot4')
        self.core.start_task('reinspect')
        self.active_goals, self.canceling_task_ids = {}, set()
        self.pending_remeasure_results, self.confirmed_cells = {}, set()
        self.store = NS(update_cell=Mock(return_value=saved))
        self.publish_alert, self.publish_task_failure = Mock(), Mock()
        self.complete_run_if_ready = Mock()
        self.get_logger = lambda: NS(info=Mock(), error=Mock(), warning=Mock())
        self.get_parameter = lambda _: NS(value=-72.)


def result(**kw):
    return ExecuteTask.Result(ok=True, n_samples=10, n_total=10, n_missed=0,
                              rssi_mean=-60., rssi_median=-60., duration_s=15.,
                              measurement_valid=True, **kw)


def future(res):
    return NS(result=lambda: NS(result=res))


@pytest.mark.parametrize('error', [False, RuntimeError('test DB unavailable')])
def test_store_failure_keeps_patrol_going_and_retry_saves(error):
    m = Manager(saved=error)
    if isinstance(error, Exception):
        m.store.update_cell.side_effect = error
    m.on_task_result('robot4', 'reinspect', future(result()))
    # 순찰은 멈추지 않는다: 작업은 끝나고, 판정 알림 없이 저장만 보류 (10/15 PM)
    assert m.core.tasks['reinspect'].state == TaskState.SUCCEEDED
    assert m.pending_remeasure_results
    codes = [c.kwargs.get('code') for c in m.publish_alert.call_args_list]
    assert 'SAVE_PENDING' in codes and 'CLEARED' not in codes and 'REMAINS' not in codes
    m.store.update_cell.side_effect, m.store.update_cell.return_value = None, True
    m.retry_remeasure_results()
    assert not m.pending_remeasure_results
    assert m.publish_alert.call_args.kwargs['code'] == 'CLEARED'


@pytest.mark.parametrize('nd,nm,valid', [(0,0,True),(1,0,True),(0,15,False)])
def test_unknown_preserves_old_cell_and_does_not_confirm(nd,nm,valid):
    m = Manager()
    res = result()
    res.n_samples, res.n_missed, res.n_total = nd,nm,nd+nm
    res.measurement_valid = valid
    m.on_task_result('robot4', 'reinspect', future(res))
    m.store.update_cell.assert_not_called()
    assert m.core.tasks['reinspect'].state == TaskState.FAILED
    assert not m.confirmed_cells
    assert m.publish_alert.call_args.kwargs['code'] == 'UNCONFIRMED'


def test_mostly_missed_is_not_cleared_by_one_good_detection():
    m = Manager()
    res = result()
    res.n_samples, res.n_missed, res.n_total = 1,29,30
    m.on_task_result('robot4', 'reinspect', future(res))
    assert m.publish_alert.call_args_list[0].kwargs['code'] == 'REMAINS'
    assert m.confirmed_cells == {(1,2)}
    assert m.store.update_cell.call_args.args[3] == 'miss'
    assert m.store.update_cell.call_args.kwargs['n_missed'] == 29
