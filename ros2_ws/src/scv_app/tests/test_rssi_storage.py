"""격리 DB 전용: SCV_RSSI_TEST_DSN을 명시해야 실행. 운영 DB를 생성/초기화하지 않는다."""
import os
import sys
from datetime import datetime, timezone
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


@pytest.fixture
def app(monkeypatch):
    dsn = os.environ.get('SCV_RSSI_TEST_DSN')
    if not dsn:
        pytest.skip('격리 DB의 SCV_RSSI_TEST_DSN 필요')
    from psycopg_pool import ConnectionPool
    from psycopg.rows import dict_row
    import server_page as sp
    pool = ConnectionPool(dsn, open=True, kwargs={'row_factory': dict_row})
    monkeypatch.setattr(sp, 'pool', pool)
    sp.GRID_CACHE.clear()
    sp.LIVE_CELLS.clear()
    with pool.connection() as c:
        # 시험 fixture는 고유 지도·종료 회차로 만들며 기존 행을 지우지 않는다.
        import uuid
        key = str(uuid.uuid4())
        c.execute("INSERT INTO robots(robot_id) VALUES ('robot4') ON CONFLICT DO NOTHING")
        mid = c.execute("INSERT INTO maps(name,robot_map_path,image_key,resolution_m,origin_x,origin_y,grid_size_m) "
                        "VALUES (%s,%s,'test',.05,0,0,.5) RETURNING map_id", (key,key)).fetchone()['map_id']
        ap = c.execute("INSERT INTO aps(map_id,ssid) VALUES (%s,'test-ap') RETURNING ap_id", (mid,)).fetchone()['ap_id']
        cid = c.execute("INSERT INTO grid_cells(map_id,ix,iy,center_x,center_y) VALUES (%s,0,0,.25,.25) RETURNING cell_id", (mid,)).fetchone()['cell_id']
        rid = c.execute("INSERT INTO runs(map_id,run_type,sector_robots,status,ended_at) "
                        "VALUES (%s,'demo','{\"1\":\"robot4\"}','done',now()) RETURNING run_id", (mid,)).fetchone()['run_id']
    yield sp, rid, ap, cid
    pool.close()


def test_missed_sample_survives_and_does_not_color_as_zero(app):
    sp, rid, ap, cid = app
    rows = [dict(seq=i, stamp=datetime.now(timezone.utc), x=.25, y=.25, yaw=0,
                 pose_ok=True, ssid='test-ap', detected=i == 0, rssi_dbm=-60 if i == 0 else None)
            for i in range(4)]
    assert sp.on_samples('robot4', rid, rows)['stored'] == 4
    assert sp.on_samples('robot4', rid, rows)['stored'] == 0
    with sp.pool.connection() as c:
        r = c.execute('SELECT count(*) AS n, count(*) FILTER(WHERE NOT detected) AS missed, '
                      'avg(rssi_dbm) AS mean FROM net_samples WHERE run_id=%s', (rid,)).fetchone()
        assert (r['n'], r['missed'], float(r['mean'])) == (4, 3, -60)
        assert sp.live_cells(c, rid)[cid][ap][:3] == [1, -60, 3600]
    # DB 복원 후가 아닌 실시간 캐시 갱신 경로도 미검출을 RSSI로 누적하지 않는다.
    assert sp.on_samples('robot4', rid, [rows[1] | {'seq': 4}])['stored'] == 1
    assert sp.LIVE_CELLS[rid][cid][ap][0] == 1


def test_all_missed_samples_have_no_numeric_heatmap(app):
    sp, rid, _, _ = app
    sample = dict(seq=1, stamp=datetime.now(timezone.utc), x=.25, y=.25, yaw=0,
                  pose_ok=True, ssid='test-ap', detected=False, rssi_dbm=None)
    assert sp.on_samples('robot4', rid, [sample])['stored'] == 1
    with sp.pool.connection() as c:
        assert sp.live_cells(c, rid) == {}


def test_db_cell_summary_replaces_old_values_and_uses_total_confidence(app):
    from scv_survey.cellmap_store import DbStore, rows_from_stats
    from scv_survey.heatmap import CellStats
    import psycopg
    sp, rid, ap, cid = app
    db = DbStore.__new__(DbStore)
    db.driver, db.dsn, db.ap_id = psycopg, os.environ['SCV_RSSI_TEST_DSN'], ap
    with sp.pool.connection() as c:
        db.map_id = c.execute('SELECT map_id FROM runs WHERE run_id=%s', (rid,)).fetchone()['map_id']
    assert db.write_cells(rid, 'Z1', rows_from_stats({(0,0):CellStats([-90,-80,-70])}, (0,0), .5)) == 1
    assert db.read_cells(rid)[(0,0)].mean == -80
    assert db.update_cell(rid, 0,0,'miss',None,15,mean=None,n_detected=0,n_missed=15)
    row = db.read_cells(rid)[(0,0)]
    assert (row.n,row.n_detected,row.n_missed,row.mean,row.median,row.confidence) == (15,0,15,None,None,'ok')
    with sp.pool.connection() as c:
        r = c.execute('SELECT var,min,max FROM grid_stats WHERE run_id=%s', (rid,)).fetchone()
        assert all(v is None for v in r.values())
    assert db.update_cell(rid,0,0,'miss',None,0)
    assert db.read_cells(rid)[(0,0)].confidence == 'low'
    assert not db.update_cell(rid,99,99,'normal',-60,3)


def test_ros_bridge_does_not_drop_missed_scan():
    # ROS가 없는 환경에서도 실제 수신 콜백을 시험한다.
    import ast
    import threading
    from types import SimpleNamespace as NS
    path = Path(__file__).parents[1] / 'ros_bridge.py'
    cls = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef) and n.name == 'Bridge')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'on_sample')
    scope = {'NetSample': object, 'iso': lambda x: x}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[])), str(path), 'exec'), scope)
    bridge = NS(docked={}, lock=threading.Lock(), samples={}, task={'robot4':'test-task'})
    msg = NS(seq=1, stamp='test-time', x=.25, y=.25, yaw=0, pose_ok=True, detected=False, rssi_dbm=0)
    scope['on_sample'](bridge, 'robot4', msg)
    assert bridge.samples['robot4'][0]['detected'] is False
    assert bridge.samples['robot4'][0]['rssi_dbm'] is None
    bridge.docked['robot4'] = True
    scope['on_sample'](bridge, 'robot4', msg)
    assert len(bridge.samples['robot4']) == 1
