"""칸 맵을 읽고 쓰는 곳 하나 (ROS 와 무관, pytest 대상). 묶음 시험용 임시 구현 (T3-6).

- cell_map_writer(RSSI)가 쓰고 inspection_manager(관제)가 읽는다
- backend 'file': out_dir/<zone>.json · 'db': 관제 PC 의 PostgreSQL (grid_stats). 관제 API 가 생기면 backend 를 하나 더한다
- 칸 상태는 어디서나 heatmap 의 이름(normal·shadow·miss)이다. DB 값으로 바꾸는 일은 이 파일 안에서만 한다
- DB 접속 정보는 환경변수(기본 SCV_DB_DSN)에서만 읽고 로그·예외에 찍지 않는다
"""

import json
import math
import os
import statistics
from pathlib import Path
from typing import Dict, List, NamedTuple, Optional, Tuple

import yaml

from scv_survey.heatmap import MISS, NORMAL, SHADOW, SHADOW_DBM, MIN_SAMPLES, CellStats, cell_center, cell_status

Cell = Tuple[int, int]
MIN_SAMPLES_OK = MIN_SAMPLES      # 통합 재점검의 최소 전체 표본 수; 미검출도 1회로 센다
TO_DB = {NORMAL: 'normal', SHADOW: 'coverage_suspect', MISS: 'unknown'}
FROM_DB = {v: k for k, v in TO_DB.items()}


class CellRow(NamedTuple):
    state: str              # normal · shadow · miss
    median: Optional[float]
    n: int
    confidence: str         # ok · low
    mean: Optional[float] = None
    n_detected: Optional[int] = None
    n_missed: Optional[int] = None


class MapInfo(NamedTuple):
    origin: Tuple[float, float]
    resolution: float
    width: int              # px
    height: int
    pgm: Path


def read_map_yaml(path) -> MapInfo:
    path = Path(path).expanduser()
    with path.open(encoding='utf-8') as f:
        y = yaml.safe_load(f)
    pgm = path.parent / y['image']
    with pgm.open('rb') as f:
        head = f.read(64).split()       # P5, width, height, maxval
    return MapInfo((float(y['origin'][0]), float(y['origin'][1])), float(y['resolution']),
                   int(head[1]), int(head[2]), pgm)


def free_cells(info: MapInfo, cell: float, free_thresh: float = 0.196) -> List[Tuple[int, int, float, float]]:
    """지도의 흰 픽셀이 하나라도 걸치는 칸 [(ix, iy, cx, cy)]. 관제 웹의 격자 생성과 같은 규칙"""
    raw = info.pgm.read_bytes()
    parts, pos = [], 0
    while len(parts) < 4:               # 머리 4개 토큰 뒤의 공백 한 글자까지 건너뜀
        while raw[pos:pos + 1].isspace():
            pos += 1
        end = pos
        while not raw[end:end + 1].isspace():
            end += 1
        parts.append(raw[pos:end])
        pos = end
    maxval, data = int(parts[3]), raw[pos + 1:]
    step = 2 if maxval > 255 else 1     # testbed_v1.pgm 은 16비트 (65535)
    out = set()
    for r in range(info.height):
        for c in range(info.width):
            k = (r * info.width + c) * step
            v = int.from_bytes(data[k:k + step], 'big') / maxval
            if 1.0 - v < free_thresh:
                x = (c + 0.5) * info.resolution
                y = (info.height - r - 0.5) * info.resolution
                out.add((math.floor(x / cell), math.floor(y / cell)))
    ox, oy = info.origin
    return [(i, j, *cell_center(i, j, ox, oy, cell)) for i, j in sorted(out)]


def rows_from_stats(cells: Dict[Cell, CellStats], origin: Tuple[float, float], cell: float,
                    shadow_dbm: float = SHADOW_DBM, miss_ratio_thresh: float = 0.5) -> List[dict]:
    """SurveyGrid.cells() → 저장할 줄. 통계는 검출된 값만으로 낸다"""
    out = []
    for (i, j), st in sorted(cells.items()):
        if st.n_total == 0:
            continue  # 미측정은 통계 행을 만들지 않는다
        cx, cy = cell_center(i, j, origin[0], origin[1], cell)
        r = st.rssi
        out.append({
            'ix': i, 'iy': j, 'cx': round(cx, 3), 'cy': round(cy, 3),
            'n': st.n_total, 'n_detected': st.n_detected, 'n_missed': st.n_missed,
            'median': st.median, 'mean': statistics.fmean(r) if r else None,
            'var': statistics.pvariance(r) if r else None,
            'min': min(r) if r else None, 'max': max(r) if r else None,
            'state': cell_status(st, shadow_dbm, miss_ratio_thresh),
        })
    return out


def _row(c: dict) -> CellRow:
    return CellRow(c['state'], c['median'], c['n'], 'ok' if c['n'] >= MIN_SAMPLES_OK else 'low',
                   c.get('mean'), c.get('n_detected'), c.get('n_missed'))


class FileStore:
    def __init__(self, map_yaml, cell_size: float = 0.5, out_dir='~/scv_cellmap'):
        self.map = read_map_yaml(map_yaml)
        self.cell = cell_size
        self.dir = Path(out_dir).expanduser()
        self.dir.mkdir(parents=True, exist_ok=True)

    def open_run(self, sector_robots: dict, close_stale: bool = False) -> int:
        counter = self.dir / 'run_counter.txt'
        run_id = int(counter.read_text()) + 1 if counter.exists() else 1
        counter.write_text(str(run_id))
        for p in self.dir.glob('*.json'):       # 지난 회차의 칸 맵이 섞이지 않게
            p.unlink()
        return run_id

    def close_run(self, run_id: int, status: str) -> None:
        pass

    def running_run(self) -> int:
        counter = self.dir / 'run_counter.txt'
        return int(counter.read_text()) if counter.exists() else 0

    def write_cells(self, run_id: int, zone: str, rows: List[dict], ap_ssid: str = '') -> int:
        doc = {'run_id': run_id, 'zone': zone, 'map': {'origin': list(self.map.origin), 'cell': self.cell},
               'ap_ssid': ap_ssid, 'cells': rows}
        (self.dir / f'{zone}.json').write_text(json.dumps(doc, ensure_ascii=False, indent=1))
        return len(rows)

    def read_cells(self, run_id: int) -> Dict[Cell, CellRow]:
        out = {}
        for p in sorted(self.dir.glob('*.json')):
            doc = json.loads(p.read_text())
            if doc['run_id'] != run_id:
                continue
            for c in doc['cells']:
                out[(c['ix'], c['iy'])] = _row(c)
        return out

    def update_cell(self, run_id: int, ix: int, iy: int, state: str, median: Optional[float], n: int,
                    *, mean=None, n_detected=None, n_missed=0) -> bool:
        n_detected = n if n_detected is None else n_detected
        if n != n_detected + n_missed:
            raise ValueError('전체 표본 수와 검출/미검출 수가 다릅니다')
        files = [p for p in sorted(self.dir.glob('*.json'))
                 if json.loads(p.read_text())['run_id'] == run_id]
        for p in files:
            doc = json.loads(p.read_text())
            for c in doc['cells']:
                if (c['ix'], c['iy']) == (ix, iy):
                    c.update(state=state, median=median, mean=mean, n=n, n_detected=n_detected,
                             n_missed=n_missed, var=None, min=None, max=None)
                    p.write_text(json.dumps(doc, ensure_ascii=False, indent=1))
                    return True
        if not files:
            return False
        doc = json.loads(files[0].read_text())  # 미측정이던 칸: 새 줄
        cx, cy = cell_center(ix, iy, *self.map.origin, self.cell)
        doc['cells'].append({'ix': ix, 'iy': iy, 'cx': round(cx, 3), 'cy': round(cy, 3),
                             'n': n, 'n_detected': n_detected, 'n_missed': n_missed,
                             'mean': mean, 'median': median, 'state': state})
        files[0].write_text(json.dumps(doc, ensure_ascii=False, indent=1))
        return True

    def grid_cells(self) -> List[Tuple[int, int, float, float]]:
        return free_cells(self.map, self.cell)


class DbStore:
    """PostgreSQL 직접 (scv_db 01_schema.sql 의 runs · grid_cells · aps · grid_stats). RSSI 확장 스키마(01_schema.sql 재적용)가 필요하다"""

    def __init__(self, map_yaml, cell_size: float = 0.5, dsn_env: str = 'SCV_DB_DSN', ap_ssid: str = ''):
        if not ap_ssid:     # 빈 값이면 DB 에 뜻 없는 AP 줄이 생긴다
            raise RuntimeError('db 백엔드는 ap_ssid 가 필요합니다')
        try:
            import psycopg as driver
        except ImportError:
            import psycopg2 as driver
        self.driver = driver
        self.dsn = os.environ.get(dsn_env, '')
        if not self.dsn:
            raise RuntimeError(f'환경변수 {dsn_env} 가 없습니다')
        self.map = read_map_yaml(map_yaml)
        self.cell = cell_size
        m = self._q('SELECT map_id, origin_x, origin_y, resolution_m, grid_size_m FROM maps WHERE is_active')
        if not m:
            raise RuntimeError('DB 에 활성 지도가 없습니다')
        self.map_id, ox, oy, res, grid = m[0]
        if (abs(ox - self.map.origin[0]) > 1e-3 or abs(oy - self.map.origin[1]) > 1e-3
                or abs(res - self.map.resolution) > 1e-6 or abs(grid - cell_size) > 1e-6):
            raise RuntimeError(f'DB 활성 지도(origin {ox:.3f},{oy:.3f} res {res} 격자 {grid})가 '
                               f'map_yaml·cell_size({cell_size})와 다릅니다')
        # 쓰는 쪽(writer)과 고치는 쪽(관제)이 같은 AP 줄을 보도록 여기서 한 번 정한다
        self._q('INSERT INTO aps (map_id, ssid, note, is_active) VALUES (%s, %s, %s, true) '
                'ON CONFLICT (map_id, ssid) DO NOTHING', (self.map_id, ap_ssid, 'cell_map_writer 자동 등록 (묶음 시험)'))
        self.ap_id = self._q('SELECT ap_id FROM aps WHERE map_id = %s AND ssid = %s', (self.map_id, ap_ssid))[0][0]

    def _q(self, sql: str, params=(), many=None):
        """한 번 접속해 실행하고 닫는다. SELECT·RETURNING 이면 줄 목록. 오류에 접속 정보를 싣지 않는다"""
        try:
            conn = self.driver.connect(self.dsn)
        except self.driver.Error:
            raise RuntimeError('DB 에 접속하지 못했습니다') from None
        try:
            cur = conn.cursor()
            if many is not None:
                cur.executemany(sql, many)
            else:
                cur.execute(sql, params)
            rows = cur.fetchall() if cur.description and many is None else cur.rowcount
            conn.commit()
            return rows
        except self.driver.Error as e:
            raise RuntimeError(f'DB 오류: {getattr(e.diag, "message_primary", None) or type(e).__name__}') from None
        finally:
            conn.close()

    def open_run(self, sector_robots: dict, close_stale: bool = False) -> int:
        stale = self._q("SELECT run_id FROM runs WHERE status = 'running'")
        if stale and not close_stale:
            raise RuntimeError(f'running 회차 run_id={stale[0][0]} 가 남아 있습니다 (close_stale_run:=true 로 닫고 시작)')
        if stale:
            self.close_run(stale[0][0], 'aborted')
        return self._q("INSERT INTO runs (map_id, run_type, sector_robots) VALUES (%s, 'patrol', %s::jsonb) "
                       'RETURNING run_id', (self.map_id, json.dumps(sector_robots)))[0][0]

    def close_stale_runs(self) -> list:
        """관제를 다시 띄웠을 때 DB 에 running 으로 남은 회차 — manager 는 기억이 없어 이어 갈 수 없으므로 aborted 로 닫음"""
        stale = [r[0] for r in self._q("SELECT run_id FROM runs WHERE status = 'running'")]
        for run_id in stale:
            self.close_run(run_id, 'aborted')
        return stale

    def close_run(self, run_id: int, status: str) -> None:
        # 남은 작업 줄은 취소로 닫는다 (tasks 는 inspection_manager 가 sync_tasks 로 씀, 10/14 ROS 통합)
        self._q("UPDATE tasks SET status = 'CANCELED', ended_at = now() WHERE run_id = %s AND status IN ('PENDING', 'ACTIVE')",
                (run_id,))
        self._q("UPDATE runs SET status = %s, ended_at = now() WHERE run_id = %s AND status = 'running'",
                (status, run_id))

    def running_run(self) -> int:
        r = self._q("SELECT run_id FROM runs WHERE status = 'running'")
        return r[0][0] if r else 0

    def sync_tasks(self, rows: List[dict]) -> None:
        """작업 줄 upsert (10/14 ROS 통합 — 작업의 원본은 inspection_manager, 웹은 이 줄을 읽기만 한다).
        rows: task_id · run_id · robot_id · type · target_ref · waypoints · status · progress · reason · result_ok.
        로봇마다 ACTIVE 는 하나(tasks_one_active_per_robot)라 ACTIVE 가 아닌 줄을 먼저 쓴다"""
        rows = sorted(rows, key=lambda r: r['status'] == 'ACTIVE')
        self._q("INSERT INTO tasks (task_id, run_id, robot_id, type, target_ref, waypoints, status, progress, reason, "
                "result_ok, assigned_at, ended_at) VALUES (%(task_id)s, %(run_id)s, %(robot_id)s, %(type)s, %(target_ref)s, "
                "%(waypoints)s::jsonb, %(status)s, %(progress)s, %(reason)s, %(result_ok)s, "
                "CASE WHEN %(started)s THEN now() END, CASE WHEN %(ended)s THEN now() END) "
                "ON CONFLICT (task_id) DO UPDATE SET robot_id = EXCLUDED.robot_id, status = EXCLUDED.status, "
                "progress = EXCLUDED.progress, reason = EXCLUDED.reason, result_ok = EXCLUDED.result_ok, "
                "assigned_at = COALESCE(tasks.assigned_at, EXCLUDED.assigned_at), "
                "ended_at = CASE WHEN EXCLUDED.status IN ('DONE', 'FAILED', 'CANCELED') "
                "THEN COALESCE(tasks.ended_at, now()) END",
                many=[dict(r, waypoints=json.dumps(r['waypoints']), started=r['status'] != 'PENDING',
                           ended=r['status'] in ('DONE', 'FAILED', 'CANCELED')) for r in rows])

    def _cell_ids(self) -> Dict[Cell, int]:
        return {(i, j): cid for i, j, cid in
                self._q('SELECT ix, iy, cell_id FROM grid_cells WHERE map_id = %s', (self.map_id,))}

    def write_cells(self, run_id: int, zone: str, rows: List[dict], ap_ssid: str = '') -> int:
        ap, ids = self.ap_id, self._cell_ids()
        vals = [(run_id, ids[(c['ix'], c['iy'])], ap, c['n'], c['mean'], c['median'], c['var'], c['min'], c['max'],
                 'ok' if c['n'] >= MIN_SAMPLES_OK else 'low', TO_DB[c['state']],
                 c['n_detected'], c['n_missed'])
                for c in rows if (c['ix'], c['iy']) in ids]     # grid_cells 에 없는 칸(벽)은 건너뜀
        if vals:
            self._q('INSERT INTO grid_stats (run_id, cell_id, ap_id, n, mean, median, var, min, max, confidence, state, n_detected, n_missed) '
                    'VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s) '
                    'ON CONFLICT (run_id, cell_id, ap_id) DO UPDATE SET n = EXCLUDED.n, mean = EXCLUDED.mean, '
                    'median = EXCLUDED.median, var = EXCLUDED.var, min = EXCLUDED.min, max = EXCLUDED.max, '
                    'confidence = EXCLUDED.confidence, state = EXCLUDED.state, n_detected = EXCLUDED.n_detected, '
                    'n_missed = EXCLUDED.n_missed, computed_at = now()', many=vals)
        return len(vals)

    def read_cells(self, run_id: int) -> Dict[Cell, CellRow]:
        rows = self._q('SELECT g.ix, g.iy, s.state, s.median, s.n, s.confidence, s.mean, s.n_detected, s.n_missed FROM grid_stats s '
                       'JOIN grid_cells g USING (cell_id) WHERE s.run_id = %s AND s.ap_id = %s', (run_id, self.ap_id))
        return {(i, j): CellRow(FROM_DB[st], med, n, conf, mean, nd, nm)
                for i, j, st, med, n, conf, mean, nd, nm in rows}

    def update_cell(self, run_id: int, ix: int, iy: int, state: str, median: Optional[float], n: int,
                    *, mean=None, n_detected=None, n_missed=0) -> bool:
        cid, ap = self._cell_ids().get((ix, iy)), self.ap_id
        if cid is None:
            return False
        n_detected = n if n_detected is None else n_detected
        if n != n_detected + n_missed:
            raise ValueError('전체 표본 수와 검출/미검출 수가 다릅니다')
        self._q('INSERT INTO grid_stats (run_id, cell_id, ap_id, n, mean, median, confidence, state, n_detected, n_missed) '
                'VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s) '
                'ON CONFLICT (run_id, cell_id, ap_id) DO UPDATE SET '
                'n = EXCLUDED.n, mean = EXCLUDED.mean, median = EXCLUDED.median, '
                'n_detected = EXCLUDED.n_detected, n_missed = EXCLUDED.n_missed, '
                'confidence = EXCLUDED.confidence, state = EXCLUDED.state, '
                'var = NULL, min = NULL, max = NULL, interpolated = false, baseline_delta = NULL, computed_at = now()',
                (run_id, cid, ap, n, mean, median, 'ok' if n >= MIN_SAMPLES_OK else 'low',
                 TO_DB[state], n_detected, n_missed))
        return True

    def grid_cells(self) -> List[Tuple[int, int, float, float]]:
        return self._q('SELECT ix, iy, center_x, center_y FROM grid_cells WHERE map_id = %s', (self.map_id,))


def open_store(backend: str, map_yaml, cell_size: float = 0.5, out_dir='~/scv_cellmap', dsn_env: str = 'SCV_DB_DSN',
               ap_ssid: str = ''):
    if backend == 'db':
        return DbStore(map_yaml, cell_size, dsn_env, ap_ssid)
    return FileStore(map_yaml, cell_size, out_dir)


def in_polygon(x: float, y: float, polygon) -> bool:
    inside = False
    for (x1, y1), (x2, y2) in zip(polygon, list(polygon[1:]) + [polygon[0]]):
        if (y1 <= y < y2) or (y2 <= y < y1):
            if x < x1 + (y - y1) / (y2 - y1) * (x2 - x1):
                inside = not inside
    return inside
