"""cell_map_writer (관제 PC, 묶음 시험용 임시 · T3-6): {ns}/survey/sample 을 칸으로 모아 칸 맵 저장소에 쓴다.

- network_map_engine 과 같은 SurveyGrid·cell_status 를 써서 판정 기준이 같다 (히트맵·RViz 는 network_map_engine 이 맡는다)
- ~/commit_<zone> (Trigger): 그 zone polygon 안의 칸을 저장소에 쓴다 (관제가 그 섹터 1차 측정이 끝났을 때 부른다).
  확정한 zone 의 이후 샘플은 버린다 — 재측정으로 서 있는 동안의 샘플이 관제가 쓴 재측정 값을 덮지 않게
- ~/reset (Trigger): 샘플과 확정 표시를 비운다 (관제가 회차를 시작할 때 부른다)
"""

from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger

from scv_msgs.msg import NetSample
from scv_survey.cellmap_store import in_polygon, open_store, rows_from_stats
from scv_survey.heatmap import MISS, SHADOW, SHADOW_DBM, Sample, SurveyGrid
from scv_survey.manual_tagger import SAMPLE_QOS


class CellMapWriter(Node):
    def __init__(self):
        super().__init__('cell_map_writer')
        bringup = Path(get_package_share_directory('scv_bringup'))
        topics = self.declare_parameter('sample_topics', ['/robot4/survey/sample', '/robot3/survey/sample']).value
        cell = self.declare_parameter('cell_size', 0.5).value
        self.shadow_dbm = float(self.declare_parameter('shadow_dbm', float(SHADOW_DBM)).value)
        self.miss_thresh = self.declare_parameter('miss_ratio_thresh', 0.5).value
        self.ap_ssid = self.declare_parameter('ap_ssid', '').value
        self.store = open_store(
            self.declare_parameter('backend', 'file').value,
            self.declare_parameter('map_yaml', str(bringup / 'maps' / 'testbed_v1.yaml')).value, cell,
            self.declare_parameter('out_dir', '~/scv_cellmap').value,
            self.declare_parameter('db_dsn_env', 'SCV_DB_DSN').value, self.ap_ssid)
        zones_file = self.declare_parameter('zones_file', str(bringup / 'config' / 'zones.yaml')).value
        with open(zones_file, encoding='utf-8') as f:
            self.zones = {str(z['id']): z['polygon'] for z in yaml.safe_load(f)['zones']}

        self.grid = SurveyGrid(cell_size=cell, origin=self.store.map.origin)
        self.committed = set()
        for zone in self.zones:
            self.create_service(Trigger, f'~/commit_{zone}', lambda req, res, z=zone: self.on_commit(z, res))
        self.create_service(Trigger, '~/reset', self.on_reset)
        for t in topics:
            self.create_subscription(NetSample, t, self.on_sample, SAMPLE_QOS)
        self.get_logger().info(f'구독 {list(topics)}, 칸 {cell} m, zone {list(self.zones)}, '
                               f'저장소 {type(self.store).__name__}')

    def zone_of(self, x: float, y: float) -> str:
        return next((z for z, poly in self.zones.items() if in_polygon(x, y, poly)), '')

    def on_sample(self, m: NetSample):
        zone = self.zone_of(m.x, m.y)
        if zone in self.committed:
            self.get_logger().info(f'{zone} 은 확정됨 — 이후 샘플은 버림', once=True)
            return
        self.grid.add(Sample(m.robot_id, m.seq, m.x, m.y, m.pose_ok, m.detected, m.rssi_dbm))

    def on_commit(self, zone: str, res):
        rows = [r for r in rows_from_stats(self.grid.cells(), self.grid.origin, self.grid.cell_size,
                                           self.shadow_dbm, self.miss_thresh)
                if self.zone_of(r['cx'], r['cy']) == zone]
        try:
            run_id = self.store.running_run()
            n = self.store.write_cells(run_id, zone, rows, self.ap_ssid)
        except RuntimeError as e:
            res.success, res.message = False, str(e)
            self.get_logger().error(res.message)
            return res
        self.committed.add(zone)
        count = lambda s: sum(1 for r in rows if r['state'] == s)
        res.success = True
        res.message = (f'{zone}: {n}칸 (음영 {count(SHADOW)} · 미검출 {count(MISS)} · '
                       f'low {sum(1 for r in rows if r["n"] < 3)}), run_id={run_id}')
        self.get_logger().info(res.message)
        return res

    def on_reset(self, req, res):
        self.grid.clear()
        self.committed.clear()
        res.success, res.message = True, '샘플과 확정 표시를 비움'
        self.get_logger().warn(res.message)
        return res


def main(args=None):
    rclpy.init(args=args)
    node = CellMapWriter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
