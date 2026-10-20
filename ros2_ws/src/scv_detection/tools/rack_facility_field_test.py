#!/usr/bin/env python3
"""랙 점검(IF-05) 현장 시험 — 순찰 없이 랙 관측 자리로 가서 이 저장소 사본의 비전 코드로 한 번 판정하고 사진을 남긴 뒤 도킹.

떠 있는 robot_pc.launch(Nav2 · mission_executor · 비전 detection_alert)는 그대로 둔다. 이 스크립트 한 프로세스가
  1. 시험용 비전 노드 detection_alert_test 를 띄운다 (이 파일이 있는 저장소의 scv_detection 코드).
     inspect_facility 서비스 · 결과 · 순간 불량 · 미리보기 토픽은 rack_test/… 로 바꿔 관제 DB · mission_executor 와 섞이지 않게 한다.
  2. dock_status 를 20 s 안에 받지 못하면 움직이지 않고 끝냄. 도크 위면 undock(도크를 벗어났는지 확인)
     → AMCL 초기 자세(HOME) → 좌우 settle (mission_executor 와 같은 함수)
  3. facilities.yaml 의 랙 view 로 Nav2 이동 → 2 s 정지 → inspect_facility 와 같은 코드로 판정
  4. 판정 창 동안의 결과 화면(주석 사진) · 판정 JSON 을 --out 폴더에 저장
  5. HOME 앞(DOCK_BACKOFF)으로 가서 dock, dock_status 로 확인 (안 되면 한 번 더). 중간에 오류가 나도 한 번은 도크로 돌아가 봄
  시험 중에는 관제에서 순찰을 시작하지 않는다 (mission_executor 가 작업을 받으면 두 주체가 로봇을 움직임).

    cd <저장소 뿌리> && python3 ros2_ws/src/scv_detection/tools/rack_facility_field_test.py --ns robot4
    (--facility 기본: robot4 → RCK-01, robot3 → RCK-02 · --no-move 는 지금 자리에서 판정만 · Ctrl+C 는 이동을 멈추고 끝냄, 도킹은 하지 않음)
    (Ctrl+C 는 Nav2 이동만 취소한다. undock · dock 동작 중이면 그 동작은 끝까지 간다)
"""
import argparse
import importlib.util
import json
import math
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]                     # 저장소 뿌리
SRC = ROOT / 'ros2_ws' / 'src'
sys.path.insert(0, str(SRC / 'scv_detection'))                  # 설치본이 아니라 이 사본의 비전 코드

import rclpy                                                     # noqa: E402
import yaml                                                      # noqa: E402
from rclpy.executors import MultiThreadedExecutor                # noqa: E402
from rclpy.parameter import Parameter                            # noqa: E402
from rclpy.qos import qos_profile_sensor_data                     # noqa: E402
from rclpy.signals import SignalHandlerOptions                   # noqa: E402
from irobot_create_msgs.msg import DockStatus                    # noqa: E402
from nav2_simple_commander.robot_navigator import TaskResult     # noqa: E402
from scv_msgs.msg import InspectionResult                        # noqa: E402
from scv_msgs.srv import InspectFacility                         # noqa: E402
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator  # noqa: E402
from scv_detection.scv_detection_manager import ScvDetectionManager       # noqa: E402

spec = importlib.util.spec_from_file_location('mission_executor', SRC / 'scv_nav' / 'scripts' / 'mission_executor.py')
me = importlib.util.module_from_spec(spec)
spec.loader.exec_module(me)                                      # localize · settle · HOME · DOCK_BACKOFF 재사용 (main 은 안 돎)

VERDICT_NAME = {InspectionResult.NORMAL: 'NORMAL', InspectionResult.ABNORMAL: 'ABNORMAL',
                InspectionResult.UNCONFIRMED: 'UNCONFIRMED'}


def log(text):
    print(f'[rack_test {time.strftime("%H:%M:%S")}] {text}', flush=True)


def wait_nav(nav, what, timeout):
    end = time.time() + timeout
    while not nav.isTaskComplete():
        if time.time() > end:
            nav.cancelTask()
            raise RuntimeError(f'{what}: {timeout:.0f} s 안에 끝나지 않아 취소')
        time.sleep(0.1)
    if nav.getResult() != TaskResult.SUCCEEDED:
        raise RuntimeError(f'{what}: {nav.getResult()}')


class DockWatch:
    """dock_status 를 따로 받아 '도크 위인가' 를 시간 제한을 두고 확인한다.

    TurtleBot4Navigator.getDockedStatus() 는 값이 올 때까지 끝없이 기다리고 옛 값을 줄 수 있어 쓰지 않는다.
    wait(True/False, 초) 는 그 상태가 되면 True, 시간이 지나면 False 를 돌려준다.
    """

    def __init__(self, node):
        self.docked = None
        node.create_subscription(DockStatus, 'dock_status', self._on, qos_profile_sensor_data)

    def _on(self, msg):
        self.docked = msg.is_docked

    def wait_known(self, timeout):
        """dock_status 가 한 번이라도 오면 True, timeout 초 동안 안 오면 False."""
        end = time.time() + timeout
        while self.docked is None and time.time() < end:
            time.sleep(0.1)
        return self.docked is not None

    def wait(self, want, timeout):
        end = time.time() + timeout
        while time.time() < end:
            if self.docked is want:
                return True
            time.sleep(0.1)
        return False


def go_dock(nav, home, dock):
    """mission_executor.go_home 과 같이 HOME 에서 DOCK_BACKOFF 만큼 앞으로 간 뒤 dock 하고,
    dock_status 가 15 s 안에 도크 위가 되는지 확인한다 (안 되면 한 번 더 dock). 끝내 안 되면 RuntimeError."""
    p, q = home.pose.position, home.pose.orientation
    yaw = 2 * math.atan2(q.z, q.w)
    log('도크 앞으로')
    nav.goToPose(nav.getPoseStamped([p.x + me.DOCK_BACKOFF * math.cos(yaw), p.y + me.DOCK_BACKOFF * math.sin(yaw)],
                                    math.degrees(yaw)))
    wait_nav(nav, '도크 앞 이동', 120)
    for attempt in (1, 2):
        log(f'dock ({attempt}/2)')
        nav.dock()
        if dock.wait(True, 15.0):
            log('도킹 완료 (dock_status 도크 위)')
            return
    raise RuntimeError('도킹 확인 안 됨 (dock_status 가 15 s 안에 도크 위가 아님, 2번)')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ns', default='robot4', choices=['robot3', 'robot4'])
    ap.add_argument('--facility', default='')
    ap.add_argument('--out', default='')
    ap.add_argument('--no-move', action='store_true')
    a = ap.parse_args()
    facility_id = a.facility or {'robot4': 'RCK-01', 'robot3': 'RCK-02'}[a.ns]
    facs = yaml.safe_load((SRC / 'scv_bringup' / 'config' / 'facilities.yaml').read_text())['facilities']
    fac = next(f for f in facs if f['id'] == facility_id)
    out = Path(a.out or f'~/rack_test_{time.strftime("%m%d_%H%M%S")}').expanduser()
    out.mkdir(parents=True, exist_ok=True)
    log(f'{a.ns} {facility_id} view={fac["view"]} → 결과 폴더 {out}')

    # Ctrl+C 를 rclpy 가 먼저 가로채면 context 가 내려가 cancelTask 가 안 나갈 수 있음 → 직접 받음 (mission_executor 와 같음)
    rclpy.init(args=['--ros-args', '-r', f'__ns:=/{a.ns}', '-r', '__node:=rack_test_nav'],
               signal_handler_options=SignalHandlerOptions.NO)
    nav = TurtleBot4Navigator()
    vision = ScvDetectionManager(
        namespace=f'/{a.ns}', use_global_arguments=False,
        cli_args=['--ros-args', '-r', '__node:=detection_alert_test',
                  '-r', 'inspect_facility:=rack_test/inspect_facility',
                  '-r', 'inspect/result:=rack_test/result',
                  '-r', 'inspect/defects:=rack_test/defects',
                  '-r', 'fire_extinguisher/detections/compressed:=rack_test/detections/compressed'],
        parameter_overrides=[Parameter('show', value=False), Parameter('device', value='cuda:0'),
                             Parameter('weights', value=str(SRC / 'scv_detection' / 'models' / 'best.pt')),
                             Parameter('rack_reference_path', value=''),
                             Parameter('shutter_enabled', value=False)])   # 랙 시험엔 Depth · TF 구독 불필요 (Wi-Fi 부담)
    watch_node = rclpy.create_node('rack_test_dock', namespace=f'/{a.ns}', use_global_arguments=False)
    dock = DockWatch(watch_node)
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(vision)
    ex.add_node(watch_node)
    threading.Thread(target=ex.spin, daemon=True).start()
    home = nav.getPoseStamped(me.HOME[f'/{a.ns}'][:2], me.HOME[f'/{a.ns}'][2])
    moved = False
    try:
        if not a.no_move:
            if not dock.wait_known(20.0):
                raise RuntimeError('dock_status 가 20 s 동안 오지 않음 (Pi · create3 확인) — 움직이지 않고 끝냄')
            if dock.docked:
                log('undock')
                nav.undock()
                moved = True
                if not dock.wait(False, 10.0):
                    raise RuntimeError('undock 뒤에도 도크 위로 나옴 — undock 실패')
                me.localize(nav, home)
                me.settle(nav)
            v = fac['view']
            log(f'Nav2 → {facility_id} ({v["x"]:.3f}, {v["y"]:.3f}, {math.degrees(v["yaw"]):.1f}°)')
            nav.goToPose(nav.getPoseStamped([v['x'], v['y']], math.degrees(v['yaw'])))
            moved = True
            wait_nav(nav, '관측 자리 이동', 120)
            time.sleep(2.0)                                       # mission_executor 와 같이 멈춘 뒤 판정

        end = time.time() + 15                                    # 카메라 영상 (도크 위에서는 꺼져 있을 수 있음)
        while (vision.last_frame_time is None or time.monotonic() - vision.last_frame_time > 1.0) and time.time() < end:
            time.sleep(0.2)
        log('카메라 영상 ' + ('받음' if vision.last_frame_time else '없음 — 판정은 UNCONFIRMED 가 됨'))

        frames, outcomes, stop = [], [], threading.Event()

        def sample():                                             # 판정 창 동안 결과 화면 · 랙 판정을 0.5 s 마다
            last = None
            while not stop.is_set():
                jpeg = vision._last_jpeg
                if jpeg is not None and jpeg is not last and vision._inspection_mode == 'yolo':
                    last = jpeg
                    path = out / f'frame_{len(frames):02d}.jpg'
                    path.write_bytes(bytes(jpeg))
                    frames.append(path.name)
                    res = vision._inspection_result or {}
                    outcomes.append({'frame': path.name, 'rack_loading': res.get('rack_loading'),
                                     'message': res.get('message')})
                time.sleep(0.5)

        sampler = threading.Thread(target=sample, daemon=True)
        sampler.start()
        req = InspectFacility.Request(task_id='rack_field_test', run_id=0, facility_id=facility_id,
                                      kind=InspectionResult.KIND_RACK)
        log('inspect_facility (수정 코드: 랙 → yolo 모드 rack_loading)')
        resp = vision._handle_inspect_facility(req, InspectFacility.Response())
        stop.set()
        sampler.join()
        verdict = VERDICT_NAME.get(resp.verdict, str(resp.verdict))
        if vision._last_jpeg is not None:
            (out / 'result_evidence.jpg').write_bytes(bytes(vision._last_jpeg))
        (out / 'result.json').write_text(json.dumps({
            'robot': a.ns, 'facility_id': facility_id, 'view': fac['view'], 'verdict': verdict, 'detail': resp.detail,
            'frames': frames, 'per_frame': outcomes, 'time': time.strftime('%Y-%m-%d %H:%M:%S')},
            ensure_ascii=False, indent=2, default=str))
        log(f'판정 {verdict} · {resp.detail[:160]}')
        log(f'저장: {out} (frame_*.jpg {len(frames)}장, result.json)')

        if moved:
            go_dock(nav, home, dock)
    except KeyboardInterrupt:
        nav.cancelTask()
        log('Ctrl+C — 이동을 멈춤 (도킹하지 않음)')
    except Exception as error:                                    # noqa: BLE001
        nav.cancelTask()
        log(f'중단: {error}')
        if moved:                                                 # 판정이 실패해도 한 번은 도크로 돌아가 봄
            try:
                go_dock(nav, home, dock)
            except Exception as again:                            # noqa: BLE001
                nav.cancelTask()
                log(f'도킹 못 함: {again} — 로봇 위치를 확인해 주세요')
    finally:
        vision._stop_event.set()
        ex.shutdown()
        vision.destroy_node()
        watch_node.destroy_node()
        nav.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
