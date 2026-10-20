#!/usr/bin/env python3
"""
SCV 가상 로봇 robot3 (3-3차) — TurtleBot4 robot3 + 제어 PC의 AMCL · Nav2 대신 ROS 2 토픽 · 액션을 내보내는 시험용 노드

  구현방안 4.1절의 로봇 통신과, 실제로는 제어 PC에서 도는 표준 노드(AMCL · Nav2)의 이름을 흉내 냅니다.
  client page가 쓰는 것만 구현합니다(필요할 때마다 누적).
    토픽 (이름공간 /robot3) — QoS: 카메라만 sensor_data(best effort), 나머지는 reliable · depth 10
      battery_state                    sensor_msgs/BatteryState                1 Hz   배터리 (도킹 중 충전, 주행 중 방전)
      dock_status                      irobot_create_msgs/DockStatus           1 Hz   도킹 상태 (패키지가 없으면 생략)
      odom                             nav_msgs/Odometry                       10 Hz  오도메트리 (도크 = odom 원점, initialpose 전에는 0)
      amcl_pose                        geometry_msgs/PoseWithCovarianceStamped 2 Hz   지도(map) 좌표 위치 — 실제로는 제어 PC AMCL (initialpose 뒤부터)
      ip                               std_msgs/String                         1 Hz   로봇 IP
      oakd/rgb/image_raw/compressed    sensor_msgs/CompressedImage             10 Hz  카메라 JPEG (--camera-hz, 0이면 없음) — 가상 세계면 순찰 영상 · 판정 사진, 아니면 합성 화면
      rssi                             std_msgs/String (JSON)                  1 Hz   [가상 세계] rssi_scanner 대신 — truth AP 모델 · 지도 벽 · 음영 구역으로 계산
                                                                                      (--rssi-hz, truth.ap_models가 있을 때만 · initialpose 뒤부터)
                                       {"stamp": UTC ISO(ms, Z), "scans": [{"ssid", "rssi_dbm"(정수)}]}
      virtual/detect_result            std_msgs/String (JSON)                  요청마다 [가상 세계] 가상 판정 결과 — YOLO 대신 truth 정답 (시험 전용)
                                       {"request_id", "facility_id", "frontal", "occluded", "detections": [{class, x, y, conf, bbox, state[, gauge]}]}
    구독
      initialpose                      geometry_msgs/PoseWithCovarianceStamped         시작 위치 — client가 관제 PC의 도크 접근 자세로 보냄
      virtual/detect_request           std_msgs/String (JSON)                          [가상 세계] client detection_alert의 가상 판정 요청 (시험 전용)
                                       {"request_id", "facility_id", "facility_type", "has_gauge", "expected_x", "expected_y"}
    액션 (3-3차)
      navigate_through_poses           nav2_msgs/NavigateThroughPoses                  Nav2 대신 — 점을 차례로 제자리 회전 + 직선 이동 (장애물 무시,
                                                                                       방향은 마지막 점만 맞춤, 피드백은 점에 도착할 때마다)
      undock · dock                    irobot_create_msgs/Undock · Dock                도크에서 25 cm 뒤로 나오기 · 도크 접근 자세 1 m 안에서 도킹

  위치: 켜지면 도크에 서 있지만(충전 중) 지도 위 위치는 모름 → amcl_pose를 내보내지 않음.
        initialpose를 받으면 그 자리를 위치로 삼고 amcl_pose를 내보냄 (실제 AMCL 초기 위치 지정과 같음).
        도킹 중(또는 처음)이면 그 자리가 도크 · odom 원점도 됨. 주행(액션 실행) 중에 온 initialpose는 무시
  가상 세계(--world, 기본 virtual_world/testbed/world.yaml): world.yaml의 truth 구역 — 로봇이 측정 · 판정으로 알아내야 하는
             실제 세계의 값(AP 신호 원점 · 세기 모델 · 지도 벽 감쇠, 음영 구역, 설비 실제 상태 · 판정 사진, 순찰 영상). DB에는 넣지 않음
             --no-world면 가상 세계 없이 로봇만 — rssi · 가상 판정이 없고(client 판정은 시간 초과 → 미확인), 카메라는 합성 화면.
             관제 PC 등록(지도 · AP · 설비 · 도크)은 virtual_world/register_world.py

  실행 (ROS 2 Jazzy 환경):
    source /opt/ros/jazzy/setup.bash
    python3 robot3_virtual.py                                       # = --world virtual_world/testbed/world.yaml
    python3 robot3_virtual.py --speed 1.5 --rssi-hz 6               # 빠른 시험
    python3 robot3_virtual.py --robot-id robot4                     # 섹터 2용 robot4
    python3 robot3_virtual.py --world 다른세계/world.yaml            # 다른 가상 세계
    python3 robot3_virtual.py --no-world                            # 가상 세계 없이 로봇만
    python3 robot3_virtual.py --camera-hz 5           # 카메라 주기 변경 (0이면 카메라 끔)
"""
from __future__ import annotations

import argparse
import array
import io
import json
import math
import random
import socket
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, CompressedImage
from std_msgs.msg import String

try:                                   # TurtleBot4(Create 3) 메시지 — 없으면 dock_status · dock · undock 생략
    from irobot_create_msgs.msg import DockStatus
    from irobot_create_msgs.action import Dock, Undock
except ImportError:                    # pragma: no cover
    DockStatus = Dock = Undock = None
try:                                   # Nav2 메시지 — 없으면 navigate_through_poses 생략
    from nav2_msgs.action import NavigateThroughPoses
except ImportError:                    # pragma: no cover
    NavigateThroughPoses = None

try:                                   # JPEG 인코딩 — OpenCV가 있으면 사용, 없으면 Pillow
    import cv2
except ImportError:                    # pragma: no cover
    cv2 = None
    from PIL import Image, ImageDraw

ROBOT_ID = "robot3"
DEFAULT_WORLD = Path(__file__).resolve().parent / "virtual_world" / "testbed" / "world.yaml"
CAM_W, CAM_H = 640, 360                # 카메라 프레임 크기 (순찰 영상도 이 크기로 맞춤)
STEP_S = 0.05                          # 이동 계산 간격
DOCK_REACH_M = 1.0                     # 도크 접근 자세에서 이만큼 안이면 dock 성공
VIEW_RANGE_M = 2.5                     # 가상 판정 — 설비가 이 거리 안이어야 보임 (world.yaml truth.camera.view_range_m)
PHOTO_HOLD_S = 6.0                     # 판정 사진을 카메라에 내보내는 시간 (truth.camera.photo_hold_s)


def local_ip() -> str:
    """이 PC의 바깥쪽 IP — UDP 소켓 connect로 경로만 고름(패킷은 안 보냄). 실패하면 127.0.0.1"""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("10.255.255.255", 1))
            return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"


def load_world(path: str | None):
    """--world → virtual_world/world_model.World (map · truth). 빈 경로면 None(가상 세계 없이 로봇만), 파일이 없으면 종료"""
    if not path:
        return None
    p = Path(path)
    if not p.exists():
        sys.exit(f"world 파일이 없습니다: {p}")
    sys.path.insert(0, str(Path(__file__).resolve().parent / "virtual_world"))   # world_model을 import하려고
    from world_model import World
    return World(p)


class VideoSource:
    """순찰 영상(mp4) → JPEG 프레임 목록(메모리). OpenCV가 있어야 읽음"""
    def __init__(self, path: Path):
        """영상 전체를 읽어 CAM_W×CAM_H · JPEG(품질 80)로 frames에 쌓음. fps는 영상 값(없으면 10)"""
        cap = cv2.VideoCapture(str(path))
        self.fps = cap.get(cv2.CAP_PROP_FPS) or 10.0
        self.frames: list[bytes] = []
        while True:                                        # 끝까지 한 장씩 읽어 크기 맞춤 → JPEG
            ok, img = cap.read()
            if not ok:
                break
            if (img.shape[1], img.shape[0]) != (CAM_W, CAM_H):
                img = cv2.resize(img, (CAM_W, CAM_H))
            self.frames.append(cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes())
        cap.release()


def yaw_of(q) -> float:
    """쿼터니언 → yaw(rad)"""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a: float) -> float:
    """각도를 -π~π로"""
    return math.atan2(math.sin(a), math.cos(a))


class VirtualRobot(Node):
    """가상 TurtleBot4 노드 (/{robot_id}) — 위 표의 토픽 발행 · 구독 · 액션 서버. 상태(pose · docked · battery …)는 self.lock으로 보호"""
    def __init__(self, robot_id: str, camera_hz: float, world, speed: float, rssi_hz: float):
        """상태 초기화(도킹 · 위치 모름) → 순찰 영상 읽기 → 발행자 · 구독 · 타이머 · 액션 서버 만들기 → 시작 로그"""
        super().__init__(f"{robot_id}_virtual", namespace=f"/{robot_id}")
        self.robot_id = robot_id
        self.world = world                                 # world_model.World 또는 None(가상 세계 없음)
        self.camera_hz = camera_hz
        self.speed = max(0.05, speed)                      # m/s
        self.turn = 1.5                                    # rad/s
        self.t0 = time.monotonic()
        self.battery = 0.87                                # 0~1 (Create 3와 같은 비율)
        self.ip = local_ip()
        self.frame_no = 0
        self.lock = threading.Lock()
        # 도크에 서서 시작(도킹 상태). 지도 위 위치는 initialpose를 받기 전까지 모름
        self.dock: tuple[float, float, float] | None = None   # 도크 자세(map) = odom 원점 · dock 액션 목표 — 도킹 중 받은 initialpose
        self.pose: list[float] | None = None                  # map 좌표 x, y, yaw
        self.docked = True
        self.moving = False                                # 액션(nav · dock · undock) 실행 중 — initialpose 무시 · 카메라 영상 진행
        self.vel = (0.0, 0.0)                              # odom 표시용 선속도 · 각속도
        self.photo: tuple[bytes, float, str] | None = None # 판정 사진 (JPEG, 끝 시각(monotonic), 설비 ID) — 카메라에 잠시
        self.photo_cache: dict[str, bytes] = {}            # 판정 사진 파일 경로 → JPEG (한 번만 읽음)
        cam = (world.truth.get("camera") or {}) if world else {}
        self.view_range = float(cam.get("view_range_m", VIEW_RANGE_M))
        self.photo_hold = float(cam.get("photo_hold_s", PHOTO_HOLD_S))
        self.video, self.video_pos = None, 0.0
        vpath = world.file(cam.get("video")) if world else None
        # 순찰 영상 — 쓸 수 없으면(OpenCV 없음 · 파일 없음 · 프레임 0장) 경고 후 합성 화면
        if vpath and camera_hz > 0:
            if cv2 is None:
                self.get_logger().warn("OpenCV(python3-opencv)가 없어 순찰 영상을 읽지 못합니다 — 합성 화면으로 대신")
            elif not vpath.exists():
                self.get_logger().warn(f"순찰 영상이 없습니다: {vpath} — virtual_world/make_assets.py로 만들거나 합성 화면으로 대신")
            else:
                self.video = VideoSource(vpath)
                if not self.video.frames:
                    self.get_logger().warn(f"순찰 영상을 읽지 못했습니다: {vpath} — 합성 화면으로 대신")
                    self.video = None

        # 발행자 — 상대 이름이라 /{robot_id}/… 가 됨
        self.pub_battery = self.create_publisher(BatteryState, "battery_state", 10)
        self.pub_dock = self.create_publisher(DockStatus, "dock_status", 10) if DockStatus else None
        self.pub_odom = self.create_publisher(Odometry, "odom", 10)
        self.pub_amcl = self.create_publisher(PoseWithCovarianceStamped, "amcl_pose", 10)
        self.pub_ip = self.create_publisher(String, "ip", 10)
        has_aps = bool(world and world.truth.get("ap_models"))
        self.pub_rssi = self.create_publisher(String, "rssi", 10) if has_aps else None        # 가상 세계에 truth.ap_models가 있을 때만
        self.pub_detect = self.create_publisher(String, "virtual/detect_result", 10) if world else None
        self.pub_cam = (self.create_publisher(CompressedImage, "oakd/rgb/image_raw/compressed", qos_profile_sensor_data)
                        if camera_hz > 0 else None)

        # 구독 · 타이머 — 1 Hz(배터리 · 도킹 · IP), 10 Hz odom, 2 Hz amcl_pose, rssi · 카메라는 인자 주기
        self.create_subscription(PoseWithCovarianceStamped, "initialpose", self.on_initialpose, 10)
        if world:
            self.create_subscription(String, "virtual/detect_request", self.on_detect_request, 10)
        self.create_timer(1.0, self.tick_1hz)
        self.create_timer(0.1, self.tick_odom)
        self.create_timer(0.5, self.tick_amcl)
        if rssi_hz > 0 and self.pub_rssi:
            self.create_timer(1.0 / rssi_hz, self.tick_rssi)
        if self.pub_cam:
            self.create_timer(1.0 / camera_hz, self.tick_camera)

        # 액션 서버 — 실행 콜백이 길게 도는 동안 타이머가 멈추지 않도록 재진입 그룹 + 다중 스레드 실행기
        cg = ReentrantCallbackGroup()
        accept = lambda _req: CancelResponse.ACCEPT        # noqa: E731 — 취소 요청은 모두 받음
        self.servers = []
        if NavigateThroughPoses is not None:
            self.servers.append(ActionServer(self, NavigateThroughPoses, "navigate_through_poses", self.exec_nav,
                                             cancel_callback=accept, callback_group=cg))
        if Dock is not None:
            self.servers.append(ActionServer(self, Undock, "undock", self.exec_undock, cancel_callback=accept, callback_group=cg))
            self.servers.append(ActionServer(self, Dock, "dock", self.exec_dock, cancel_callback=accept, callback_group=cg))
        if not DockStatus:
            self.get_logger().warn("irobot_create_msgs가 없어 dock_status · dock · undock은 내보내지 않습니다")
        if NavigateThroughPoses is None:
            self.get_logger().warn("nav2_msgs가 없어 navigate_through_poses는 내보내지 않습니다")
        # 시작 로그 — 제공하는 액션 · 가상 세계 요약
        acts = ', '.join(['navigate_through_poses'] * bool(NavigateThroughPoses) + ['undock', 'dock'] * bool(Dock)) or '없음'
        self.get_logger().info(
            f"가상 로봇 {robot_id} 시작 — 도킹 중, 위치는 /{robot_id}/initialpose를 기다림 · 이동 {self.speed:g} m/s · 액션 {acts}")
        if world:
            t = world.truth
            self.get_logger().info(
                f"가상 세계 {world.path} — 지도 {world.map_yaml.name} · AP {', '.join(t.get('ap_models') or {}) or '없음'} "
                f"(RSSI {rssi_hz:g} Hz, 벽 {world.wall_loss:g} dB) · 음영 {len(t.get('holes') or [])}곳 · "
                f"설비 정답 {len(t.get('facilities') or {})}개 · 카메라 {'순찰 영상 ' + str(len(self.video.frames)) + '장' if self.video else '합성 화면'}")
        else:
            self.get_logger().info("가상 세계 없음(--no-world) — 로봇만: rssi · 가상 판정 없음, 카메라는 합성 화면")

    # ------------------------------------------------------------------ initialpose — 시작 위치 지정
    def on_initialpose(self, m):
        """initialpose 구독 — 받은 자세를 위치(map)로. 도킹 중(또는 처음)이면 도크 · odom 원점도 그 자리로. 주행 중이면 무시"""
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = yaw_of(q)
        with self.lock:
            if self.moving:                                # 주행 중에는 무시 (실제 AMCL은 받아들이지만 시험 단순화)
                return
            self.pose = [float(p.x), float(p.y), yaw]
            if self.dock is None or self.docked:           # 도크에 있으면 이 자리가 도크 · odom 원점
                self.dock = (float(p.x), float(p.y), yaw)
        self.get_logger().info(f"initialpose 받음 — ({p.x:.2f}, {p.y:.2f}, {yaw:.3f}) · amcl_pose 발행 시작")

    # ------------------------------------------------------------------ 이동 (액션 실행 스레드에서)
    def _move_to(self, x: float, y: float, goal_handle, yaw: float | None = None) -> str:
        """현재 자세에서 (x, y)까지 제자리 회전 후 직선 이동(STEP_S마다 pose · vel 갱신, 배터리 조금 소모).
        yaw를 주면 도착 뒤 그 방향으로 회전. 반환: 'ok' · 'canceled'(취소 요청) · 'lost'(위치 모름)"""
        while True:                                        # (x, y)에 닿을 때까지 한 걸음씩
            if goal_handle is not None and goal_handle.is_cancel_requested:
                return "canceled"
            with self.lock:
                if self.pose is None:
                    return "lost"
                px, py, pyaw = self.pose
                dx, dy = x - px, y - py
                dist = math.hypot(dx, dy)
                if dist < 1e-3:
                    break
                head = math.atan2(dy, dx)
                err = wrap(head - pyaw)
                if abs(err) > 0.05:                        # 먼저 제자리 회전
                    step = math.copysign(min(abs(err), self.turn * STEP_S), err)
                    self.pose[2] = wrap(pyaw + step)
                    self.vel = (0.0, step / STEP_S)
                else:                                      # 방향이 맞으면 직진 (한 걸음 = speed × STEP_S, 남은 거리 이하)
                    step = min(dist, self.speed * STEP_S)
                    self.pose[0] = px + math.cos(head) * step
                    self.pose[1] = py + math.sin(head) * step
                    self.pose[2] = head
                    self.vel = (step / STEP_S, 0.0)
                self.battery = max(0.05, self.battery - 0.00001)
            time.sleep(STEP_S)
        if yaw is not None:                                # 마지막 자세 방향 맞추기
            while True:
                if goal_handle is not None and goal_handle.is_cancel_requested:
                    return "canceled"
                with self.lock:
                    err = wrap(yaw - self.pose[2])
                    if abs(err) < 0.02:
                        self.pose[2] = yaw
                        break
                    step = math.copysign(min(abs(err), self.turn * STEP_S), err)
                    self.pose[2] = wrap(self.pose[2] + step)
                    self.vel = (0.0, step / STEP_S)
                time.sleep(STEP_S)
        with self.lock:
            self.vel = (0.0, 0.0)
        return "ok"

    def _goal_poses(self, goal):
        """목표 점 목록 [(x, y, yaw)] — Jazzy: goal.poses (PoseStamped[]), 이후 판: goal.goals.goals — 둘 다 받음"""
        poses = getattr(goal, "poses", None)
        if poses is None and hasattr(goal, "goals"):
            poses = getattr(goal.goals, "goals", goal.goals)
        return [(float(p.pose.position.x), float(p.pose.position.y), yaw_of(p.pose.orientation)) for p in (poses or [])]

    def exec_nav(self, goal_handle):
        """navigate_through_poses 실행 — 위치를 알고 도킹이 아닐 때만. 점마다 _move_to(마지막 점만 방향 맞춤),
        도착할 때마다 피드백(current_pose · number_of_poses_remaining · distance_remaining). 취소 → canceled, 그 밖 실패 → abort"""
        pts = self._goal_poses(goal_handle.request)
        result = NavigateThroughPoses.Result()
        with self.lock:
            ready = self.pose is not None and not self.docked
        if not ready or not pts:                           # 위치 모름 · 도킹 중 · 점 없음 → 거부(abort)
            self.get_logger().warn(f"navigate_through_poses 거부 — {'위치 모름' if self.pose is None else '도킹 중' if self.docked else '점 없음'}")
            goal_handle.abort()
            return result
        self.get_logger().info(f"navigate_through_poses 시작 — 점 {len(pts)}개")
        with self.lock:
            self.moving = True
        try:
            for i, (x, y, yaw) in enumerate(pts):
                last = i == len(pts) - 1
                r = self._move_to(x, y, goal_handle, yaw if last else None)
                if r != "ok":                              # 취소 → canceled, 위치 잃음 → abort
                    if r == "canceled":
                        goal_handle.canceled()
                        self.get_logger().info("navigate_through_poses 취소됨")
                    else:
                        goal_handle.abort()
                    return result
                # 피드백 — 지금 자세, 남은 점 수, 남은 점들 사이 직선 거리 합
                fb = NavigateThroughPoses.Feedback()
                with self.lock:
                    fb.current_pose.header.frame_id = "map"
                    fb.current_pose.pose.position.x = self.pose[0]
                    fb.current_pose.pose.position.y = self.pose[1]
                    fb.current_pose.pose.orientation.z = math.sin(self.pose[2] / 2)
                    fb.current_pose.pose.orientation.w = math.cos(self.pose[2] / 2)
                fb.number_of_poses_remaining = len(pts) - i - 1
                fb.distance_remaining = float(sum(math.hypot(pts[k + 1][0] - pts[k][0], pts[k + 1][1] - pts[k][1])
                                                  for k in range(i, len(pts) - 1)))
                goal_handle.publish_feedback(fb)
        finally:
            with self.lock:
                self.moving = False
        goal_handle.succeed()
        self.get_logger().info("navigate_through_poses 완료")
        return result

    def exec_undock(self, goal_handle):
        """undock 실행 — 0.5초 뒤 지금 방향 그대로 25 cm 뒤로 물러나 docked=False. 취소는 보지 않음.
        이미 도킹이 아니면 바로 성공, 위치 모름(initialpose 전)이면 abort. 결과 is_docked"""
        res = Undock.Result()
        with self.lock:
            if not self.docked or self.pose is None:       # 이미 나와 있음 → 성공 / 도킹 중인데 위치 모름 → 실패
                res.is_docked = self.docked
                goal_handle.succeed() if not self.docked else goal_handle.abort()
                return res
            self.docked = False
            self.moving = True
            x, y, yaw = self.pose
        time.sleep(0.5)
        back = (x - math.cos(yaw) * 0.25, y - math.sin(yaw) * 0.25)   # 도크에서 25 cm 뒤로 (방향 유지)
        with self.lock:
            self.pose[0], self.pose[1] = back
            self.moving = False
        res.is_docked = False
        goal_handle.succeed()
        self.get_logger().info("undock 완료")
        return res

    def exec_dock(self, goal_handle):
        """dock 실행 — 도크 자세(self.dock)에서 DOCK_REACH_M 안이면 그 자리로 이동 · 방향 맞춘 뒤 docked=True(충전 시작).
        이미 도킹이면 바로 성공, 멀거나 위치 모름이면 abort, 취소 → canceled. 결과 is_docked"""
        res = Dock.Result()
        with self.lock:
            dock, pose = self.dock, list(self.pose) if self.pose else None
        if self.docked:
            res.is_docked = True
            goal_handle.succeed()
            return res
        if dock is None or pose is None or math.hypot(pose[0] - dock[0], pose[1] - dock[1]) > DOCK_REACH_M:
            self.get_logger().warn("dock 실패 — 도크가 보이지 않음(도크 접근 자세에서 멂)")
            res.is_docked = False
            goal_handle.abort()
            return res
        with self.lock:
            self.moving = True
        r = self._move_to(dock[0], dock[1], goal_handle, dock[2])
        with self.lock:
            self.moving = False
            if r == "ok":
                self.docked = True
        if r != "ok":                                      # 취소 → canceled, 그 밖 → abort (is_docked=False)
            goal_handle.canceled() if r == "canceled" else goal_handle.abort()
            return res
        res.is_docked = True
        goal_handle.succeed()
        self.get_logger().info("dock 완료 — 충전 시작")
        return res

    # ------------------------------------------------------------------ RSSI — rssi_scanner 대신 (world.yaml 모델)
    def tick_rssi(self):
        """rssi 타이머(--rssi-hz) — 지금 위치의 AP별 RSSI를 JSON {"stamp", "scans": [{ssid, rssi_dbm}]}로 발행.
        값은 world_model.World.rssi_at (거리 · 지도 벽 · 음영 구역 · 잡음)"""
        with self.lock:
            pose = list(self.pose) if self.pose else None
        if pose is None:                                   # 위치를 모름(initialpose 전) — 내보내지 않음
            return
        s = String()
        s.data = json.dumps({"stamp": datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z"),
                             "scans": self.world.rssi_at(pose[0], pose[1])})
        self.pub_rssi.publish(s)

    # ------------------------------------------------------------------ 가상 판정 — YOLO 대신 world.yaml 정답 (시험 전용)
    def on_detect_request(self, m):
        """virtual/detect_request 구독 — 요청 JSON(request_id · facility_id · facility_type · has_gauge · expected_x/y)에
        truth 정답 · 거리로 판정해 virtual/detect_result 발행, 판정 사진을 카메라에 photo_hold_s 동안 띄움.
        truth 값: normal · abnormal → 등록 위치에 검출, position → moved_to에 검출, not_detected → 검출 없음,
        occluded → frontal=False · occluded=True. 멀거나(view_range 밖 · 위치 모름) truth에 없으면 frontal=False"""
        try:
            req = json.loads(m.data)
        except ValueError:                                 # JSON이 아니면 무시 (client는 시간 초과)
            return
        fid = str(req.get("facility_id", ""))
        ftype = str(req.get("facility_type", ""))
        ex, ey = float(req.get("expected_x", 0)), float(req.get("expected_y", 0))
        truth = self.world.facility_truth(fid)
        t = truth.get("truth", "normal") if truth else "unknown"
        with self.lock:
            pose = list(self.pose) if self.pose else None
        out = {"request_id": req.get("request_id"), "facility_id": fid, "frontal": True, "occluded": False, "detections": []}
        # 보이는지 — 설비 자리(등록 위치)가 view_range 안이어야 그 자리를 볼 수 있음. 멀거나 정답이 없으면 정면 관측 실패(→ 미확인)
        near = pose is not None and math.hypot(pose[0] - ex, pose[1] - ey) <= self.view_range
        why = ""
        if not near or truth is None:
            out.update(frontal=False)
            why = " — 멀어서 그 자리를 못 봄" if not near else " — world.yaml truth에 이 설비가 없음"
        elif t == "occluded":
            out.update(frontal=False, occluded=True)
        elif t != "not_detected":                          # normal · abnormal · position(그 밖 값은 normal처럼) → 검출 1개
            px, py = (float(truth["moved_to"]["x"]), float(truth["moved_to"]["y"])) if t == "position" else (ex, ey)
            if math.hypot(pose[0] - px, pose[1] - py) <= self.view_range:   # 옮겨진 설비도 보이는 거리 안일 때만 검출
                det = {"class": ftype, "x": px, "y": py, "conf": round(random.uniform(0.82, 0.97), 3),
                       "bbox": [220, 90, 420, 300], "state": "abnormal" if t == "abnormal" else "normal"}
                if req.get("has_gauge"):                   # 게이지 설비 — abnormal이면 "abnormal", 아니면 "green"
                    det["gauge"] = "abnormal" if t == "abnormal" else "green"
                out["detections"].append(det)
        # 판정 사진 — 카메라 토픽에 photo_hold_s 동안 (client가 판정 직후 새로 받은 프레임을 판정 사진으로 media 업로드)
        self.photo = (self.photo_jpeg(fid, ftype, t if near else "far"), time.monotonic() + self.photo_hold, fid)
        s = String()
        s.data = json.dumps(out)
        self.pub_detect.publish(s)
        self.get_logger().info(f"가상 판정 {fid}: {t}{why}")

    def photo_jpeg(self, fid: str, ftype: str, t: str) -> bytes:
        """판정 사진 JPEG — truth의 photo 파일(make_assets.py, 캐시), 멀면(t='far') · 파일이 없으면 글자만 있는 합성 화면"""
        truth = self.world.facility_truth(fid) or {}
        p = self.world.file(truth.get("photo")) if t != "far" else None
        if p and p.exists():
            if str(p) not in self.photo_cache:
                self.photo_cache[str(p)] = p.read_bytes()
            return self.photo_cache[str(p)]
        if p:
            self.get_logger().warn(f"판정 사진이 없습니다: {p} — virtual_world/make_assets.py로 만드세요")
        return self.render_frame(f"{fid} {ftype} - {'too far' if t == 'far' else 'truth ' + t}")

    # ------------------------------------------------------------------ 1 Hz
    def tick_1hz(self):
        """1 Hz 타이머 — 배터리 충전/방전 계산 후 battery_state · dock_status(있으면) · ip 발행"""
        now = self.get_clock().now().to_msg()
        with self.lock:
            if self.docked:                                # 도킹 중이면 충전, 아니면 방전
                self.battery = min(1.0, self.battery + 0.0005)
            else:
                self.battery = max(0.05, self.battery - 0.0002)
            docked, battery = self.docked, self.battery
        # BatteryState — percentage 0~1, 전압 · 전류는 그럴듯한 값 (도킹 중 +, 주행 중 −)
        b = BatteryState()
        b.header.stamp = now
        b.header.frame_id = "base_link"
        b.percentage = float(battery)
        b.voltage = float(14.4 + 2.0 * battery)
        b.current = 1.5 if docked else -1.2
        b.temperature = 31.5
        b.charge = float(2.1 * battery)
        b.capacity = 2.1
        b.design_capacity = 2.1
        b.power_supply_status = (BatteryState.POWER_SUPPLY_STATUS_FULL if docked and battery >= 1.0
                                 else BatteryState.POWER_SUPPLY_STATUS_CHARGING if docked
                                 else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING)
        b.present = True
        self.pub_battery.publish(b)

        if self.pub_dock:                                  # DockStatus — 도킹 중이면 dock_visible · is_docked 모두 True
            d = DockStatus()
            d.header.stamp = now
            d.dock_visible = docked
            d.is_docked = docked
            self.pub_dock.publish(d)

        s = String()
        s.data = self.ip
        self.pub_ip.publish(s)

    # ------------------------------------------------------------------ 10 Hz — odom (도크 자세 기준)
    def tick_odom(self):
        """10 Hz 타이머 — odom 발행: map 위치를 도크 자세(odom 원점) 기준 좌표로 바꾼 값 + 선속도 · 각속도"""
        with self.lock:
            pose, dock, vel = (list(self.pose) if self.pose else None), self.dock, self.vel
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id = "odom"
        o.child_frame_id = "base_link"
        if pose is None:                                   # 위치를 모름 — odom은 원점(0, 0)
            o.pose.pose.orientation.w = 1.0
            self.pub_odom.publish(o)
            return
        # map → odom: 도크 위치를 빼고 도크 yaw만큼 거꾸로 회전
        dx, dy = pose[0] - dock[0], pose[1] - dock[1]
        c, s_ = math.cos(-dock[2]), math.sin(-dock[2])
        o.pose.pose.position.x = c * dx - s_ * dy
        o.pose.pose.position.y = s_ * dx + c * dy
        yaw = pose[2] - dock[2]
        o.pose.pose.orientation.z = math.sin(yaw / 2)
        o.pose.pose.orientation.w = math.cos(yaw / 2)
        o.twist.twist.linear.x = float(vel[0])
        o.twist.twist.angular.z = float(vel[1])
        self.pub_odom.publish(o)

    # ------------------------------------------------------------------ 2 Hz — amcl_pose (map 좌표)
    def tick_amcl(self):
        """2 Hz 타이머 — amcl_pose 발행 (frame map, 위치 σ 5 cm 고정 공분산). initialpose 전에는 발행 안 함"""
        with self.lock:
            pose = list(self.pose) if self.pose else None
        if pose is None:                                   # initialpose 전 — AMCL처럼 위치를 내보내지 않음
            return
        m = PoseWithCovarianceStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.pose.pose.position.x = float(pose[0])
        m.pose.pose.position.y = float(pose[1])
        m.pose.pose.orientation.z = math.sin(pose[2] / 2)
        m.pose.pose.orientation.w = math.cos(pose[2] / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.0025                           # 위치 분산 (σ 5 cm)
        cov[35] = 0.003                                    # yaw 분산
        m.pose.covariance = cov
        self.pub_amcl.publish(m)

    # ------------------------------------------------------------------ 카메라 — 판정 사진 · 순찰 영상 · 합성 화면 JPEG
    def tick_camera(self):
        """카메라 타이머(--camera-hz) — CompressedImage(jpeg) 발행. 고르는 순서:
        판정 사진(photo_hold_s 동안) → 순찰 영상(움직일 때 다음 장면, 멈추면 그 장면) → 합성 화면"""
        self.frame_no += 1
        now = time.monotonic()
        with self.lock:
            moving, docked = self.moving, self.docked
        photo = self.photo if self.photo and self.photo[1] > now else None
        if photo:
            jpeg = self.stamp_jpeg(photo[0], f"INSPECT {photo[2]}")
        elif self.video:
            if moving:                                     # 영상 fps에 맞춰 진행 (끝나면 처음부터)
                self.video_pos = (self.video_pos + self.video.fps / self.camera_hz) % len(self.video.frames)
            jpeg = self.stamp_jpeg(self.video.frames[int(self.video_pos)], "DOCKED" if docked else "MOVING" if moving else "STOP")
        else:
            jpeg = self.render_frame()
        m = CompressedImage()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "oakd_rgb_camera_optical_frame"
        m.format = "jpeg"
        m.data = array.array("B", jpeg)
        self.pub_cam.publish(m)

    def stamp_jpeg(self, jpeg: bytes, state: str) -> bytes:
        """영상 · 사진 위 맨 위 띠에 로봇 · 상태 · 시각 (영어 — OpenCV 글꼴). OpenCV가 없으면 Pillow로 → 새 JPEG"""
        text = f"{self.robot_id}  {state}  {time.strftime('%H:%M:%S')}"
        if cv2 is not None:
            img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            cv2.rectangle(img, (0, 0), (img.shape[1], 30), (20, 20, 20), -1)
            cv2.putText(img, text, (10, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
            return cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()
        img = Image.open(io.BytesIO(jpeg)).convert("RGB")
        d = ImageDraw.Draw(img)
        d.rectangle([0, 0, img.width, 30], fill=(20, 20, 20))
        d.text((10, 9), text, fill=(255, 255, 255))
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=80)
        return buf.getvalue()

    def render_frame(self, ov: str | None = None) -> bytes:
        """합성 화면 JPEG — 가상 세계가 없거나 영상 · 사진 파일이 없을 때. 좌우로 움직이는 상자 + 로봇 · 프레임 번호 · 시각,
        ov가 있으면 아래쪽에 한 줄 더(ASCII만). OpenCV가 없으면 Pillow로"""
        t = time.monotonic() - self.t0
        label = f"{self.robot_id} virtual  frame {self.frame_no}  {time.strftime('%H:%M:%S')}"
        x = int((CAM_W - 120) * (0.5 + 0.5 * math.sin(t * 0.8)))
        if cv2 is not None:                                # OpenCV (색은 BGR)
            img = np.zeros((CAM_H, CAM_W, 3), np.uint8)
            img[:] = (60, 52, 40)
            cv2.rectangle(img, (x, 120), (x + 120, 240), (40, 170, 220), -1)
            cv2.line(img, (0, 300), (CAM_W, 300), (120, 120, 120), 2)
            cv2.putText(img, label, (16, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            if ov:
                cv2.putText(img, ov.encode("ascii", "replace").decode(), (16, 340), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (80, 220, 255), 2)
            ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
            return buf.tobytes()
        img = Image.new("RGB", (CAM_W, CAM_H), (40, 52, 60))   # Pillow (색은 RGB — 위와 같은 색)
        d = ImageDraw.Draw(img)
        d.rectangle([x, 120, x + 120, 240], fill=(220, 170, 40))
        d.line([0, 300, CAM_W, 300], fill=(120, 120, 120), width=2)
        d.text((16, 24), label, fill=(255, 255, 255))
        if ov:
            d.text((16, 330), ov.encode("ascii", "replace").decode(), fill=(255, 220, 80))
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=70)
        return buf.getvalue()


def main():
    """인자 해석 → 가상 세계 읽기 → VirtualRobot을 다중 스레드 실행기(4)로 돌림. Ctrl+C로 종료"""
    ap = argparse.ArgumentParser(description="SCV 가상 로봇")
    ap.add_argument("--robot-id", default=ROBOT_ID)
    ap.add_argument("--camera-hz", type=float, default=10.0)
    ap.add_argument("--speed", type=float, default=0.3, help="이동 속도 m/s (기본 0.3 — TurtleBot4 순찰 속도)")
    ap.add_argument("--rssi-hz", type=float, default=1.0, help="RSSI 발행 주기 (기본 1 Hz, 0이면 끔)")
    ap.add_argument("--world", default=str(DEFAULT_WORLD),
                    help="가상 세계 world.yaml (기본: virtual_world/testbed/world.yaml)")
    ap.add_argument("--no-world", action="store_true", help="가상 세계 없이 로봇만 (rssi · 가상 판정 없음, 합성 카메라)")
    a, _ros_args = ap.parse_known_args()       # --ros-args … 는 rclpy가 sys.argv에서 직접 읽음
    world = None if a.no_world else load_world(a.world)
    rclpy.init()
    node = VirtualRobot(a.robot_id, a.camera_hz, world, a.speed, a.rssi_hz)
    ex = MultiThreadedExecutor(num_threads=4)              # 액션 실행 중에도 타이머 · 구독이 돌도록
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ex.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
