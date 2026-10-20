"""ros_bridge — SCV_RUN_OWNER=ros 일 때 server_page 프로세스 안에서 도는 ROS 2 다리 (10/14 PM 결정: 순찰 주인은 ROS)

순찰 · 작업은 inspection_manager(관제 A)가 정하고, 이 다리는 그 결과를 server_page 의 기존 저장 함수로 넘긴다.
  B1 /{rid}/status (RobotStatus)            → store_status · robots 활성        (모니터 로봇 상태)
  B2 웹 버튼                                 → /inspection/start · estop · resume (Trigger)
  B3 /inspection/alerts (Alert)              → add_event (DB 규칙에 맞는 것) / 화면만 (나머지)
  B4 /{rid}/inspect/result (InspectionResult) → on_inspection · on_media          (점검 이력 · 사진)
  (작업 줄 tasks 는 inspection_manager 가 직접 DB 에 쓴다 — sync_tasks. 이 다리는 읽기만)
  B6 /{rid}/survey/sample (NetSample)        → on_samples (1초 묶음)             (히트맵 · 칸별 측정값)
  B7 /{rid}/plan (nav_msgs/Path, Nav2 전역 경로) → sp.ROBOT_PLAN (모니터 지도에 점, RViz 처럼)
  B8 /{rid}/camera/preview (CompressedImage, 로봇 PC camera_preview — 주행 중만, 640 px 1 Hz) → 블랙박스 파일
     scv_media/recordings/r{run}/{rid}/{ms}.jpg (진행 중 회차 · 도크 밖일 때만) — 이력 탭에서 넘겨 봄 (실시간 표시는 없음)
  비상정지 여부: estop · resume 응답과 E7 · RESUMED 알림 → sp.ESTOP["on"] (모니터 「재개」 버튼)
server_page 를 import 하지 않고 시작할 때 모듈(sp)을 받아 쓴다 (server_page 는 __main__ 으로 뜨므로)."""
from __future__ import annotations

import asyncio
import itertools
import json
import threading
import time
import uuid
from datetime import datetime, timezone

import math

import rclpy
from nav_msgs.msg import Path
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from scv_msgs.msg import Alert, InspectionResult, NetSample, RobotStatus
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Trigger

ROBOTS = ("robot4", "robot3")
NAV_STATE = ("IDLE", "EXECUTING", "BLOCKED", "DOCKING", "DOCKED", "ESTOP", "ERROR")
VERDICT = {0: ("NORMAL", "normal"), 1: ("NOT_DETECTED", "abnormal"), 2: ("OCCLUDED", "abnormal"),
           3: ("POSITION_ANOMALY", "abnormal"), 4: ("ABNORMAL", "abnormal"),
           5: ("OCCLUDED", "unconfirmed")}          # 판정 못 함 — client_page judge() 와 같은 저장 방식
# robot_events.code CHECK 에 있는 것 · source=control 로 넣을 수 있는 것 · task_id 가 꼭 있어야 하는 것 (01_schema.sql)
DB_CODES = {"E1", "E2", "E3", "E4", "E5", "E6", "E7", "E8", "CANDIDATE", "CONFIRMED", "CLEARED", "UNCONFIRMED"}
CONTROL_CODES = {"E3", "E8"}
NEED_TASK = {"CANDIDATE", "CONFIRMED", "CLEARED", "UNCONFIRMED"}
LEVEL = ("info", "warn", "error")
CAMERA_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)   # 블랙박스 프레임 — 놓쳐도 됨
STATUS_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
RESULT_QOS = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
SAMPLE_QOS = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)


def iso(stamp) -> str:
    """builtin_interfaces/Time → ISO 문자열 (0 이면 지금)"""
    t = stamp.sec + stamp.nanosec * 1e-9
    return datetime.fromtimestamp(t or time.time(), timezone.utc).isoformat()


class Bridge(Node):
    def __init__(self, sp):
        super().__init__("scv_ros_bridge")
        self.sp = sp
        self.task = {}                              # robot_id → 지금 작업 (status 로 본 것)
        self.docked = {}                            # robot_id → 도크 위 (그동안 표본은 버림 — 회차가 열려 있어도 측정이 아님)
        self.samples = {}                           # (robot_id, run_id) → 모은 표본
        self.target = {}                            # run_id → 측정 AP SSID
        self.local_id = itertools.count(-1, -1)     # DB 에 넣지 않은 알림의 event_id (화면용)
        self.lock = threading.Lock()
        for rid in ROBOTS:
            self.create_subscription(RobotStatus, f"/{rid}/status", lambda m, r=rid: self.on_status(r, m), STATUS_QOS)
            self.create_subscription(InspectionResult, f"/{rid}/inspect/result",
                                     lambda m, r=rid: self.on_result(r, m), RESULT_QOS)
            self.create_subscription(NetSample, f"/{rid}/survey/sample", lambda m, r=rid: self.on_sample(r, m), SAMPLE_QOS)
            self.create_subscription(Path, f"/{rid}/plan", lambda m, r=rid: self.on_plan(r, m), STATUS_QOS)
            self.create_subscription(CompressedImage, f"/{rid}/camera/preview",
                                     lambda m, r=rid: self.on_frame(r, m), CAMERA_QOS)
        self.create_subscription(Alert, "/inspection/alerts", self.on_alert, 10)
        self.clients_ = {n: self.create_client(Trigger, f"/inspection/{n}") for n in ("start", "estop", "resume", "abort")}
        self.create_timer(1.0, self.flush_samples)

    # ------------------------------------------------------------ B2 버튼
    def call(self, name: str, timeout: float = 5.0) -> dict:
        """웹 요청 스레드에서 부름 — Trigger 서비스 → {ok, message}. 실행기는 다른 스레드에서 돈다"""
        cli = self.clients_[name]
        if not cli.wait_for_service(timeout_sec=1.0):
            return {"ok": False, "message": f"/inspection/{name} 서비스가 없습니다 (inspection_manager 실행 확인)"}
        fut = cli.call_async(Trigger.Request())
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.05)
        if not fut.done():
            return {"ok": False, "message": f"/inspection/{name} 응답이 {timeout:.0f}초 안에 없습니다"}
        r = fut.result()
        self.refresh_run()
        if r.success and name in ("estop", "resume"):
            self.sp.ESTOP["on"] = name == "estop"
        elif r.success and name in ("start", "abort"):
            self.sp.ESTOP["on"] = False
        return {"ok": bool(r.success), "message": r.message}

    # ------------------------------------------------------------ B7 Nav2 경로
    def on_frame(self, rid: str, m: CompressedImage):
        """블랙박스: 진행 중 회차 · 도크 밖 프레임만 파일로 (파일 이름 = 촬영 시각 ms). 이력 탭이 목록을 읽어 재생"""
        run_id = self.sp.CURRENT_RUN["run_id"]
        if not run_id or self.docked.get(rid) or not m.data:
            return
        ms = m.header.stamp.sec * 1000 + m.header.stamp.nanosec // 1_000_000 or int(time.time() * 1000)
        d = self.sp.MEDIA_DIR / "recordings" / f"r{run_id}" / rid
        try:
            d.mkdir(parents=True, exist_ok=True)
            (d / f"{ms}.jpg").write_bytes(bytes(m.data))
        except OSError as e:
            self.get_logger().warning(f"블랙박스 {rid}: {e}", throttle_duration_sec=30.0)

    def on_plan(self, rid: str, m: Path):
        """전역 경로를 약 0.1 m 간격 점으로 줄여 둠 — 웹이 1초마다 가져감"""
        pts, last = [], None
        for p in m.poses:
            xy = (round(p.pose.position.x, 2), round(p.pose.position.y, 2))
            if last is None or math.dist(xy, last) >= 0.1:
                pts.append(xy)
                last = xy
        with self.sp.MEM_LOCK:
            self.sp.ROBOT_PLAN[rid] = pts

    def refresh_run(self):
        with self.sp.pool.connection() as conn:
            run = self.sp.running_run(conn)
        self.sp.CURRENT_RUN["run_id"] = run and run["run_id"]

    # ------------------------------------------------------------ B1 상태
    def known_task(self, task: str | None) -> str | None:
        """DB tasks 에 있는 작업이면 그 id (외래키 — manager 가 1초마다 쓰므로 막 시작한 작업은 잠깐 없을 수 있음)"""
        if not task:
            return None
        with self.sp.pool.connection() as conn:
            return task if conn.execute("SELECT 1 FROM tasks WHERE task_id = %s", (task,)).fetchone() else None

    def on_status(self, rid: str, m: RobotStatus):
        try:
            self.task[rid] = self.known_task(m.task_id)
            state = NAV_STATE[m.state] if m.state < len(NAV_STATE) else "UNKNOWN"
            self.docked[rid] = state == "DOCKED"
            if not m.task_id or state == "DOCKED":         # 작업이 없으면 지난 경로는 지움
                with self.sp.MEM_LOCK:
                    self.sp.ROBOT_PLAN.pop(rid, None)
            body = {"stamp": iso(m.stamp), "run_id": m.run_id or None, "task_id": self.task[rid],
                    "nav_state": state, "progress": m.progress, "x": m.x, "y": m.y, "yaw": m.yaw,
                    "battery_pct": m.battery_percent, "is_docked": state == "DOCKED", "estop": state == "ESTOP"}
            if rid not in self.sp.ROBOT_STATE:             # 처음 받거나 끊겼다 다시 옴 → 활성
                self.sp.set_robot_active(rid, True)
            self.sp.store_status(rid, body, None)
        except Exception as e:                             # 한 건 실패로 구독을 멈추지 않음
            self.get_logger().warning(f"status {rid}: {e}")

    # ------------------------------------------------------------ B3 알림
    def on_alert(self, m: Alert):
        try:
            self.refresh_run()
            run_id = self.sp.CURRENT_RUN["run_id"]
            rid = m.robot_id
            if m.code in ("E7", "RESUMED", "RUN_COMPLETED", "RUN_ABORTED"):  # 다른 브라우저에서 누른 것도 맞춤
                self.sp.ESTOP["on"] = m.code == "E7"
            task = self.known_task(m.ref) if m.code in ("E1", "TASK_FAILED") else self.task.get(rid)
            ev = {"event_uid": str(uuid.uuid5(uuid.NAMESPACE_URL, f"{rid}|{m.code}|{m.ref}|{m.stamp.sec}.{m.stamp.nanosec}")),
                  "run_id": run_id, "task_id": task, "code": m.code, "level": LEVEL[min(m.level, 2)],
                  "detail": {"ref": m.ref, "message": m.message, "delayed": m.delayed, "from": "inspection_manager"},
                  "ts": datetime.fromisoformat(iso(m.stamp))}
            with self.sp.pool.connection() as conn:
                with conn.transaction():
                    to_db = False
                    if m.code == "E7" and not rid and run_id:   # 비상정지는 회차 전체 — 그 회차 로봇마다 한 줄 (robot_id NOT NULL)
                        r = conn.execute("SELECT sector_robots FROM runs WHERE run_id = %s", (run_id,)).fetchone()
                        for each in sorted(set((r and r["sector_robots"] or {}).values()) - {None, ""}):
                            uid = str(uuid.uuid5(uuid.NAMESPACE_URL, f"{each}|E7|{m.stamp.sec}.{m.stamp.nanosec}"))
                            self.sp.add_event(conn, each, ev | {"event_uid": uid, "task_id": self.task.get(each),
                                                                "source": "robot"})
                            to_db = True
                    elif rid and m.code in DB_CODES:
                        to_db = bool(task) or m.code not in NEED_TASK
                    if to_db and rid:                           # E7 은 위에서 로봇마다 넣음 (rid "" 로 또 넣으면 FK 위반 · 롤백)
                        self.sp.add_event(conn, rid, ev | {"source": "control" if m.code in CONTROL_CODES else "robot"})
            if not to_db:                                  # DB 규칙 밖 (RUN_STARTED · KEEPOUT_BLOCKED …) — 화면에만
                ts = ev["ts"].isoformat()
                self.sp.remember_event(ev | {"event_id": next(self.local_id), "robot_id": rid or "-", "source": "control",
                                             "ts": ts})
        except Exception as e:
            self.get_logger().warning(f"alert {m.code}: {e}")

    # ------------------------------------------------------------ B4 판정
    def on_result(self, rid: str, m: InspectionResult, tries: int = 0):
        try:
            if not (m.run_id and m.task_id):
                return
            if not self.known_task(m.task_id):              # manager 가 작업 줄을 아직 안 썼으면 2초 뒤 다시 (최대 5번)
                if tries < 5:
                    threading.Timer(2.0, self.on_result, (rid, m, tries + 1)).start()
                return
            uid = str(uuid.uuid5(uuid.NAMESPACE_URL, f"{rid}|{m.run_id}|{m.task_id}|{m.facility_id}"))
            detail, final = VERDICT.get(m.verdict, ("OCCLUDED", "unconfirmed"))
            try:                                            # 비전이 소화기 ArUco 제조 정보를 detail JSON 에 실어 보냄 (10/15)
                aruco = json.loads(m.detail).get("aruco") if m.detail.startswith("{") else None
            except (ValueError, AttributeError):
                aruco = None
            self.sp.on_inspection(rid, {"inspection_uid": uid, "task_id": m.task_id, "facility_id": m.facility_id,
                                        "detail": detail, "final": final, "frontal": m.verdict != 5, "pose_retry": 0,
                                        "model_version": "yolo26n", "inspected_at": iso(m.stamp), "aruco": aruco})
            if m.image.data and self.sp.MAIN_LOOP is not None:
                meta = {"inspection_uid": uid, "index": 1, "kind": "rgb", "captured_at": iso(m.stamp)}
                asyncio.run_coroutine_threadsafe(self.sp.on_media(rid, meta, bytes(m.image.data)), self.sp.MAIN_LOOP)
        except Exception as e:
            self.get_logger().warning(f"inspect/result {rid} {m.facility_id}: {e}")

    # ------------------------------------------------------------ B6 표본
    def on_sample(self, rid: str, m: NetSample):
        if self.docked.get(rid):                         # 미검출도 스캔 시도 1회로 보존한다
            return
        with self.lock:
            self.samples.setdefault(rid, []).append(
                {"seq": m.seq, "stamp": iso(m.stamp), "x": m.x, "y": m.y, "yaw": m.yaw, "pose_ok": m.pose_ok,
                 "detected": bool(m.detected), "rssi_dbm": m.rssi_dbm if m.detected else None,
                 "task_id": self.task.get(rid)})

    def flush_samples(self):
        with self.lock:
            batches, self.samples = self.samples, {}
        if not batches:
            return
        # 표본의 run_id 는 쓰지 않는다 — manual_tagger 는 run_id 파라미터(기본 1)를 고정으로 찍는다.
        # cell_map_writer 처럼 지금 진행 중인 회차에 붙이고, 진행 중인 회차가 없으면 버린다.
        try:
            with self.sp.pool.connection() as conn:
                r = conn.execute("SELECT run_id FROM runs WHERE status = 'running'").fetchone()
        except Exception as e:
            self.get_logger().warning(f"samples: 진행 중 회차 조회 실패: {e}")
            return
        if not r:
            return
        run_id = r["run_id"]
        for rid, rows in batches.items():
            try:
                ssid = self.target.get(run_id)
                if ssid is None:                           # NetSample 에 SSID 가 없어 그 회차 지도의 측정 AP 로
                    with self.sp.pool.connection() as conn:
                        r = conn.execute("SELECT a.ssid FROM aps a JOIN runs r USING (map_id) "
                                         "WHERE r.run_id = %s AND a.is_target", (run_id,)).fetchone()
                    ssid = self.target[run_id] = r and r["ssid"]
                if not ssid:
                    continue
                self.sp.on_samples(rid, run_id, [s | {"ssid": ssid} for s in rows])
                self.sp.mark_aps_seen(rid, [ssid])
            except Exception as e:
                self.get_logger().warning(f"samples {rid} run {run_id}: {e}")


BRIDGE: Bridge | None = None


def start(sp) -> Bridge:
    """server_page lifespan 에서 부름 — rclpy 를 신호 처리 없이 켜고(uvicorn 이 맡음) 실행기를 데몬 스레드로"""
    global BRIDGE
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    BRIDGE = Bridge(sp)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(BRIDGE)
    threading.Thread(target=ex.spin, name="ros_bridge", daemon=True).start()
    print("[ros_bridge] 켜짐 — status · alerts · inspect/result · survey/sample 구독, /inspection/* 클라이언트", flush=True)
    return BRIDGE
