#!/usr/bin/env python3
"""PySide6 ROS 2 remote control GUI for rc_arm_2."""

from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import threading
import time
import uuid
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from PySide6.QtCore import QObject, QSettings, Qt, QTimer, Signal, Slot
from PySide6.QtGui import QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QPlainTextEdit,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, Int32, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
import tf2_ros


REMOTE_SERVICE_ACTIONS = {
    "start_mujoco": "/rc_arm_2/remote/start_mujoco",
    "stop_mujoco": "/rc_arm_2/remote/stop_mujoco",
    "start_real": "/rc_arm_2/remote/start_real",
    "stop_real": "/rc_arm_2/remote/stop_real",
    "start_middleware": "/rc_arm_2/remote/start_middleware",
    "stop_middleware": "/rc_arm_2/remote/stop_middleware",
}
REMOTE_LOG_TOPIC = "/rc_arm_2/remote/log"
REMOTE_PROCESS_STATUS_TOPIC = "/rc_arm_2/remote/process_status"
REMOTE_REACHABILITY_REQUEST_TOPIC = "/rc_arm_2/remote/reachability_request"
REMOTE_REACHABILITY_RESULT_TOPIC = "/rc_arm_2/remote/reachability_result"
NEON_CONSOLE_STYLESHEET = """
QMainWindow {
    background: #030712;
}
QScrollArea {
    background: #030712;
    border: none;
}
QWidget#contentRoot {
    background: #030712;
}
QGroupBox {
    color: #72f8ff;
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
        stop:0 rgba(7, 18, 34, 245),
        stop:1 rgba(3, 10, 20, 250));
    border: 1px solid rgba(0, 234, 255, 95);
    border-radius: 4px;
    margin-top: 18px;
    padding: 12px;
    font: 800 14px "Cascadia Code", "Liberation Mono", monospace;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 12px;
    padding: 0 8px;
    color: #72f8ff;
    background: #030712;
}
QLabel {
    color: #eafcff;
    font-size: 15px;
}
QFormLayout QLabel {
    color: #9ab8c6;
}
QDoubleSpinBox, QSpinBox {
    color: #f2fdff;
    background: #020812;
    border: 1px solid rgba(0, 234, 255, 110);
    border-radius: 3px;
    min-height: 34px;
    padding: 4px 8px;
    font: 800 17px "Cascadia Code", "Liberation Mono", monospace;
}
QDoubleSpinBox:focus, QSpinBox:focus {
    border: 1px solid #ff2bf3;
}
QCheckBox {
    color: #9ab8c6;
    font-size: 15px;
    spacing: 8px;
}
QCheckBox#languageToggle {
    color: #72f8ff;
    font: 800 12px "Cascadia Code", "Liberation Mono", monospace;
    spacing: 6px;
    padding: 0 4px;
}
QCheckBox#languageToggle::indicator {
    width: 34px;
    height: 16px;
    border-radius: 8px;
    border: 1px solid rgba(0, 234, 255, 135);
    background: rgba(0, 234, 255, 18);
}
QCheckBox#languageToggle::indicator:checked {
    border: 1px solid #ff2bf3;
    background: rgba(255, 43, 243, 120);
}
QPushButton {
    min-height: 38px;
    color: #f2fdff;
    background: rgba(0, 234, 255, 22);
    border: 1px solid rgba(0, 234, 255, 110);
    border-radius: 3px;
    padding: 7px 10px;
    font: 800 13px "Segoe UI", Arial, sans-serif;
    text-transform: uppercase;
}
QPushButton:hover {
    border: 1px solid #00eaff;
    background: rgba(0, 234, 255, 42);
}
QPushButton:focus {
    border: 1px solid #ffffff;
    background: rgba(255, 255, 255, 22);
}
QPushButton:pressed {
    color: #020812;
    border: 1px solid #ffffff;
    background: #72f8ff;
}
QPushButton:disabled {
    color: #40505d;
    border-color: rgba(64, 80, 93, 105);
    background: #060b14;
}
QPushButton[role="primary"] {
    border: 1px solid #00eaff;
    background: rgba(0, 234, 255, 58);
}
QPushButton[role="primary"]:pressed {
    color: #020812;
    border: 1px solid #b8fbff;
    background: #00eaff;
}
QPushButton[role="primary"]:disabled {
    color: #36525c;
    border: 1px solid rgba(0, 234, 255, 42);
    background: rgba(0, 234, 255, 10);
}
QPushButton[role="safe"] {
    color: #deffe9;
    border: 1px solid #39ff88;
    background: rgba(57, 255, 136, 34);
}
QPushButton[role="safe"]:pressed {
    color: #021109;
    border: 1px solid #bbffd4;
    background: #39ff88;
}
QPushButton[role="safe"]:disabled {
    color: #365045;
    border: 1px solid rgba(57, 255, 136, 38);
    background: rgba(57, 255, 136, 9);
}
QPushButton[role="danger"] {
    color: #ffe1e8;
    border: 1px solid #ff2f5f;
    background: rgba(255, 47, 95, 36);
}
QPushButton[role="danger"]:pressed {
    color: #19020a;
    border: 1px solid #ffc0cf;
    background: #ff2f5f;
}
QPushButton[role="danger"]:disabled {
    color: #5c3942;
    border: 1px solid rgba(255, 47, 95, 38);
    background: rgba(255, 47, 95, 9);
}
QPushButton[role="warn"] {
    color: #fff0b6;
    border: 1px solid #ffd23f;
    background: rgba(255, 210, 63, 34);
}
QPushButton[role="warn"]:pressed {
    color: #171004;
    border: 1px solid #fff0b6;
    background: #ffd23f;
}
QPushButton[role="warn"]:disabled {
    color: #5c5135;
    border: 1px solid rgba(255, 210, 63, 40);
    background: rgba(255, 210, 63, 9);
}
QPushButton[role="magenta"] {
    border: 1px solid #ff2bf3;
    background: rgba(255, 43, 243, 34);
}
QPushButton[role="magenta"]:pressed {
    color: #170216;
    border: 1px solid #ffc2fb;
    background: #ff2bf3;
}
QPushButton[role="magenta"]:disabled {
    color: #5c3959;
    border: 1px solid rgba(255, 43, 243, 38);
    background: rgba(255, 43, 243, 9);
}
QPlainTextEdit#logView {
    color: #c8fbff;
    background: #02060c;
    border: 1px solid rgba(0, 234, 255, 115);
    border-radius: 3px;
    padding: 10px;
    font: 15px "Cascadia Code", "Liberation Mono", monospace;
}
"""


def set_button_role(button: QPushButton, role: str) -> None:
    button.setProperty("role", role)


UI_TEXT = {
    "target_editor": {"en": "Target Editor", "zh": "目标编辑"},
    "system_control": {"en": "System Control", "zh": "系统控制"},
    "status": {"en": "Status", "zh": "状态"},
    "log": {"en": "Log", "zh": "日志"},
    "j4_world": {"en": "j4 world (deg)", "zh": "j4 世界角 (deg)"},
    "xyz_step": {"en": "xyz step", "zh": "XYZ 步长"},
    "j4_step": {"en": "j4 step", "zh": "J4 步长"},
    "j5_target": {"en": "J5 target (m)", "zh": "J5 目标 (m)"},
    "send_j5": {"en": "Send J5", "zh": "发送 J5"},
    "use_actual": {"en": "Use actual", "zh": "使用实际值"},
    "send_if_changed": {"en": "Send if changed only", "zh": "仅变化时发送"},
    "send": {"en": "Send", "zh": "发送"},
    "reset_xyz": {"en": "Reset XYZ to current", "zh": "XYZ 重置到当前"},
    "request_start_mujoco": {"en": "Request Start MuJoCo", "zh": "请求启动 MuJoCo"},
    "request_stop_mujoco": {"en": "Request Stop MuJoCo", "zh": "请求停止 MuJoCo"},
    "request_start_real": {"en": "Request Start Real", "zh": "请求启动实机"},
    "request_stop_real": {"en": "Request Stop Real", "zh": "请求停止实机"},
    "vacuum_on": {"en": "Vacuum ON", "zh": "吸盘开"},
    "vacuum_off": {"en": "Vacuum OFF", "zh": "吸盘关"},
    "payload_on": {"en": "Payload ON", "zh": "负载开"},
    "payload_off": {"en": "Payload OFF", "zh": "负载关"},
    "action_set_id": {"en": "Action set id", "zh": "动作集 ID"},
    "run_action_set": {"en": "Run Action Set", "zh": "执行动作集"},
    "actual_pose": {"en": "Actual current pose", "zh": "当前实际位姿"},
    "editing_target": {"en": "Editing target", "zh": "编辑目标"},
    "last_sent": {"en": "Last sent target", "zh": "上次发送目标"},
    "last_send_result": {"en": "Last send result", "zh": "上次发送结果"},
    "reachability": {"en": "Reachability", "zh": "可达性"},
    "process_status": {"en": "Process status", "zh": "进程状态"},
    "payload_active": {"en": "Payload active", "zh": "负载状态"},
    "j5_actual": {"en": "J5 actual (m)", "zh": "J5 实际值 (m)"},
}


def normalize_frame_id(frame_id: str) -> str:
    return (frame_id or "").strip().lstrip("/")


def quaternion_from_world_pitch(axis: str, pitch_rad: float) -> Tuple[float, float, float, float]:
    half = 0.5 * float(pitch_rad)
    s = math.sin(half)
    c = math.cos(half)
    if axis == "x":
        return (s, 0.0, 0.0, c)
    if axis == "z":
        return (0.0, 0.0, s, c)
    return (0.0, s, 0.0, c)


@dataclass
class TargetState:
    x: float
    y: float
    z: float
    j4_rad: float

    def to_display(self) -> str:
        return "x={:.4f} y={:.4f} z={:.4f} j4 world={:.2f} deg ({:.4f} rad)".format(
            self.x,
            self.y,
            self.z,
            math.degrees(self.j4_rad),
            self.j4_rad,
        )

    def almost_equal(self, other: "TargetState", tol: float = 1.0e-6) -> bool:
        return (
            abs(self.x - other.x) <= tol
            and abs(self.y - other.y) <= tol
            and abs(self.z - other.z) <= tol
            and abs(self.j4_rad - other.j4_rad) <= tol
        )


@dataclass
class ActualPose:
    x: float
    y: float
    z: float

    def to_display(self) -> str:
        return "x={:.4f} y={:.4f} z={:.4f}".format(self.x, self.y, self.z)


class RemoteRosBackend(QObject):
    actual_pose_updated = Signal(object)
    payload_state_updated = Signal(bool)
    j5_position_updated = Signal(object)
    last_sent_updated = Signal(object)
    send_status = Signal(str)
    command_status = Signal(str)
    process_status_updated = Signal(object)
    reachability_updated = Signal(object)
    log_line = Signal(str)
    backend_error = Signal(str)

    def __init__(self, args) -> None:
        super().__init__()
        self._args = args
        self._node: Optional[Node] = None
        self._tf_pub = None
        self._vacuum_pub = None
        self._payload_command_pub = None
        self._j5_command_pub = None
        self._middleware_run_pub = None
        self._reachability_request_pub = None
        self._tf_buffer = None
        self._tf_listener = None
        self._service_clients: Dict[str, object] = {}
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._pending_send: Optional[Tuple[TargetState, bool]] = None
        self._pending_vacuum: Optional[bool] = None
        self._pending_payload: Optional[bool] = None
        self._pending_j5: Optional[float] = None
        self._pending_action_set: Optional[int] = None
        self._pending_service_action: Optional[str] = None
        self._pending_reachability: Optional[TargetState] = None
        self._last_sent: Optional[TargetState] = None
        self._last_actual_emit = 0.0

    def start(self) -> None:
        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = Node("rc_arm_remote_control_gui")
        self._tf_pub = self._node.create_publisher(TFMessage, self._args.tf_topic, 10)
        self._vacuum_pub = self._node.create_publisher(Bool, self._args.vacuum_topic, 10)
        self._payload_command_pub = self._node.create_publisher(Bool, self._args.payload_command_topic, 10)
        self._j5_command_pub = self._node.create_publisher(Float64, self._args.j5_command_topic, 10)
        self._middleware_run_pub = self._node.create_publisher(Int32, self._args.middleware_run_action_set_topic, 10)
        self._reachability_request_pub = self._node.create_publisher(
            String, REMOTE_REACHABILITY_REQUEST_TOPIC, 10
        )
        self._node.create_subscription(Bool, self._args.payload_active_topic, self._on_payload, 10)
        self._node.create_subscription(Float64, self._args.j5_position_topic, self._on_j5, 20)
        self._node.create_subscription(String, REMOTE_LOG_TOPIC, self._on_remote_log, 50)
        status_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._node.create_subscription(
            String, REMOTE_PROCESS_STATUS_TOPIC, self._on_process_status, status_qos
        )
        self._node.create_subscription(
            String, REMOTE_REACHABILITY_RESULT_TOPIC, self._on_reachability_result, 10
        )
        self._service_clients = {
            action: self._node.create_client(Trigger, service_name)
            for action, service_name in REMOTE_SERVICE_ACTIONS.items()
        }
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self._node, spin_thread=False)
        self._thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if rclpy.ok():
            rclpy.shutdown()

    @Slot(object, bool)
    def queue_send_target(self, state: object, changed_only: bool) -> None:
        with self._lock:
            self._pending_send = (state, changed_only)

    @Slot(bool)
    def queue_vacuum(self, enabled: bool) -> None:
        with self._lock:
            self._pending_vacuum = enabled

    @Slot(bool)
    def queue_payload(self, enabled: bool) -> None:
        with self._lock:
            self._pending_payload = enabled

    @Slot(float)
    def queue_j5(self, position_m: float) -> None:
        with self._lock:
            self._pending_j5 = float(position_m)

    @Slot(int)
    def queue_action_set(self, action_set_id: int) -> None:
        with self._lock:
            self._pending_action_set = int(action_set_id)

    @Slot(str)
    def queue_service_action(self, action: str) -> None:
        with self._lock:
            self._pending_service_action = action

    @Slot(object)
    def queue_reachability(self, state: object) -> None:
        with self._lock:
            self._pending_reachability = state

    def _spin_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                rclpy.spin_once(self._node, timeout_sec=0.05)
                self._refresh_actual_pose()
                self._flush_send()
                self._flush_bool("_pending_vacuum", self._vacuum_pub, "vacuum")
                self._flush_bool("_pending_payload", self._payload_command_pub, "payload")
                self._flush_j5()
                self._flush_action_set()
                self._flush_service_action()
                self._flush_reachability_request()
            except Exception as exc:  # pragma: no cover
                self.backend_error.emit(str(exc))
                time.sleep(0.1)

    def _refresh_actual_pose(self) -> None:
        if self._tf_buffer is None:
            return
        now = time.time()
        if now - self._last_actual_emit < 0.2:
            return
        self._last_actual_emit = now
        try:
            trans = self._tf_buffer.lookup_transform(
                normalize_frame_id(self._args.current_pose_parent_frame),
                normalize_frame_id(self._args.current_pose_child_frame),
                rclpy.time.Time(),
            )
        except Exception:
            return
        self.actual_pose_updated.emit(
            ActualPose(
                x=float(trans.transform.translation.x),
                y=float(trans.transform.translation.y),
                z=float(trans.transform.translation.z),
            )
        )

    def _flush_send(self) -> None:
        with self._lock:
            pending = self._pending_send
            self._pending_send = None
        if pending is None or self._node is None:
            return
        state, changed_only = pending
        if changed_only and self._last_sent is not None and state.almost_equal(self._last_sent):
            self.send_status.emit("skipped: unchanged target")
            return
        qx, qy, qz, qw = quaternion_from_world_pitch(self._args.j4_axis, state.j4_rad)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self._node.get_clock().now().to_msg()
        tf_msg.header.frame_id = normalize_frame_id(self._args.parent_frame)
        tf_msg.child_frame_id = normalize_frame_id(self._args.child_frame)
        tf_msg.transform.translation.x = state.x
        tf_msg.transform.translation.y = state.y
        tf_msg.transform.translation.z = state.z
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self._tf_pub.publish(TFMessage(transforms=[tf_msg]))
        self._last_sent = TargetState(state.x, state.y, state.z, state.j4_rad)
        self.last_sent_updated.emit(self._last_sent)
        self.send_status.emit("published target to {}".format(self._args.tf_topic))

    def _flush_bool(self, attr_name: str, publisher, label: str) -> None:
        with self._lock:
            pending = getattr(self, attr_name)
            setattr(self, attr_name, None)
        if pending is None or publisher is None:
            return
        msg = Bool()
        msg.data = bool(pending)
        publisher.publish(msg)
        self.command_status.emit("{} command: {}".format(label, "ON" if pending else "OFF"))

    def _flush_j5(self) -> None:
        with self._lock:
            pending = self._pending_j5
            self._pending_j5 = None
        if pending is None or self._j5_command_pub is None:
            return
        msg = Float64()
        msg.data = float(pending)
        self._j5_command_pub.publish(msg)
        self.command_status.emit("published J5 target={:.4f} m".format(float(pending)))

    def _flush_action_set(self) -> None:
        with self._lock:
            pending = self._pending_action_set
            self._pending_action_set = None
        if pending is None or self._middleware_run_pub is None:
            return
        msg = Int32()
        msg.data = int(pending)
        self._middleware_run_pub.publish(msg)
        self.command_status.emit("published action_set={}".format(int(pending)))

    def _flush_service_action(self) -> None:
        with self._lock:
            pending = self._pending_service_action
            self._pending_service_action = None
        if pending is None:
            return
        client = self._service_clients.get(pending)
        if client is None:
            self.command_status.emit("unknown remote service action {}".format(pending))
            return
        if not client.service_is_ready():
            self.command_status.emit("service not ready: {}".format(REMOTE_SERVICE_ACTIONS[pending]))
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda done, action=pending: self._on_service_done(action, done)
        )
        self.command_status.emit("requested {}".format(pending))

    def _on_service_done(self, action: str, future) -> None:
        try:
            response = future.result()
            self.command_status.emit(
                "{}: {} {}".format(
                    action,
                    "ok" if response.success else "failed",
                    response.message,
                ).strip()
            )
        except Exception as exc:
            self.command_status.emit("{} failed: {}".format(action, exc))

    def _flush_reachability_request(self) -> None:
        with self._lock:
            pending = self._pending_reachability
            self._pending_reachability = None
        if pending is None or self._reachability_request_pub is None:
            return
        msg = String()
        msg.data = json.dumps(
            {
                "request_id": uuid.uuid4().hex,
                "x": pending.x,
                "y": pending.y,
                "z": pending.z,
                "j4_rad": pending.j4_rad,
            },
            sort_keys=True,
        )
        self._reachability_request_pub.publish(msg)

    def _on_payload(self, msg: Bool) -> None:
        self.payload_state_updated.emit(bool(msg.data))

    def _on_j5(self, msg: Float64) -> None:
        self.j5_position_updated.emit(float(msg.data))

    def _on_remote_log(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            source = payload.get("source", "remote")
            text = payload.get("text", "")
            self.log_line.emit("[{}] {}".format(source, text))
        except Exception:
            self.log_line.emit(msg.data)

    def _on_process_status(self, msg: String) -> None:
        try:
            self.process_status_updated.emit(json.loads(msg.data))
        except Exception as exc:
            self.backend_error.emit("bad process status JSON: {}".format(exc))

    def _on_reachability_result(self, msg: String) -> None:
        try:
            self.reachability_updated.emit(json.loads(msg.data))
        except Exception as exc:
            self.backend_error.emit("bad reachability JSON: {}".format(exc))


class RemoteControlWindow(QMainWindow):
    def __init__(self, args) -> None:
        super().__init__()
        self._args = args
        self._editing_target = TargetState(0.30, 0.0, 0.30, 0.0)
        self._last_sent: Optional[TargetState] = None
        self._actual_pose: Optional[ActualPose] = None
        self._payload_active = False
        self._latest_j5_position: Optional[float] = None
        self._syncing_editor = False
        self._shutdown_started = False
        self._settings = QSettings("RCArm", "RemoteControlGui")
        self._language = str(self._settings.value("language", "en"))
        if self._language not in ("en", "zh"):
            self._language = "en"
        self._translatable_widgets = []
        self._language_toggle: Optional[QCheckBox] = None

        self.setWindowTitle("RC Arm Remote Control")
        self.setMinimumSize(1280, 760)
        self.resize(1500, 900)
        self.setStyleSheet(NEON_CONSOLE_STYLESHEET)

        self._backend = RemoteRosBackend(args)
        self._backend.actual_pose_updated.connect(self._on_actual_pose)
        self._backend.payload_state_updated.connect(self._on_payload_state)
        self._backend.j5_position_updated.connect(self._on_j5_position)
        self._backend.last_sent_updated.connect(self._on_last_sent)
        self._backend.send_status.connect(self._set_send_status)
        self._backend.command_status.connect(self._append_log)
        self._backend.process_status_updated.connect(self._on_process_status)
        self._backend.reachability_updated.connect(self._on_reachability)
        self._backend.log_line.connect(self._append_log)
        self._backend.backend_error.connect(self._append_log)

        self._reachability_timer = QTimer(self)
        self._reachability_timer.setInterval(300)
        self._reachability_timer.setSingleShot(True)
        self._reachability_timer.timeout.connect(self._request_reachability)

        self._build_ui()
        self._apply_language()
        self._install_shortcuts()
        self._sync_editing_widgets()
        self._backend.start()
        self._request_reachability()

    def _text(self, key: str) -> str:
        return UI_TEXT[key][self._language]

    def _register_text(self, widget, key: str):
        self._translatable_widgets.append((widget, key))
        self._apply_widget_text(widget, key)
        return widget

    def _apply_widget_text(self, widget, key: str) -> None:
        text = self._text(key)
        if isinstance(widget, QGroupBox):
            widget.setTitle(text)
        else:
            widget.setText(text)

    def _build_language_toggle(self) -> QCheckBox:
        toggle = QCheckBox()
        toggle.setObjectName("languageToggle")
        toggle.setToolTip("Switch language / 切换语言")
        toggle.toggled.connect(self._set_language)
        self._language_toggle = toggle
        return toggle

    def _apply_language(self) -> None:
        for widget, key in self._translatable_widgets:
            self._apply_widget_text(widget, key)
        if self._language_toggle is not None:
            self._language_toggle.blockSignals(True)
            self._language_toggle.setChecked(self._language == "zh")
            self._language_toggle.setText("中" if self._language == "zh" else "EN")
            self._language_toggle.blockSignals(False)
        self._settings.setValue("language", self._language)

    @Slot(bool)
    def _set_language(self, checked: bool) -> None:
        self._language = "zh" if checked else "en"
        self._apply_language()

    def _build_ui(self) -> None:
        central = QWidget()
        central.setObjectName("contentRoot")
        root = QVBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(12)
        top = QHBoxLayout()
        bottom = QHBoxLayout()
        top.setSpacing(12)
        bottom.setSpacing(12)
        root.addLayout(top, stretch=3)
        root.addLayout(bottom, stretch=2)
        footer = QHBoxLayout()
        footer.addWidget(self._build_language_toggle(), stretch=0)
        footer.addStretch(1)
        root.addLayout(footer, stretch=0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)
        scroll.setWidget(central)
        self.setCentralWidget(scroll)

        top.addWidget(self._build_target_editor(), stretch=5)
        top.addWidget(self._build_system_panel(), stretch=3)
        bottom.addWidget(self._build_status_panel(), stretch=4)
        bottom.addWidget(self._build_log_panel(), stretch=6)

    def _build_target_editor(self) -> QWidget:
        box = self._register_text(QGroupBox(), "target_editor")
        layout = QGridLayout(box)
        self._xyz_step_spin = QDoubleSpinBox()
        self._xyz_step_spin.setDecimals(3)
        self._xyz_step_spin.setRange(0.001, 1.0)
        self._xyz_step_spin.setValue(0.05)
        self._j4_step_spin = QDoubleSpinBox()
        self._j4_step_spin.setDecimals(1)
        self._j4_step_spin.setRange(0.1, 180.0)
        self._j4_step_spin.setValue(5.0)
        self._field_spins = {}
        for row, (field_key, display_name, unit) in enumerate(
            [("x", "x", "m"), ("y", "y", "m"), ("z", "z", "m"), ("j4", "j4 world", "deg")]
        ):
            spin = QDoubleSpinBox()
            spin.setDecimals(4 if field_key != "j4" else 2)
            spin.setRange(-10.0, 10.0)
            if field_key == "j4":
                spin.setRange(-180.0, 180.0)
            spin.valueChanged.connect(self._on_editing_changed)
            minus = QPushButton("-")
            plus = QPushButton("+")
            minus.clicked.connect(lambda _=False, axis=field_key: self._step_axis(axis, -1.0))
            plus.clicked.connect(lambda _=False, axis=field_key: self._step_axis(axis, 1.0))
            label = self._register_text(QLabel(), "j4_world") if field_key == "j4" else QLabel("{} ({})".format(display_name, unit))
            layout.addWidget(label, row, 0)
            layout.addWidget(minus, row, 1)
            layout.addWidget(spin, row, 2)
            layout.addWidget(plus, row, 3)
            self._field_spins[field_key] = spin

        self._send_if_changed = self._register_text(QCheckBox(), "send_if_changed")
        self._send_if_changed.setChecked(True)
        self._j5_target_spin = QDoubleSpinBox()
        self._j5_target_spin.setDecimals(4)
        self._j5_target_spin.setRange(-10.0, 10.0)
        self._j5_target_spin.setSingleStep(0.001)
        self._j5_target_spin.setSuffix(" m")
        send_btn = self._register_text(QPushButton(), "send")
        set_button_role(send_btn, "primary")
        send_btn.clicked.connect(self._send_target)
        reset_btn = self._register_text(QPushButton(), "reset_xyz")
        reset_btn.clicked.connect(self._reset_xyz_to_current)
        send_j5_btn = self._register_text(QPushButton(), "send_j5")
        set_button_role(send_j5_btn, "magenta")
        send_j5_btn.clicked.connect(self._send_j5)
        use_j5_btn = self._register_text(QPushButton(), "use_actual")
        use_j5_btn.clicked.connect(self._use_actual_j5)

        layout.addWidget(self._register_text(QLabel(), "xyz_step"), 4, 0)
        layout.addWidget(self._xyz_step_spin, 4, 2)
        layout.addWidget(self._register_text(QLabel(), "j4_step"), 5, 0)
        layout.addWidget(self._j4_step_spin, 5, 2)
        layout.addWidget(self._register_text(QLabel(), "j5_target"), 6, 0)
        layout.addWidget(self._j5_target_spin, 6, 2)
        layout.addWidget(send_j5_btn, 6, 3)
        layout.addWidget(use_j5_btn, 7, 2, 1, 2)
        layout.addWidget(self._send_if_changed, 8, 0, 1, 4)
        layout.addWidget(send_btn, 9, 0, 1, 2)
        layout.addWidget(reset_btn, 9, 2, 1, 2)
        return box

    def _build_system_panel(self) -> QWidget:
        box = self._register_text(QGroupBox(), "system_control")
        layout = QVBoxLayout(box)
        for key, action, role in [
            ("request_start_mujoco", "start_mujoco", "safe"),
            ("request_stop_mujoco", "stop_mujoco", "danger"),
            ("request_start_real", "start_real", "safe"),
            ("request_stop_real", "stop_real", "danger"),
        ]:
            button = self._register_text(QPushButton(), key)
            set_button_role(button, role)
            button.clicked.connect(lambda _=False, a=action: self._backend.queue_service_action(a))
            layout.addWidget(button)

        vacuum_on = self._register_text(QPushButton(), "vacuum_on")
        vacuum_off = self._register_text(QPushButton(), "vacuum_off")
        payload_on = self._register_text(QPushButton(), "payload_on")
        payload_off = self._register_text(QPushButton(), "payload_off")
        set_button_role(vacuum_on, "warn")
        set_button_role(payload_on, "warn")
        vacuum_on.clicked.connect(lambda: self._backend.queue_vacuum(True))
        vacuum_off.clicked.connect(lambda: self._backend.queue_vacuum(False))
        payload_on.clicked.connect(lambda: self._backend.queue_payload(True))
        payload_off.clicked.connect(lambda: self._backend.queue_payload(False))
        for widget in (vacuum_on, vacuum_off, payload_on, payload_off):
            layout.addWidget(widget)

        self._action_set_spin = QSpinBox()
        self._action_set_spin.setRange(1, 999)
        self._action_set_spin.setValue(1)
        run_action = self._register_text(QPushButton(), "run_action_set")
        set_button_role(run_action, "primary")
        run_action.clicked.connect(lambda: self._backend.queue_action_set(self._action_set_spin.value()))
        action_row = QHBoxLayout()
        action_row.addWidget(self._register_text(QLabel(), "action_set_id"))
        action_row.addWidget(self._action_set_spin)
        layout.addLayout(action_row)
        layout.addWidget(run_action)
        layout.addStretch(1)
        return box

    def _build_status_panel(self) -> QWidget:
        box = self._register_text(QGroupBox(), "status")
        layout = QFormLayout(box)
        self._actual_label = QLabel("waiting for actual pose")
        self._editing_label = QLabel("NA")
        self._last_sent_label = QLabel("NA")
        self._send_status_label = QLabel("idle")
        self._reachability_label = QLabel("Unknown")
        self._process_status_label = QLabel("waiting for host")
        self._payload_status_label = QLabel("false")
        self._j5_actual_label = QLabel("waiting")
        layout.addRow(self._register_text(QLabel(), "actual_pose"), self._actual_label)
        layout.addRow(self._register_text(QLabel(), "editing_target"), self._editing_label)
        layout.addRow(self._register_text(QLabel(), "last_sent"), self._last_sent_label)
        layout.addRow(self._register_text(QLabel(), "last_send_result"), self._send_status_label)
        layout.addRow(self._register_text(QLabel(), "reachability"), self._reachability_label)
        layout.addRow(self._register_text(QLabel(), "process_status"), self._process_status_label)
        layout.addRow(self._register_text(QLabel(), "payload_active"), self._payload_status_label)
        layout.addRow(self._register_text(QLabel(), "j5_actual"), self._j5_actual_label)
        return box

    def _build_log_panel(self) -> QWidget:
        box = self._register_text(QGroupBox(), "log")
        layout = QVBoxLayout(box)
        self._log_view = QPlainTextEdit()
        self._log_view.setObjectName("logView")
        self._log_view.setReadOnly(True)
        self._log_view.setMinimumHeight(320)
        self._log_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout.addWidget(self._log_view)
        return box

    def _install_shortcuts(self) -> None:
        QShortcut(QKeySequence("Ctrl+Return"), self, activated=self._send_target)
        QShortcut(QKeySequence(Qt.Key_Left), self, activated=lambda: self._step_axis("x", -1.0))
        QShortcut(QKeySequence(Qt.Key_Right), self, activated=lambda: self._step_axis("x", 1.0))
        QShortcut(QKeySequence(Qt.Key_Down), self, activated=lambda: self._step_axis("y", -1.0))
        QShortcut(QKeySequence(Qt.Key_Up), self, activated=lambda: self._step_axis("y", 1.0))
        QShortcut(QKeySequence(Qt.Key_PageDown), self, activated=lambda: self._step_axis("z", -1.0))
        QShortcut(QKeySequence(Qt.Key_PageUp), self, activated=lambda: self._step_axis("z", 1.0))
        QShortcut(QKeySequence("["), self, activated=lambda: self._step_axis("j4", -1.0))
        QShortcut(QKeySequence("]"), self, activated=lambda: self._step_axis("j4", 1.0))

    def _step_axis(self, axis: str, direction: float) -> None:
        spin = self._field_spins[axis]
        step = self._xyz_step_spin.value() if axis != "j4" else self._j4_step_spin.value()
        spin.setValue(spin.value() + direction * step)

    def _sync_editing_widgets(self) -> None:
        self._syncing_editor = True
        try:
            self._field_spins["x"].setValue(self._editing_target.x)
            self._field_spins["y"].setValue(self._editing_target.y)
            self._field_spins["z"].setValue(self._editing_target.z)
            self._field_spins["j4"].setValue(math.degrees(self._editing_target.j4_rad))
        finally:
            self._syncing_editor = False
        self._update_status_labels()

    def _read_editing_target(self) -> TargetState:
        return TargetState(
            x=self._field_spins["x"].value(),
            y=self._field_spins["y"].value(),
            z=self._field_spins["z"].value(),
            j4_rad=math.radians(self._field_spins["j4"].value()),
        )

    def _update_status_labels(self) -> None:
        self._editing_label.setText(self._editing_target.to_display())
        self._last_sent_label.setText(self._last_sent.to_display() if self._last_sent else "NA")
        self._actual_label.setText(
            self._actual_pose.to_display() if self._actual_pose else "waiting for actual pose"
        )
        self._payload_status_label.setText("true" if self._payload_active else "false")

    def _on_editing_changed(self) -> None:
        if self._syncing_editor:
            return
        self._editing_target = self._read_editing_target()
        self._update_status_labels()
        self._reachability_timer.start()

    def _request_reachability(self) -> None:
        self._backend.queue_reachability(self._editing_target)

    def _send_target(self) -> None:
        self._editing_target = self._read_editing_target()
        self._backend.queue_send_target(self._editing_target, self._send_if_changed.isChecked())

    def _send_j5(self) -> None:
        self._backend.queue_j5(self._j5_target_spin.value())

    def _use_actual_j5(self) -> None:
        if self._latest_j5_position is not None:
            self._j5_target_spin.setValue(self._latest_j5_position)

    def _reset_xyz_to_current(self) -> None:
        if self._actual_pose is None:
            return
        current = self._actual_pose
        self._editing_target = TargetState(
            current.x,
            current.y,
            current.z,
            self._editing_target.j4_rad,
        )
        self._sync_editing_widgets()
        self._reachability_timer.start()

    @Slot(object)
    def _on_actual_pose(self, pose: object) -> None:
        self._actual_pose = pose
        self._update_status_labels()

    @Slot(object)
    def _on_last_sent(self, state: object) -> None:
        self._last_sent = state
        self._update_status_labels()

    @Slot(bool)
    def _on_payload_state(self, active: bool) -> None:
        self._payload_active = active
        self._update_status_labels()

    @Slot(object)
    def _on_j5_position(self, position_m: object) -> None:
        self._latest_j5_position = float(position_m)
        self._j5_actual_label.setText("{:.4f}".format(self._latest_j5_position))

    @Slot(str)
    def _set_send_status(self, text: str) -> None:
        self._send_status_label.setText(text)
        self._append_log(text)

    @Slot(object)
    def _on_process_status(self, payload: object) -> None:
        summary = str(payload.get("summary", "unknown")) if isinstance(payload, dict) else str(payload)
        self._process_status_label.setText(summary)

    @Slot(object)
    def _on_reachability(self, payload: object) -> None:
        if not isinstance(payload, dict):
            self._reachability_label.setText(str(payload))
            return
        status = payload.get("status", "Unknown")
        ranges = payload.get("ranges", {})
        suffix = ""
        if isinstance(ranges, dict) and ranges:
            suffix = " x={} y={} z={}".format(
                ranges.get("x", "NA"),
                ranges.get("y", "NA"),
                ranges.get("z", "NA"),
            )
        self._reachability_label.setText("{}{}".format(status, suffix))

    @Slot(str)
    def _append_log(self, text: str) -> None:
        if text:
            self._log_view.appendPlainText(text)

    def shutdown(self) -> None:
        if self._shutdown_started:
            return
        self._shutdown_started = True
        self._backend.stop()

    def closeEvent(self, event) -> None:  # noqa: N802
        self.shutdown()
        super().closeEvent(event)


def parse_args():
    parser = argparse.ArgumentParser(description="PySide6 ROS 2 remote control GUI")
    parser.add_argument("--tf-topic", default="/tf")
    parser.add_argument("--parent-frame", default="world")
    parser.add_argument("--child-frame", default="rc_arm_2_target")
    parser.add_argument("--current-pose-parent-frame", default="world")
    parser.add_argument("--current-pose-child-frame", default="end_effector")
    parser.add_argument("--vacuum-topic", default="/rc_arm_2/vacuum_activate")
    parser.add_argument("--payload-command-topic", default="/rc_arm_2/payload_active_command")
    parser.add_argument("--payload-active-topic", default="/rc_arm_2/payload_active")
    parser.add_argument("--j5-command-topic", default="/rc_arm_2/j5/command_position")
    parser.add_argument("--j5-position-topic", default="/rc_arm_2/j5/actual_position")
    parser.add_argument("--middleware-run-action-set-topic", default="/arm2/middleware/run_action_set")
    parser.add_argument("--j4-axis", choices=["x", "y", "z"], default="y")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    app = QApplication(sys.argv)
    window = RemoteControlWindow(args)
    app.aboutToQuit.connect(window.shutdown)
    signal.signal(signal.SIGINT, lambda *_args: app.quit())
    window.show()
    signal_timer = QTimer()
    signal_timer.start(200)
    signal_timer.timeout.connect(lambda: None)
    try:
        exit_code = app.exec()
    finally:
        window.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
