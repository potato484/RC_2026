#!/usr/bin/env python3

import json
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from rc26_interfaces.msg import MechanismTransportFeedback
from rc26_interfaces.srv import SendMechanismTransportCommand
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


def parse_byte_value(value: Any) -> int:
    if isinstance(value, str):
        return int(value, 0) & 0xFF
    return int(value) & 0xFF


def startup_ready_should_notify(
    *,
    have_color: bool,
    have_depth: bool,
    have_info: bool,
    gate_waiting: bool,
    service_ready: bool,
    limit_already_seen: bool,
    sent: bool,
    done: bool,
) -> bool:
    return (
        have_color
        and have_depth
        and have_info
        and gate_waiting
        and service_ready
        and not limit_already_seen
        and not sent
        and not done
    )


class StartupReadyNotifyNode(Node):
    def __init__(self) -> None:
        super().__init__("startup_ready_notify_node")

        self.command_id = parse_byte_value(self.declare_parameter("command_id", 0x20).value)
        self.timeout_s = float(self.declare_parameter("timeout_s", 30.0).value)
        self.send_command_service = str(
            self.declare_parameter("send_command_service", "/mechanism/send_command").value
        )
        self.feedback_topic = str(
            self.declare_parameter("feedback_topic", "/mechanism/command_feedback").value
        )
        self.gate_state_topic = str(
            self.declare_parameter("gate_state_topic", "/decision/preselection_gate_state").value
        )
        self.color_topic = str(
            self.declare_parameter("color_topic", "/camera/color/image_raw").value
        )
        self.depth_topic = str(
            self.declare_parameter(
                "depth_topic", "/camera/aligned_depth_to_color/image_raw"
            ).value
        )
        self.info_topic = str(
            self.declare_parameter("info_topic", "/camera/color/camera_info").value
        )
        self.limit_feedback_ids = [
            int(value)
            for value in self.declare_parameter("limit_feedback_ids", [0x06, 0x10]).value
        ]
        self.timer_period_s = float(self.declare_parameter("timer_period_s", 0.1).value)

        self.have_color = False
        self.have_depth = False
        self.have_info = False
        self.gate_waiting = False
        self.limit_already_seen = False
        self.sent = False
        self.done = False
        self.start_time = self.get_clock().now()

        gate_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.send_client = self.create_client(
            SendMechanismTransportCommand, self.send_command_service
        )
        self.create_subscription(Image, self.color_topic, self._on_color, 1)
        self.create_subscription(Image, self.depth_topic, self._on_depth, 1)
        self.create_subscription(CameraInfo, self.info_topic, self._on_info, 1)
        self.create_subscription(
            MechanismTransportFeedback, self.feedback_topic, self._on_feedback, 32
        )
        self.create_subscription(String, self.gate_state_topic, self._on_gate_state, gate_qos)
        self.timer = self.create_timer(self.timer_period_s, self._tick)

        self.get_logger().info(
            "启动就绪通知节点等待: "
            f"color={self.color_topic} depth={self.depth_topic} info={self.info_topic} "
            f"gate={self.gate_state_topic} service={self.send_command_service} "
            f"command=0x{int(self.command_id) & 0xFF:02X} timeout={self.timeout_s:.1f}s"
        )

    def _on_color(self, _: Image) -> None:
        self.have_color = True

    def _on_depth(self, _: Image) -> None:
        self.have_depth = True

    def _on_info(self, _: CameraInfo) -> None:
        self.have_info = True

    def _on_feedback(self, msg: MechanismTransportFeedback) -> None:
        if int(msg.feedback_id) in self.limit_feedback_ids:
            self.limit_already_seen = True
            if not self.sent:
                self.done = True
                self.get_logger().warn(
                    f"启动就绪通知跳过: 已先收到人工限位 feedback=0x{int(msg.feedback_id) & 0xFF:02X}"
                )

    def _on_gate_state(self, msg: String) -> None:
        try:
            state: dict[str, Any] = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"忽略无法解析的预选 gate 状态: {msg.data}")
            return
        self.gate_waiting = bool(state.get("waiting", False))

    def _tick(self) -> None:
        if self.done or self.sent:
            return

        elapsed_s = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed_s > self.timeout_s:
            self.done = True
            self.get_logger().warn(
                "启动就绪通知超时未发送: "
                f"color={self.have_color} depth={self.have_depth} info={self.have_info} "
                f"gate_waiting={self.gate_waiting} service_ready={self.send_client.service_is_ready()} "
                f"limit_seen={self.limit_already_seen}"
            )
            return

        if not startup_ready_should_notify(
            have_color=self.have_color,
            have_depth=self.have_depth,
            have_info=self.have_info,
            gate_waiting=self.gate_waiting,
            service_ready=self.send_client.service_is_ready(),
            limit_already_seen=self.limit_already_seen,
            sent=self.sent,
            done=self.done,
        ):
            return

        request = SendMechanismTransportCommand.Request()
        request.command_id = int(self.command_id) & 0xFF
        request.payload = []
        request.wait_ack = False
        self.sent = True
        future = self.send_client.call_async(request)
        future.add_done_callback(self._on_send_done)
        self.get_logger().info(
            f"启动就绪通知已发送请求: command=0x{int(self.command_id) & 0xFF:02X} wait_ack=false"
        )

    def _on_send_done(self, future: Any) -> None:
        self.done = True
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - ROS future can surface transport exceptions.
            self.get_logger().error(f"启动就绪通知发送异常: {exc}")
            return
        if response and response.accepted:
            self.get_logger().info(
                f"启动就绪通知发送完成: command=0x{int(self.command_id) & 0xFF:02X} "
                f"seq={int(response.seq)} no-ack"
            )
        else:
            self.get_logger().warn(
                f"启动就绪通知发送被拒绝: command=0x{int(self.command_id) & 0xFF:02X}"
            )


def main() -> None:
    rclpy.init()
    node = StartupReadyNotifyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
