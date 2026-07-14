#!/usr/bin/env python3

import json
from typing import Any

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from rc26_interfaces.msg import MechanismTransportFeedback
from rc26_interfaces.srv import SendMechanismTransportCommand
from std_msgs.msg import String


def parse_byte_value(value: Any) -> int:
    if isinstance(value, str):
        return int(value, 0) & 0xFF
    return int(value) & 0xFF


def make_dynamic_parameter_descriptor() -> ParameterDescriptor:
    return ParameterDescriptor(dynamic_typing=True)


def startup_ready_should_notify(
    *,
    gate_waiting: bool,
    service_ready: bool,
    limit_already_seen: bool,
    sent: bool,
    done: bool,
) -> bool:
    return (
        gate_waiting
        and service_ready
        and not limit_already_seen
        and not sent
        and not done
    )


class StartupReadyNotifyNode(Node):
    def __init__(self) -> None:
        super().__init__("startup_ready_notify_node")

        self.command_id = parse_byte_value(
            self.declare_parameter(
                "command_id", 0x20, make_dynamic_parameter_descriptor()
            ).value
        )
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
        self.limit_feedback_ids = [
            parse_byte_value(value)
            for value in self.declare_parameter("limit_feedback_ids", [0x06, 0x10]).value
        ]
        self.timer_period_s = float(self.declare_parameter("timer_period_s", 0.1).value)

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
        self.create_subscription(
            MechanismTransportFeedback, self.feedback_topic, self._on_feedback, 32
        )
        self.create_subscription(String, self.gate_state_topic, self._on_gate_state, gate_qos)
        self.timer = self.create_timer(self.timer_period_s, self._tick)

        self.get_logger().info(
            "启动就绪通知节点等待: "
            f"gate={self.gate_state_topic} service={self.send_command_service} "
            f"command=0x{int(self.command_id) & 0xFF:02X} timeout={self.timeout_s:.1f}s"
        )

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
                f"gate_waiting={self.gate_waiting} service_ready={self.send_client.service_is_ready()} "
                f"limit_seen={self.limit_already_seen}"
            )
            return

        if not startup_ready_should_notify(
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
    from rclpy.executors import ExternalShutdownException

    rclpy.init()
    node = StartupReadyNotifyNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
