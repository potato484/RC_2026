import importlib.util
import os
import sys
import types
from pathlib import Path


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "startup_ready_notify_node.py"
)


def test_startup_ready_script_is_ros2_executable():
    assert SCRIPT.read_text(encoding="utf-8").startswith("#!/usr/bin/env python3")
    assert os.access(SCRIPT, os.X_OK)


def load_module():
    class DummyNode:
        pass

    class DummyQoSProfile:
        def __init__(self, **kwargs):
            self.kwargs = kwargs

    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda: None
    rclpy.spin = lambda node: None
    rclpy.shutdown = lambda: None
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = DummyNode
    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL=1)
    rclpy_qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    rclpy_qos.QoSProfile = DummyQoSProfile
    rclpy_qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1)

    rc26_interfaces = types.ModuleType("rc26_interfaces")
    rc26_interfaces_msg = types.ModuleType("rc26_interfaces.msg")
    rc26_interfaces_msg.MechanismTransportFeedback = type(
        "MechanismTransportFeedback", (), {}
    )
    rc26_interfaces_srv = types.ModuleType("rc26_interfaces.srv")
    rc26_interfaces_srv.SendMechanismTransportCommand = type(
        "SendMechanismTransportCommand", (), {"Request": type("Request", (), {})}
    )

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.CameraInfo = type("CameraInfo", (), {})
    sensor_msgs_msg.Image = type("Image", (), {})
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = type("String", (), {})

    sys.modules.update(
        {
            "rclpy": rclpy,
            "rclpy.node": rclpy_node,
            "rclpy.qos": rclpy_qos,
            "rc26_interfaces": rc26_interfaces,
            "rc26_interfaces.msg": rc26_interfaces_msg,
            "rc26_interfaces.srv": rc26_interfaces_srv,
            "sensor_msgs": sensor_msgs,
            "sensor_msgs.msg": sensor_msgs_msg,
            "std_msgs": std_msgs,
            "std_msgs.msg": std_msgs_msg,
        }
    )

    spec = importlib.util.spec_from_file_location("startup_ready_notify_node", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_parse_byte_value_accepts_decimal_and_hex_strings():
    module = load_module()

    assert module.parse_byte_value(0x20) == 0x20
    assert module.parse_byte_value("0x20") == 0x20
    assert module.parse_byte_value("32") == 0x20
    assert module.parse_byte_value(0x120) == 0x20


def test_startup_ready_requires_all_inputs_and_service():
    module = load_module()

    assert module.startup_ready_should_notify(
        have_color=True,
        have_depth=True,
        have_info=True,
        gate_waiting=True,
        service_ready=True,
        limit_already_seen=False,
        sent=False,
        done=False,
    )

    assert not module.startup_ready_should_notify(
        have_color=True,
        have_depth=False,
        have_info=True,
        gate_waiting=True,
        service_ready=True,
        limit_already_seen=False,
        sent=False,
        done=False,
    )
    assert not module.startup_ready_should_notify(
        have_color=True,
        have_depth=True,
        have_info=True,
        gate_waiting=False,
        service_ready=True,
        limit_already_seen=False,
        sent=False,
        done=False,
    )
    assert not module.startup_ready_should_notify(
        have_color=True,
        have_depth=True,
        have_info=True,
        gate_waiting=True,
        service_ready=False,
        limit_already_seen=False,
        sent=False,
        done=False,
    )


def test_startup_ready_suppresses_after_limit_or_send():
    module = load_module()

    base = dict(
        have_color=True,
        have_depth=True,
        have_info=True,
        gate_waiting=True,
        service_ready=True,
    )
    assert not module.startup_ready_should_notify(
        **base, limit_already_seen=True, sent=False, done=False
    )
    assert not module.startup_ready_should_notify(
        **base, limit_already_seen=False, sent=True, done=False
    )
    assert not module.startup_ready_should_notify(
        **base, limit_already_seen=False, sent=False, done=True
    )
