from typing import Tuple
import rclpy
import time
import yaml

from google.protobuf.timestamp_pb2 import Timestamp
from pollen_msgs.srv import GetDynamicState
from reachy_sdk_api_v2 import orbita2d_pb2


def parse_reachy_config(reachy_config_path: str) -> dict:
    with open(reachy_config_path, "r") as f:
        config = yaml.safe_load(f)
    return config


def get_uid_from_name(name: str, node: rclpy.node.Node) -> int:
    c = node.create_client(GetDynamicState, f"/get_dynamic_state")

    while not c.wait_for_service(timeout_sec=1.0):
        node.get_logger().info(
            f"Service '{c.srv_name}' not available, waiting again..."
        )

    req = GetDynamicState.Request()
    req.name = name
    req.interfaces = ["uid"]

    future = c.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    node.destroy_client(c)

    return int(future.result().values[0])


def get_component_full_state(component_name: str, node: rclpy.node.Node) -> dict:
    c = node.create_client(GetDynamicState, f"/get_dynamic_state")

    while not c.wait_for_service(timeout_sec=1.0):
        node.get_logger().info(
            f"Service '{c.srv_name}' not available, waiting again..."
        )

    req = GetDynamicState.Request()
    req.name = component_name
    req.interfaces = []

    future = c.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    node.destroy_client(c)

    return dict(zip(future.result().interfaces, future.result().values))


def axis_from_str(name: str) -> orbita2d_pb2.Axis:
    if name == "roll":
        return orbita2d_pb2.Axis.ROLL
    elif name == "pitch":
        return orbita2d_pb2.Axis.PITCH
    elif name == "yaw":
        return orbita2d_pb2.Axis.YAW
    else:
        raise ValueError(f"Unknown axis '{name}'.")


def endless_get_stream(func, request, context, period):
    while True:
        yield func(request, context)
        time.sleep(period)


def get_current_timestamp(bridge_node: rclpy.node.Node) -> Timestamp:
    t = Timestamp()
    t.FromNanoseconds(bridge_node.get_clock().now().nanoseconds)

    return t


def cleanup_fields(FieldEnum, fields):
    if FieldEnum.NONE in fields:
        cleanup_fields = []
    elif FieldEnum.ALL in fields:
        cleanup_fields = list(FieldEnum.keys())
        cleanup_fields.remove("ALL")
        cleanup_fields.remove("NONE")
    else:
        cleanup_fields = [
            FieldEnum.DESCRIPTOR.values_by_number[field].name for field in fields
        ]

    cleanup_fields = [f.lower() for f in cleanup_fields]

    return cleanup_fields


def extract_fields(FieldEnum, fields, conversion_table, component) -> dict:
    grpc_state = {}

    for f in cleanup_fields(FieldEnum, fields):
        if f not in conversion_table:
            continue
        grpc_state[f] = conversion_table[f](component)

    return grpc_state
