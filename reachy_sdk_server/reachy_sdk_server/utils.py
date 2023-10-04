import rclpy
import yaml

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


def axis_from_str(name: str) -> orbita2d_pb2.Axis:
    if name == "roll":
        return orbita2d_pb2.Axis.ROLL
    elif name == "pitch":
        return orbita2d_pb2.Axis.PITCH
    elif name == "yaw":
        return orbita2d_pb2.Axis.YAW
    else:
        raise ValueError(f"Unknown axis '{name}'.")
