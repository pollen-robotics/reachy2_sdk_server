import os
from typing import Iterator

import grpc
import rclpy
import reachy2_sdk_api

from google.protobuf.empty_pb2 import Empty
from reachy2_sdk_api.reachy_pb2 import (
    ReachyCoreMode,
)
from reachy2_sdk_api.mujoco_pb2 import (
    MujocoObjectPose,
    MujocoObjectsPoses,
)

from reachy2_sdk_api.mujoco_pb2_grpc import add_MujocoServiceServicer_to_server

from ..abstract_bridge_node import AbstractBridgeNode


class MujocoServicer:
    def __init__(
        self,
        bridge_node: AbstractBridgeNode,
        logger: rclpy.impl.rcutils_logger.RcutilsLogger,
    ):
        self.bridge_node = bridge_node
        self.logger = logger

    def register_to_server(self, server: grpc.Server):
        self.logger.info("Registering 'MujocoServiceServicer' to server.")
        add_MujocoServiceServicer_to_server(self, server)

    def GetObjectsPoses(self, request: Empty, context: grpc.ServicerContext) -> MujocoObjectsPoses:
        response = MujocoObjectsPoses()

        for obj_name, pose_matrix in self.bridge_node.mujoco_objects_poses.items():
            obj_pose = MujocoObjectPose()
            obj_pose.name = obj_name
            obj_pose.pose.pose.data.extend(pose_matrix.flatten())
            response.poses.append(obj_pose)

        return response
