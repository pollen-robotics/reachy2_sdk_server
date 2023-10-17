from typing import Iterator
import grpc
import rclpy

from google.protobuf.empty_pb2 import Empty

from reachy_sdk_api_v2.part_pb2 import PartId
from reachy_sdk_api_v2.reachy_pb2 import (
    Reachy,
    ReachyId,
    ReachyState,
    ReachyStreamStateRequest,
)
from reachy_sdk_api_v2.reachy_pb2_grpc import add_ReachyServiceServicer_to_server


from ..abstract_bridge_node import AbstractBridgeNode
from .arm import ArmServicer
from ..utils import endless_get_stream, get_current_timestamp


class ReachyServicer:
    def __init__(
        self,
        bridge_node: AbstractBridgeNode,
        logger: rclpy.impl.rcutils_logger.RcutilsLogger,
        arm_servicer: ArmServicer,
    ):
        self.bridge_node = bridge_node
        self.logger = logger
        self.arm_servicer = arm_servicer

    def register_to_server(self, server: grpc.Server):
        self.logger.info("Registering 'ArmServiceServicer' to server.")
        add_ReachyServiceServicer_to_server(self, server)

    def GetReachy(self, request: Empty, context: grpc.ServicerContext) -> Reachy:
        params = {
            "id": ReachyId(id=1, name="reachy"),
        }

        for p in self.bridge_node.parts:
            if p.type == "arm":
                params[p.name] = self.arm_servicer.get_arm(p, context)

        return Reachy(**params)

    def GetReachyState(
        self, request: ReachyId, context: grpc.ServicerContext
    ) -> ReachyState:
        params = {
            "timestamp": get_current_timestamp(self.bridge_node),
            "id": request,
        }

        for p in self.bridge_node.parts:
            if p.type == "arm":
                params[f"{p.name}_state"] = self.arm_servicer.GetState(
                    PartId(id=p.id), context
                )

        return ReachyState(**params)

    def StreamReachyState(
        self, request: ReachyStreamStateRequest, context: grpc.ServicerContext
    ) -> Iterator[ReachyState]:
        return endless_get_stream(
            self.GetReachyState, request.id, context, 1 / request.publish_frequency
        )
