from typing import Iterator
import grpc
import rclpy

from google.protobuf.empty_pb2 import Empty

from reachy_sdk_api_v2.reachy_pb2 import (
    ListOfReachy,
    Reachy,
    ReachyId,
    ReachyState,
    ReachyStreamStateRequest,
)
from reachy_sdk_api_v2.reachy_pb2_grpc import add_ReachyServiceServicer_to_server


from ..abstract_bridge_node import AbstractBridgeNode
from .arm import ArmServicer
from ..utils import get_current_timestamp


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

    def GetListOfReachy(
        self, request: Empty, context: grpc.ServicerContext
    ) -> ListOfReachy:
        return ListOfReachy(reachy=[self.GetReachy(request, context)])

    def GetReachy(self, request: Empty, context: grpc.ServicerContext) -> Reachy:
        parts = self.bridge_node.parts

        params = {
            "id": ReachyId(name="reachy"),
        }

        for arm_side in ("r", "l"):
            name = f"{arm_side}_arm"

            try:
                params[name] = self.arm_servicer.get_arm(
                    parts.get_by_name(name), context
                )
            except KeyError:
                pass

        return Reachy(**params)

    def GetReachyState(
        self, request: ReachyId, context: grpc.ServicerContext
    ) -> ReachyState:
        params = {
            "timestamp": get_current_timestamp(self.bridge_node),
            "id": request,
        }

        # TODO: add whrn we have arm.GetState
        # for arm_side in ("r", "l"):
        #     name = f"{arm_side}_arm"

        #     try:
        #         params[name] = self.arm_servicer.GetState()
        #     except KeyError:
        #         pass

        return ReachyState(**params)

    def StreamReachyState(
        self, request: ReachyStreamStateRequest, context: grpc.ServicerContext
    ) -> Iterator[ReachyState]:
        pass
