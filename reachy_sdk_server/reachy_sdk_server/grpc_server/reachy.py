from typing import Iterator
import grpc
import rclpy

from google.protobuf.empty_pb2 import Empty

from reachy2_sdk_api.part_pb2 import PartId
from reachy2_sdk_api.reachy_pb2 import (
    Reachy,
    ReachyId,
    ReachyState,
    ReachyStreamStateRequest,
)

from reachy2_sdk_api.reachy_pb2_grpc import add_ReachyServiceServicer_to_server


from ..abstract_bridge_node import AbstractBridgeNode
from .arm import ArmServicer
from .hand import HandServicer
from .head import HeadServicer
from .mobile_base import MobileBaseServicer
from ..utils import endless_timer_get_stream, get_current_timestamp


class ReachyServicer:
    def __init__(
        self,
        bridge_node: AbstractBridgeNode,
        logger: rclpy.impl.rcutils_logger.RcutilsLogger,
        arm_servicer: ArmServicer,
        hand_servicer: HandServicer,
        head_servicer: HeadServicer,
        mobile_base_servicer: MobileBaseServicer,
    ):
        self.bridge_node = bridge_node
        self.logger = logger

        self.arm_servicer = arm_servicer
        self.hand_servicer = hand_servicer
        self.head_servicer = head_servicer
        self.mobile_base_servicer = mobile_base_servicer

        self.reachy_id = ReachyId(id=1, name="reachy")
        self.active_calls = 0
        self.timer = self.bridge_node.create_timer(0.2, self.print_active_calls)

    def print_active_calls(self):
        self.logger.info(f"self.active_calls={self.active_calls}")

    def register_to_server(self, server: grpc.Server):
        self.logger.info("Registering 'ArmServiceServicer' to server.")
        add_ReachyServiceServicer_to_server(self, server)

    def GetReachy(self, request: Empty, context: grpc.ServicerContext) -> Reachy:
        params = {
            "id": self.reachy_id,
        }

        for p in self.bridge_node.parts:
            if p.type == "arm":
                params[p.name] = self.arm_servicer.get_arm(p, context)
            elif p.type == "head":
                params[p.name] = self.head_servicer.get_head(p, context)
            elif p.type == "hand":
                params[p.name] = self.hand_servicer.get_hand(p, context)

        if self.mobile_base_servicer.get_mobile_base(context) is not None:
            params["mobile_base"] = self.mobile_base_servicer.get_mobile_base(context)

        return Reachy(**params)

    def GetReachyState(
        self, request: ReachyId, context: grpc.ServicerContext
    ) -> ReachyState:
        self.active_calls+=1 
        if request.id != self.reachy_id.id and request.name != self.reachy_id.name:
            context.abort(grpc.StatusCode.NOT_FOUND, "Reachy not found.")

        params = {
            "timestamp": get_current_timestamp(self.bridge_node),
            "id": self.reachy_id,
        }

        for p in self.bridge_node.parts:
            if p.type == "arm":
                params[f"{p.name}_state"] = self.arm_servicer.GetState(
                    PartId(id=p.id), context
                )
            elif p.type == "head":
                params[f"{p.name}_state"] = self.head_servicer.GetState(
                    PartId(id=p.id), context
                )
            elif p.type == "hand":
                params[f"{p.name}_state"] = self.hand_servicer.GetState(
                    PartId(id=p.id), context
                )
        """
        params["mobile_base_state"] = self.mobile_base_servicer.GetState(
            Empty(), context
        )
        """
        self.active_calls-=1
        return ReachyState(**params)

    def StreamReachyState(
        self, request: ReachyStreamStateRequest, context: grpc.ServicerContext
    ) -> Iterator[ReachyState]:
        return endless_timer_get_stream_works(
            self.bridge_node,
            self.GetReachyState,
            request.id,
            context,
            1 / request.publish_frequency,
        )
