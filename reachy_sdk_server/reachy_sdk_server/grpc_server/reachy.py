from typing import Iterator

import grpc
import rclpy
from google.protobuf.empty_pb2 import Empty
from reachy2_sdk_api.part_pb2 import PartId
from reachy2_sdk_api.reachy_pb2 import (
    Reachy,
    ReachyId,
    ReachyState,
    ReachyStatus,
    ReachyStreamAuditRequest,
    ReachyStreamStateRequest,
)
from reachy2_sdk_api.reachy_pb2_grpc import add_ReachyServiceServicer_to_server

from ..abstract_bridge_node import AbstractBridgeNode
from ..utils import endless_timer_get_stream, endless_timer_get_stream_works, get_current_timestamp
from .arm import ArmServicer
from .hand import HandServicer
from .head import HeadServicer
from .mobile_base import MobileBaseServicer


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

    def GetReachyState(self, request: ReachyId, context: grpc.ServicerContext) -> ReachyState:
        if request.id != self.reachy_id.id and request.name != self.reachy_id.name:
            context.abort(grpc.StatusCode.NOT_FOUND, "Reachy not found.")

        params = {
            "timestamp": get_current_timestamp(self.bridge_node),
            "id": self.reachy_id,
        }

        for p in self.bridge_node.parts:
            if p.type == "arm":
                params[f"{p.name}_state"] = self.arm_servicer.GetState(PartId(id=p.id), context)
            elif p.type == "head":
                params[f"{p.name}_state"] = self.head_servicer.GetState(PartId(id=p.id), context)
            elif p.type == "hand":
                params[f"{p.name}_state"] = self.hand_servicer.GetState(PartId(id=p.id), context)

        # params["mobile_base_state"] = self.mobile_base_servicer.GetState(Empty(), context)

        return ReachyState(**params)

    def StreamReachyState(self, request: ReachyStreamStateRequest, context: grpc.ServicerContext) -> Iterator[ReachyState]:
        return endless_timer_get_stream_works(
            self.bridge_node,
            self.GetReachyState,
            request.id,
            context,
            1 / request.publish_frequency,
        )

    def Audit(self, request: ReachyId, context: grpc.ServicerContext) -> ReachyStatus:
        if request.id != self.reachy_id.id and request.name != self.reachy_id.name:
            context.abort(grpc.StatusCode.NOT_FOUND, "Reachy not found.")

        params = {
            "timestamp": get_current_timestamp(self.bridge_node),
            "id": self.reachy_id,
        }

        for p in self.bridge_node.parts:
            if p.type == "arm":
                params[f"{p.name}_status"] = self.arm_servicer.Audit(PartId(id=p.id), context)
            elif p.type == "head":
                params[f"{p.name}_status"] = self.head_servicer.Audit(PartId(id=p.id), context)
            elif p.type == "hand":
                params[f"{p.name}_status"] = self.hand_servicer.Audit(PartId(id=p.id), context)

        # params["mobile_base_state"] = self.mobile_base_servicer.GetState(Empty(), context)

        return ReachyStatus(**params)

    def StreamAudit(self, request: ReachyStreamAuditRequest, context: grpc.ServicerContext) -> Iterator[ReachyStatus]:
        return endless_timer_get_stream_works(
            self.bridge_node,
            self.Audit,
            request.id,
            context,
            1 / request.publish_frequency,
        )
