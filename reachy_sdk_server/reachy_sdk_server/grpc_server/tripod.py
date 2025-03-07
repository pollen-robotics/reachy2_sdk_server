import grpc
import rclpy
from control_msgs.msg import DynamicJointState, InterfaceValue
from google.protobuf.empty_pb2 import Empty
from google.protobuf.wrappers_pb2 import FloatValue
from reachy2_sdk_api.component_pb2 import JointLimits
from reachy2_sdk_api.tripod_pb2 import (
    Tripod,
    TripodAxis,
    TripodCommand,
    TripodDescription,
    TripodJoint,
    TripodJointsLimits,
    TripodJointState,
    TripodState,
)
from reachy2_sdk_api.tripod_pb2_grpc import add_TripodServiceServicer_to_server

from ..abstract_bridge_node import AbstractBridgeNode
from ..parts import Part, PartId


class TripodServicer:
    def __init__(
        self,
        bridge_node: AbstractBridgeNode,
        logger: rclpy.impl.rcutils_logger.RcutilsLogger,
    ) -> None:
        self.bridge_node = bridge_node
        self.logger = logger

        self.tripod = self.bridge_node.parts.get_by_type("tripod")[0]
        self.joint = "tripod_joint"
        self.min_height = 0.996
        self.max_height = 1.2

    def register_to_server(self, server: grpc.Server):
        self.logger.info("Registering 'TripodServiceServicer' to server.")
        add_TripodServiceServicer_to_server(self, server)

    def get_tripod(self, tripod: Part, context: grpc.ServicerContext) -> Tripod:
        return Tripod(
            part_id=PartId(name=self.tripod.name, id=self.tripod.id),
            description=TripodDescription(height_joint=TripodJoint(axis=TripodAxis.HEIGHT)),
        )

    def GetTripod(self, request: Empty, context: grpc.ServicerContext) -> Tripod:
        return self.get_tripod(self.tripod, context)

    # State
    def GetState(self, request: PartId, context: grpc.ServicerContext) -> TripodState:
        component = self.bridge_node.components.get_by_name(self.joint)
        tripod_state = TripodState(
            part_id=PartId(name=self.tripod.name, id=self.tripod.id),
            height=TripodJointState(
                joint=TripodJoint(axis=TripodAxis.HEIGHT),
                present_position=FloatValue(value=component.state.get("position", 0) + self.min_height),
                goal_position=FloatValue(value=component.state.get("target_position", 0) + self.min_height),
            ),
        )
        return tripod_state

    # Command
    def SendCommand(self, request: TripodCommand, context: grpc.ServicerContext) -> Empty:
        cmd = DynamicJointState()
        cmd.joint_names = []

        if request.HasField("height_position"):
            cmd.joint_names.append(self.joint)
            cmd.interface_values.append(
                InterfaceValue(interface_names=["position"], values=[request.height_position.value - self.min_height])
            )

        if cmd.joint_names:
            self.bridge_node.publish_command(cmd)

        return Empty()

    def ResetDefaultValues(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        self.SendCommand(TripodCommand(height_position=FloatValue(value=self.min_height)), context)
        return Empty()

    def GetJointsLimits(self, request: PartId, context: grpc.ServicerContext) -> TripodJointsLimits:
        return TripodJointsLimits(
            height_limit=JointLimits(min=FloatValue(value=self.min_height), max=FloatValue(value=self.max_height))
        )
