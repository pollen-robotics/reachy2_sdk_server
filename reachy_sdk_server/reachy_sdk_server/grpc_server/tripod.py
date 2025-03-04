import math
from typing import Iterator

import grpc
import numpy as np
import rclpy
import reachy2_monitoring as rm
from control_msgs.msg import DynamicJointState, InterfaceValue
from google.protobuf.empty_pb2 import Empty
from google.protobuf.wrappers_pb2 import BoolValue, FloatValue
from reachy2_sdk_api.component_pb2 import ComponentId, JointLimits, PIDGains
from reachy2_sdk_api.tripod_pb2 import (
    Tripod,
    TripodAxis,
    TripodCommand,
    TripodDescription,
    TripodJoint,
    TripodJointsLimits,
    TripodState,
)
from reachy2_sdk_api.tripod_pb2_grpc import add_TripodServiceServicer_to_server
from sensor_msgs.msg import JointState

from ..abstract_bridge_node import AbstractBridgeNode
from ..parts import PartId
from ..utils import endless_get_stream, extract_fields, get_current_timestamp


class TripodServicer:

    def __init__(
        self,
        bridge_node: AbstractBridgeNode,
        logger: rclpy.impl.rcutils_logger.RcutilsLogger,
    ) -> None:
        self.bridge_node = bridge_node
        self.logger = logger

    def register_to_server(self, server: grpc.Server):
        self.logger.info("Registering 'TripodServiceServicer' to server.")
        add_TripodServiceServicer_to_server(self, server)

    def GetTripod(self, request: Empty, context: grpc.ServicerContext) -> Tripod:
        return Tripod(
            id=PartId(name="tripod", id=100),
            description=TripodDescription(height_joint=TripodJoint(axis=TripodAxis.HEIGHT, name="tripod_joint"))
        )

    # State
    def GetState(self, request: PartId, context: grpc.ServicerContext) -> TripodState:
        components = self.bridge_node.components
        dxl_motor = components.get_by_component_id(request.id)
        state = extract_fields(DynamixelMotorField, request.fields, conversion_table, dxl_motor)

        state["timestamp"] = get_current_timestamp(self.bridge_node)
        state["temperature"] = FloatValue(value=40.0)
        state["joint_limits"] = JointLimits(min=FloatValue(value=0.0), max=FloatValue(value=100.0))
        return DynamixelMotorState(**state)

    # Command
    def SendCommand(self, request: TripodCommand, context: grpc.ServicerContext) -> Empty:
        cmd = DynamicJointState()
        cmd.joint_names = []

        if not request.HasField("part_id"):
            context.abort(grpc.StatusCode.INVALID_ARGUMENT, "Missing 'id' field.")

        components = self.bridge_node.components

        dxl_motor = components.get_by_component_id(req_cmd.id)
        if dxl_motor is None:
            dxl_motor = components.get_by_component_name(req_cmd.name)

        if dxl_motor is None:
            context.abort(grpc.StatusCode.NOT_FOUND, "Component not found.")
        else:
            if req_cmd.HasField("compliant"):
                cmd.joint_names.append(dxl_motor.name)
                cmd.interface_values.append(
                    InterfaceValue(
                        interface_names=["torque"],
                        values=[not req_cmd.compliant.value],
                    )
                )

            if req_cmd.HasField("goal_position"):
                cmd.joint_names.append(dxl_motor.name)
                cmd.interface_values.append(
                    InterfaceValue(
                        interface_names=["position"],
                        values=[req_cmd.goal_position.value],
                    )
                )

            if req_cmd.HasField("speed_limit"):
                cmd.joint_names.append(dxl_motor.name)
                cmd.interface_values.append(
                    InterfaceValue(
                        interface_names=["speed_limit"],
                        values=[req_cmd.speed_limit.value],
                    )
                )

            if req_cmd.HasField("torque_limit"):
                cmd.joint_names.append(dxl_motor.name)
                cmd.interface_values.append(
                    InterfaceValue(
                        interface_names=["torque_limit"],
                        values=[req_cmd.torque_limit.value],
                    )
                )

        if cmd.joint_names:
            self.bridge_node.publish_command(cmd)

        return Empty()

    def ResetDefaultValues(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        return Empty()

    def GetJointsLimits(self, request: PartId, context: grpc.ServicerContext) -> TripodJointsLimits:
        return TripodJointsLimits(height_limit=JointLimits(min=FloatValue(value=0.996), max=FloatValue(value=1.2)))


conversion_table = {
    "id": lambda o: ComponentId(id=o.id, name=o.name),
    "present_position": lambda o: FloatValue(value=o.state["position"]),
    "present_speed": lambda o: FloatValue(value=o.state["velocity"]),
    "present_load": lambda o: FloatValue(value=o.state["effort"]),
    "compliant": lambda o: BoolValue(value=not o.state["torque"]),
    "goal_position": lambda o: FloatValue(value=o.state["target_position"]),
    "speed_limit": lambda o: FloatValue(value=o.state["speed_limit"]),
    "torque_limit": lambda o: FloatValue(value=o.state["torque_limit"]),
    "pid": lambda o: PIDGains(
        p=(
            FloatValue(value=o.raw_motor_1.state["p_gain"])
            if not math.isnan(o.raw_motor_1.state["p_gain"])
            else FloatValue(value=100.0)
        ),
        i=(
            FloatValue(value=o.raw_motor_1.state["i_gain"])
            if not math.isnan(o.raw_motor_1.state["i_gain"])
            else FloatValue(value=100.0)
        ),
        d=(
            FloatValue(value=o.raw_motor_1.state["d_gain"])
            if not math.isnan(o.raw_motor_1.state["d_gain"])
            else FloatValue(value=100.0)
        ),
    ),
}
