from collections import namedtuple

import grpc
import numpy as np
import rclpy
from control_msgs.msg import DynamicJointState, InterfaceValue
from google.protobuf.empty_pb2 import Empty
from google.protobuf.wrappers_pb2 import BoolValue, FloatValue
from reachy2_sdk_api.component_pb2 import ComponentId
from reachy2_sdk_api.hand_pb2 import (
    Force,
    Hand,
    HandPosition,
    HandPositionRequest,
    HandState,
    HandStatus,
    HandTemperatures,
    JointsLimits,
    ListOfHand,
    ParallelGripperPosition,
    SpeedLimitRequest,
)
from reachy2_sdk_api.hand_pb2_grpc import add_HandServiceServicer_to_server
from reachy2_sdk_api.part_pb2 import PartId

from ..abstract_bridge_node import AbstractBridgeNode
from ..parts import Part
import reachy2_monitoring as rm

HandComponents = namedtuple("HandComponents", ["actuator", "finger", "raw_motor_1"])


class HandServicer:
    def __init__(
        self,
        bridge_node: AbstractBridgeNode,
        logger: rclpy.impl.rcutils_logger.RcutilsLogger,
    ) -> None:
        self.bridge_node = bridge_node
        self.logger = logger

        self.hands = self.bridge_node.parts.get_by_type("hand")

    def register_to_server(self, server: grpc.Server):
        self.logger.info("Registering 'HandServiceServicer' to server.")
        add_HandServiceServicer_to_server(self, server)

    def get_hand(self, hand: Part, context: grpc.ServicerContext) -> Hand:
        return Hand(
            part_id=PartId(name=hand.name, id=hand.id),
        )

    def get_hand_part_from_part_id(self, part_id: PartId, context: grpc.ServicerContext) -> Part:
        part = self.bridge_node.parts.get_by_part_id(part_id)

        if part is None:
            context.abort(grpc.StatusCode.NOT_FOUND, f"Part not found (id={part_id}).")

        if part.type != "hand":
            context.abort(
                grpc.StatusCode.INVALID_ARGUMENT,
                f"Part '{part_id}' is not an hand.",
            )

        return part

    def GetAllHands(self, request: Empty, context: grpc.ServicerContext) -> ListOfHand:
        return ListOfHand(hand=[self.get_hand(hand, context) for hand in self.hands])

    def GetState(self, request: PartId, context: grpc.ServicerContext) -> HandState:
        hand = self.get_hand_part_from_part_id(request, context)
        hand_components = self.get_hand_components(hand.components[0], context)

        position = hand_components.finger.state["position"]
        opening = self.position_to_opening(position)
        torque = hand_components.actuator.state["torque"]

        return HandState(
            opening=FloatValue(value=opening),
            present_position=HandPosition(
                parallel_gripper=ParallelGripperPosition(position=position),
            ),
            goal_position=self.GetHandGoalPosition(request, context),
            compliant=BoolValue(value=not torque),
        )

    def OpenHand(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        return self.SetHandPosition(
            request=HandPositionRequest(
                id=request,
                position=HandPosition(
                    parallel_gripper=ParallelGripperPosition(position=1.0),
                ),
            ),
            context=context,
        )

    def CloseHand(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        return self.SetHandPosition(
            request=HandPositionRequest(
                id=request,
                position=HandPosition(
                    parallel_gripper=ParallelGripperPosition(position=0.0),
                ),
            ),
            context=context,
        )

    def set_stiffness(self, request: PartId, torque: bool, context: grpc.ServicerContext) -> None:

        with rm.PollenSpan(tracer=self.bridge_node.tracer, trace_name=f"SetHandPosition"):
            hand = self.get_hand_part_from_part_id(request, context)

            cmd = DynamicJointState()
            cmd.joint_names = []

            for c in hand.components:
                cmd.joint_names.append(c.name)
                cmd.interface_values.append(
                    InterfaceValue(
                        interface_names=["torque"],
                        values=[torque],
                    )
                )

        self.bridge_node.publish_command(cmd)

    def TurnOn(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        self.set_stiffness(request, torque=True, context=context)
        return Empty()

    def TurnOff(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        self.set_stiffness(request, torque=False, context=context)
        return Empty()

    def GetHandGoalPosition(self, request: PartId, context: grpc.ServicerContext) -> HandPosition:
        hand = self.get_hand_part_from_part_id(request, context)
        hand_components = self.get_hand_components(hand.components[0], context)
        position = hand_components.finger.state["target_position"]

        return HandPosition(
            parallel_gripper=ParallelGripperPosition(position=position),
        )

    def SetHandPosition(self, request: HandPositionRequest, context: grpc.ServicerContext) -> Empty:
        with rm.PollenSpan(tracer=self.bridge_node.tracer, trace_name=f"SetHandPosition"):
            hand = self.get_hand_part_from_part_id(request.id, context)

            # This is a % of the opening
            opening = np.clip(request.position.parallel_gripper.position, 0, 1)

            cmd = DynamicJointState()
            cmd.joint_names = []

            for c in hand.components:
                cmd.joint_names.append(c.name + "_finger")
                cmd.interface_values.append(
                    InterfaceValue(
                        interface_names=["position"],
                        values=[opening],
                    )
                )
            self.bridge_node.publish_command(cmd)

        return Empty()

    def Audit(self, request: PartId, context: grpc.ServicerContext) -> HandStatus:
        return HandStatus()

    def HeartBeat(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        return Empty()

    def Restart(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        return Empty()

    def ResetDefaultValues(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        return Empty()

    def GetJointLimit(self, request: PartId, context: grpc.ServicerContext) -> JointsLimits:
        return JointsLimits()

    def GetTemperature(self, request: PartId, context: grpc.ServicerContext) -> HandTemperatures:
        return HandTemperatures()

    def SetSpeedLimit(self, request: SpeedLimitRequest, context: grpc.ServicerContext) -> Empty:
        return Empty()

    def GetForce(self, request: PartId, context: grpc.ServicerContext) -> Force:
        return Force()

    OPEN_POSITION = np.deg2rad(130)
    CLOSE_POSITION = 0.0

    def position_to_opening(self, position: float) -> float:
        opening = (position - self.CLOSE_POSITION) / (self.OPEN_POSITION - self.CLOSE_POSITION)
        opening = np.clip(opening, 0, 1)
        return opening

    # Setup utils
    def get_hand_components(self, component_id: ComponentId, context: grpc.ServicerContext) -> HandComponents:
        if not hasattr(self, "_lazy_components"):
            self._lazy_components = {}

        components = self.bridge_node.components

        c = components.get_by_component_id(component_id)
        if c is None:
            context.abort(
                grpc.StatusCode.NOT_FOUND,
                f"Could not find component with id '{component_id}'.",
            )

        if c.type != "dynamixel":
            context.abort(
                grpc.StatusCode.INVALID_ARGUMENT,
                f"Component '{component_id}' is not an dynamixel.",
            )

        if c.id not in self._lazy_components:
            hand = components.get_by_component_id(component_id)
            hand_finger = components.get_by_name(f"{hand.name}_finger")
            hand_raw_motor_1 = components.get_by_name(f"{hand.name}_raw_motor_1")

            self._lazy_components[c.id] = HandComponents(
                hand,
                hand_finger,
                hand_raw_motor_1,
            )

        return self._lazy_components[c.id]
