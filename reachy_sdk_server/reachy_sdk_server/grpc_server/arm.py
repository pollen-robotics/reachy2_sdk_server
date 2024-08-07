import copy
from typing import List, Optional

import grpc
import numpy as np
import rclpy
from control_msgs.msg import DynamicJointState, InterfaceValue
from google.protobuf.empty_pb2 import Empty
from google.protobuf.wrappers_pb2 import BoolValue, Int32Value
from pollen_msgs.msg import IKRequest
from reachy2_sdk_api.arm_pb2 import (
    Arm,
    ArmCartesianGoal,
    ArmDescription,
    ArmFKRequest,
    ArmFKSolution,
    ArmIKRequest,
    ArmIKSolution,
    ArmLimits,
    ArmPosition,
    ArmState,
    ArmStatus,
    ArmTemperatures,
    IKConstrainedMode,
    IKContinuousMode,
    ListOfArm,
    ReachabilityAnswer,
    ReachabilityError,
    SpeedLimitRequest,
    TorqueLimitRequest,
)
from reachy2_sdk_api.arm_pb2_grpc import add_ArmServiceServicer_to_server
from reachy2_sdk_api.kinematics_pb2 import Matrix4x4
from reachy2_sdk_api.orbita2d_pb2 import Orbita2dStatus
from reachy2_sdk_api.orbita3d_pb2 import Orbita3dStatus
from reachy2_sdk_api.part_pb2 import PartId


from . import tracing_helper
from opentelemetry import trace

from ..abstract_bridge_node import AbstractBridgeNode
from ..conversion import arm_position_to_joint_state, joint_state_to_arm_position, matrix_to_pose
from ..parts import Part
from ..utils import get_current_timestamp
from .orbita2d import ComponentId, Orbita2dCommand, Orbita2dsCommand, Orbita2dServicer, Orbita2dStateRequest
from .orbita3d import Orbita3dCommand, Orbita3dsCommand, Orbita3dServicer, Orbita3dStateRequest


class ArmServicer:
    def __init__(
        self,
        bridge_node: AbstractBridgeNode,
        logger: rclpy.impl.rcutils_logger.RcutilsLogger,
        orbita2d_servicer: Orbita2dServicer,
        orbita3d_servicer: Orbita3dServicer,
    ) -> None:
        self.bridge_node = bridge_node
        self.logger = logger

        self.orbita2d_servicer = orbita2d_servicer
        self.orbita3d_servicer = orbita3d_servicer

        self.arms = self.bridge_node.parts.get_by_type("arm")

        self.default_preferred_theta = -4 * np.pi / 6
        self.default_d_theta_max = 0.01

    def register_to_server(self, server: grpc.Server):
        self.logger.info("Registering 'ArmServiceServicer' to server.")
        add_ArmServiceServicer_to_server(self, server)

    def get_arm(self, arm: Part, context: grpc.ServicerContext) -> Arm:
        return Arm(
            part_id=PartId(name=arm.name, id=arm.id),
            description=ArmDescription(
                shoulder=Orbita2dServicer.get_info(self.bridge_node.components.get_by_name(arm.components[0].name)),
                elbow=Orbita2dServicer.get_info(self.bridge_node.components.get_by_name(arm.components[1].name)),
                wrist=Orbita3dServicer.get_info(self.bridge_node.components.get_by_name(arm.components[2].name)),
            ),
        )

    def GetAllArms(self, request: Empty, context: grpc.ServicerContext) -> ListOfArm:
        return ListOfArm(arm=[self.get_arm(arm, context) for arm in self.arms])

    def get_arm_part_by_part_id(self, part_id: PartId, context: grpc.ServicerContext) -> Part:
        part = self.bridge_node.parts.get_by_part_id(part_id)

        if part is None:
            context.abort(grpc.StatusCode.NOT_FOUND, f"Part not found (id={part_id}).")

        if part.type != "arm":
            context.abort(
                grpc.StatusCode.INVALID_ARGUMENT,
                f"Part '{part_id}' is not an arm.",
            )
        return part

    def get_reachability_answer(self, order_id, is_reachable, state):
        state_to_description = {
            "wrist out of range": ReachabilityError.DISTANCE_LIMIT,
            "Pose out of reach": ReachabilityError.DISTANCE_LIMIT,
            "Backward pose": ReachabilityError.DISTANCE_LIMIT,
            "limited by shoulder": ReachabilityError.SHOULDER_LIMIT,
            "limited by elbow": ReachabilityError.ELBOW_LIMIT,
            "limited by wrist": ReachabilityError.WRIST_LIMIT,
            "continuity limit": ReachabilityError.CONTINUITY_LIMIT,
        }

        if not is_reachable:
            description = state_to_description.get(state, ReachabilityError.OTHER)
        else:
            description = ReachabilityError.NO_ERROR

        return ReachabilityAnswer(
            order_id=Int32Value(value=order_id),
            is_reachable=BoolValue(value=is_reachable),
            description=description,
        )

    def get_reachability_state(self, arm: str):
        reachability_answers = []
        while len(self.bridge_node.reachability_deque[arm]) > 0:
            (order_id, is_reachable, state) = self.bridge_node.reachability_deque[arm].popleft()
            reachability_answer = self.get_reachability_answer(order_id, is_reachable, state)
            reachability_answers.append(reachability_answer)
        return reachability_answers

    def GetState(self, request: PartId, context: grpc.ServicerContext) -> ArmState:
        arm = self.get_arm_part_by_part_id(request, context)
        arm_state = ArmState(
            timestamp=get_current_timestamp(self.bridge_node),
            id=request,
            activated=True,
            shoulder_state=self.orbita2d_servicer.GetState(
                Orbita2dStateRequest(
                    fields=self.orbita2d_servicer.default_fields,
                    id=ComponentId(id=arm.components[0].id),
                ),
                context,
            ),
            elbow_state=self.orbita2d_servicer.GetState(
                Orbita2dStateRequest(
                    fields=self.orbita2d_servicer.default_fields,
                    id=ComponentId(id=arm.components[1].id),
                ),
                context,
            ),
            wrist_state=self.orbita3d_servicer.GetState(
                Orbita3dStateRequest(
                    fields=self.orbita3d_servicer.default_fields,
                    id=ComponentId(id=arm.components[2].id),
                ),
                context,
            ),
            reachability=self.get_reachability_state(arm.name),
        )
        return arm_state

    def GetCartesianPosition(self, request: PartId, context: grpc.ServicerContext) -> Matrix4x4:
        request = ArmFKRequest(
            id=request,
            position=self.GetJointPosition(request, context),
        )

        sol = self.ComputeArmFK(request, context)
        assert sol.success

        return sol.end_effector.pose

    def GetJointPosition(self, request: PartId, context: grpc.ServicerContext) -> ArmPosition:
        state = self.GetState(request, context)
        return ArmPosition(
            shoulder_position=state.shoulder_state.present_position,
            elbow_position=state.elbow_state.present_position,
            wrist_position=state.wrist_state.present_position,
        )

    def GetJointGoalPosition(self, request: PartId, context: grpc.ServicerContext) -> ArmPosition:
        state = self.GetState(request, context)
        return ArmPosition(
            shoulder_position=state.shoulder_state.goal_position,
            elbow_position=state.elbow_state.goal_position,
            wrist_position=state.wrist_state.goal_position,
        )

    # Compliances
    def set_stiffness(self, request: PartId, torque: bool, context: grpc.ServicerContext) -> None:
        # TODO: re-write using self.orbita2d_servicer.SendCommand?
        part = self.get_arm_part_by_part_id(request, context)

        cmd = DynamicJointState()
        cmd.joint_names = []

        for c in part.components:
            cmd.joint_names.append(c.name)
            cmd.interface_values.append(
                InterfaceValue(
                    interface_names=["torque"],
                    values=[torque],
                )
            )

        self.bridge_node.publish_command(cmd)

    def TurnOn(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        with tracing_helper.PollenSpan(tracer=self.bridge_node.tracer, trace_name=f"TurnOn"):
            self.set_stiffness(request, torque=True, context=context)
            # Set all goal positions to the current position for safety
            part = self.get_arm_part_by_part_id(request, context)
            self.bridge_node.set_all_joints_to_current_position(part.name)

        return Empty()

    def TurnOff(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        with tracing_helper.PollenSpan(tracer=self.bridge_node.tracer, trace_name=f"TurnOff"):
            self.set_stiffness(request, torque=False, context=context)
        return Empty()

    # Temperatures
    def GetTemperatures(self, request: PartId, context: grpc.ServicerContext) -> ArmTemperatures:
        return ArmTemperatures()

    # Position and Speed limit
    def GetJointsLimits(self, request: PartId, context: grpc.ServicerContext) -> ArmLimits:
        return ArmLimits()

    def SetSpeedLimit(self, request: SpeedLimitRequest, context: grpc.ServicerContext) -> Empty:
        with tracing_helper.PollenSpan(tracer=self.bridge_node.tracer, trace_name=f"SetSpeedLimit"):
            # TODO: re-write using self.orbita2d_servicer.SendCommand?
            part = self.get_arm_part_by_part_id(request.id, context)
            cmd = DynamicJointState()
            cmd.joint_names = []
            for c in part.components:
                nb_rw_motor = 3 if c.type == "orbita3d" else 2
                for i in range(1, nb_rw_motor + 1):
                    # cmd.joint_names.append(c.name)
                    cmd.joint_names.append(f"{c.name}_raw_motor_{i}")

                    cmd.interface_values.append(
                        InterfaceValue(
                            interface_names=["speed_limit"],
                            values=[request.limit / 100],
                        )
                    )

            self.bridge_node.publish_command(cmd)
        return Empty()

    def SetTorqueLimit(self, request: TorqueLimitRequest, context: grpc.ServicerContext) -> Empty:
        with tracing_helper.PollenSpan(tracer=self.bridge_node.tracer, trace_name=f"SetTorqueLimit"):
            # TODO: re-write using self.orbita2d_servicer.SendCommand?
            part = self.get_arm_part_by_part_id(request.id, context)

            cmd = DynamicJointState()
            cmd.joint_names = []

            for c in part.components:
                nb_rw_motor = 3 if c.type == "orbita3d" else 2
                for i in range(1, nb_rw_motor + 1):
                    # cmd.joint_names.append(c.name)
                    cmd.joint_names.append(f"{c.name}_raw_motor_{i}")

                    cmd.interface_values.append(
                        InterfaceValue(
                            interface_names=["torque_limit"],
                            values=[request.limit / 100],
                        )
                    )

            self.bridge_node.publish_command(cmd)
        return Empty()

    # Kinematics
    def ComputeArmFK(self, request: ArmFKRequest, context: grpc.ServicerContext) -> ArmFKSolution:
        with tracing_helper.PollenSpan(tracer=self.bridge_node.tracer, trace_name=f"ComputeArmFK"):
            arm = self.get_arm_part_by_part_id(request.id, context)
            success, pose = self.bridge_node.compute_forward(request.id, arm_position_to_joint_state(request.position, arm))

            sol = ArmFKSolution()

            if success:
                sol.success = True
                sol.end_effector.pose.data.extend(pose.flatten())

        return sol

    def ComputeArmIK(self, request: ArmIKRequest, context: grpc.ServicerContext) -> ArmIKSolution:
        with tracing_helper.PollenSpan(tracer=self.bridge_node.tracer, trace_name=f"ComputeArmIK"):
            arm = self.get_arm_part_by_part_id(request.id, context)

            success, joint_position = self.bridge_node.compute_inverse(
                request.id,
                request.target.pose.data,
                arm_position_to_joint_state(request.q0, arm),
            )

            sol = ArmIKSolution()

            if success:
                sol.success = True
                sol.arm_position.CopyFrom(joint_state_to_arm_position(joint_position, arm))

        return sol

    # Doctor
    def Audit(self, request: PartId, context: grpc.ServicerContext) -> ArmStatus:
        arm = self.get_arm_part_by_part_id(request, context)

        return ArmStatus(
            shoulder_status=self.orbita2d_servicer.Audit(
                ComponentId(id=arm.components[0].id),
                context,
            ),
            elbow_status=self.orbita2d_servicer.Audit(
                ComponentId(id=arm.components[1].id),
                context,
            ),
            wrist_status=self.orbita3d_servicer.Audit(
                ComponentId(id=arm.components[2].id),
                context,
            ),
        )

    def HeartBeat(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        return Empty()

    def Restart(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        return Empty()

    def ResetDefaultValues(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        return Empty()

    def SendArmCartesianGoal(self, request: ArmCartesianGoal, context: grpc.ServicerContext) -> Empty:
        with tracing_helper.PollenSpan(tracer=self.bridge_node.tracer, trace_name=f"SendArmCartesianGoal"):
            constrained_mode_dict = {
                IKConstrainedMode.UNCONSTRAINED: "unconstrained",
                IKConstrainedMode.LOW_ELBOW: "low_elbow",
                IKConstrainedMode.UNDEFINED_CONSTRAINED_MODE: "undefined",
            }

            continuous_mode_dict = {
                IKContinuousMode.CONTINUOUS: "continuous",
                IKContinuousMode.DISCRETE: "discrete",
                IKContinuousMode.UNDEFINED_CONTINUOUS_MODE: "undefined",
            }

            constrained_mode = constrained_mode_dict.get(request.constrained_mode, "undefined")
            continuous_mode = continuous_mode_dict.get(request.continuous_mode, "undefined")

            # PREFERRED_THETA et D_THETA_MAX
            if request.HasField("preferred_theta"):  # -> on utilise request.preferred_theta.value, sinon valeur par défaut
                preferred_theta = request.preferred_theta.value
            else:
                preferred_theta = self.default_preferred_theta

            if request.HasField("d_theta_max"):  # -> on utilise request.d_theta_max.value, sinon valeur par défaut
                d_theta_max = request.d_theta_max.value
            else:
                d_theta_max = self.default_d_theta_max

            # REACHABILITY:
            if request.HasField("order_id"):  # -> on fait un truc avec request.order_id.value, sinon osef
                order_id = request.order_id.value
            else:
                order_id = 0

            msg = IKRequest()
            msg.traceparent = tracing_helper.traceparent()

            msg.pose.pose = matrix_to_pose(request.goal_pose.data)
            msg.constrained_mode = constrained_mode
            msg.continuous_mode = continuous_mode
            msg.preferred_theta = preferred_theta
            msg.d_theta_max = d_theta_max
            msg.order_id = order_id

            with tracing_helper.PollenSpan(tracer=self.bridge_node.tracer,
                                           trace_name=f"bridge_node.publish_arm_target_pose",
                                           kind=trace.SpanKind.CLIENT):
                # timestamp here for tracing purposes
                msg.pose.header.stamp = self.bridge_node.get_clock().now().to_msg()
                self.bridge_node.publish_arm_target_pose(
                    request.id,
                    msg,
                )

        # self.bridge_node.logger.info(f"Received goal pose for arm : request {request}  \nmsg : {msg}'.")
        return Empty()
