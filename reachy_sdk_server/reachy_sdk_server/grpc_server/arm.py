from typing import List, Optional

import grpc
import rclpy
from control_msgs.msg import DynamicJointState, InterfaceValue
from google.protobuf.empty_pb2 import Empty
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
    ListOfArm,
    SpeedLimitRequest,
    TorqueLimitRequest,
)
from reachy2_sdk_api.arm_pb2_grpc import add_ArmServiceServicer_to_server
from reachy2_sdk_api.kinematics_pb2 import Matrix4x4
from reachy2_sdk_api.orbita2d_pb2 import Orbita2dStatus
from reachy2_sdk_api.orbita3d_pb2 import Orbita3dStatus
from reachy2_sdk_api.part_pb2 import PartId

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

    def register_to_server(self, server: grpc.Server):
        self.logger.info("Registering 'ArmServiceServicer' to server.")
        add_ArmServiceServicer_to_server(self, server)

    def get_arm(self, arm: Part, context: grpc.ServicerContext) -> Arm:
        # self.logger.warn(f'ARM id:{arm.id} name:{arm.name}')
        # self.logger.warn(f'ARM components:{arm.components}')
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

    def GetState(self, request: PartId, context: grpc.ServicerContext) -> ArmState:
        arm = self.get_arm_part_by_part_id(request, context)

        return ArmState(
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
        )

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
        self.set_stiffness(request, torque=True, context=context)
        # Set all goal positions to the current position for safety
        part = self.get_arm_part_by_part_id(request, context)
        self.bridge_node.set_all_joints_to_current_position(part.name)

        return Empty()

    def TurnOff(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        self.set_stiffness(request, torque=False, context=context)
        return Empty()

    # Temperatures
    def GetTemperatures(self, request: PartId, context: grpc.ServicerContext) -> ArmTemperatures:
        return ArmTemperatures()

    # Position and Speed limit
    def GetJointsLimits(self, request: PartId, context: grpc.ServicerContext) -> ArmLimits:
        return ArmLimits()

    def SetSpeedLimit(self, request: SpeedLimitRequest, context: grpc.ServicerContext) -> Empty:
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
        arm = self.get_arm_part_by_part_id(request.id, context)
        success, pose = self.bridge_node.compute_forward(request.id, arm_position_to_joint_state(request.position, arm))

        sol = ArmFKSolution()

        if success:
            sol.success = True
            sol.end_effector.pose.data.extend(pose.flatten())

        return sol

    def ComputeArmIK(self, request: ArmIKRequest, context: grpc.ServicerContext) -> ArmIKSolution:
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
        self.bridge_node.publish_target_pose(
            request.id,
            matrix_to_pose(request.goal_pose.data),
        )
        return Empty()
