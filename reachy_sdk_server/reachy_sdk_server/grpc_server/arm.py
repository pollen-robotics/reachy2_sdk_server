import grpc
import rclpy

from control_msgs.msg import DynamicJointState, InterfaceValue

from google.protobuf.empty_pb2 import Empty

from reachy_sdk_api_v2.arm_pb2 import (
    Arm,
    ArmCartesianGoal,
    ArmDescription,
    ArmFKRequest,
    ArmFKSolution,
    ArmIKRequest,
    ArmIKSolution,
    ArmJointGoal,
    ArmPosition,
    ArmStatus,
    ArmTemperatures,
    ArmLimits,
    ListOfArm,
    SpeedLimitRequest,
)
from reachy_sdk_api_v2.arm_pb2_grpc import (
    add_ArmServiceServicer_to_server,
)
from reachy_sdk_api_v2.part_pb2 import PartId
from reachy_sdk_api_v2.kinematics_pb2 import Matrix4x4


from ..abstract_bridge_node import AbstractBridgeNode
from ..conversion import (
    arm_position_to_joint_state,
    joint_state_to_arm_position,
    pose_from_pos_and_ori,
)
from .orbita2d import (
    ComponentId,
    Orbita2DCommand,
    Orbita2DField,
    Orbita2dServicer,
    Orbita2DStateRequest,
)
from .orbita3d import (
    Orbita3DCommand,
    Orbita3DField,
    Orbita3DStateRequest,
    Orbita3dServicer,
)
from ..parts import Part


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
        return Arm(
            part_id=PartId(name=arm.name, id=arm.id),
            description=ArmDescription(
                shoulder=Orbita2dServicer.get_info(
                    self.bridge_node.components.get_by_name(arm.components[0].name)
                ),
                elbow=Orbita2dServicer.get_info(
                    self.bridge_node.components.get_by_name(arm.components[1].name)
                ),
                wrist=Orbita3dServicer.get_info(
                    self.bridge_node.components.get_by_name(arm.components[2].name)
                ),
            ),
        )

    def GetAllArms(self, request: Empty, context: grpc.ServicerContext) -> ListOfArm:
        return ListOfArm(arm=[self.get_arm(arm, context) for arm in self.arms])

    # Position and GoTo
    def GoToCartesianPosition(
        self, request: ArmCartesianGoal, context: grpc.ServicerContext
    ) -> Empty:
        # TODO:
        # We do not take the duration or tolerance into account
        # We will develop a more advanced controller to handles this

        self.bridge_node.publish_target_pose(
            request.id,
            pose_from_pos_and_ori(request.target_position, request.target_orientation),
        )

        return Empty()

    def GoToJointPosition(
        self, request: ArmJointGoal, context: grpc.ServicerContext
    ) -> Empty:
        arm = self.bridge_node.parts.get_by_part_id(request.id)

        # TODO:
        # We do not take the duration into account
        # We will develop a more advanced controller to handles this

        # TODO: Use Orbita2DsCommand
        self.orbita2d_servicer.SendCommand(
            Orbita2DCommand(
                id=ComponentId(id=arm.components[0].id),
                goal_position=request.position.shoulder_position,
            ),
            context,
        )
        self.orbita2d_servicer.SendCommand(
            Orbita2DCommand(
                id=ComponentId(id=arm.components[1].id),
                goal_position=request.position.elbow_position,
            ),
            context,
        )
        self.orbita3d_servicer.SendCommand(
            Orbita3DCommand(
                id=ComponentId(id=arm.components[2].id),
                goal_position=request.position.wrist_position,
            ),
            context,
        )

        return Empty()

    def GetCartesianPosition(
        self, request: PartId, context: grpc.ServicerContext
    ) -> Matrix4x4:
        request = ArmFKRequest(
            id=request,
            position=self.GetJointPosition(request, context),
        )

        sol = self.ComputeArmFK(request, context)
        assert sol.success

        return sol.end_effector.pose

    def GetJointPosition(
        self, request: PartId, context: grpc.ServicerContext
    ) -> ArmPosition:
        arm = self.bridge_node.parts.get_by_part_id(request)

        return ArmPosition(
            shoulder_position=self.orbita2d_servicer.GetState(
                Orbita2DStateRequest(
                    fields=[Orbita2DField.PRESENT_POSITION],
                    id=ComponentId(id=arm.components[0].id),
                ),
                context,
            ).present_position,
            elbow_position=self.orbita2d_servicer.GetState(
                Orbita2DStateRequest(
                    fields=[Orbita2DField.PRESENT_POSITION],
                    id=ComponentId(id=arm.components[1].id),
                ),
                context,
            ).present_position,
            wrist_position=self.orbita3d_servicer.GetState(
                Orbita3DStateRequest(
                    fields=[Orbita3DField.PRESENT_POSITION],
                    id=ComponentId(id=arm.components[2].id),
                ),
                context,
            ).present_position,
        )

    def GetJointGoalPosition(
        self, request: PartId, context: grpc.ServicerContext
    ) -> ArmPosition:
        arm = self.bridge_node.parts.get_by_part_id(request)

        return ArmPosition(
            shoulder_position=self.orbita2d_servicer.GetState(
                Orbita2DStateRequest(
                    fields=[Orbita2DField.GOAL_POSITION],
                    id=ComponentId(id=arm.components[0].id),
                ),
                context,
            ).goal_position,
            elbow_position=self.orbita2d_servicer.GetState(
                Orbita2DStateRequest(
                    fields=[Orbita2DField.GOAL_POSITION],
                    id=ComponentId(id=arm.components[1].id),
                ),
                context,
            ).goal_position,
            wrist_position=self.orbita3d_servicer.GetState(
                Orbita3DStateRequest(
                    fields=[Orbita3DField.GOAL_POSITION],
                    id=ComponentId(id=arm.components[2].id),
                ),
                context,
            ).goal_position,
        )

    # Compliances
    def set_stiffness(self, request: PartId, torque: bool) -> None:
        part = self.bridge_node.parts.get_by_part_id(request)

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

        self.logger.info(f"Publishing command: {cmd}")
        self.bridge_node.publish_command(cmd)

    def TurnOn(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        self.set_stiffness(request, torque=True)
        return Empty()

    def TurnOff(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        self.set_stiffness(request, torque=False)
        return Empty()

    # Temperatures
    def GetTemperatures(
        self, request: PartId, context: grpc.ServicerContext
    ) -> ArmTemperatures:
        return ArmTemperatures()

    # Position and Speed limit
    def GetJointsLimits(
        self, request: PartId, context: grpc.ServicerContext
    ) -> ArmLimits:
        return ArmLimits()

    def SetSpeedLimit(
        self, request: SpeedLimitRequest, context: grpc.ServicerContext
    ) -> Empty:
        return Empty()

    # Kinematics
    def ComputeArmFK(
        self, request: ArmFKRequest, context: grpc.ServicerContext
    ) -> ArmFKSolution:
        arm = self.bridge_node.parts.get_by_part_id(request.id)
        success, pose = self.bridge_node.compute_forward(
            request.id, arm_position_to_joint_state(request.position, arm)
        )

        sol = ArmFKSolution()

        if success:
            sol.success = True
            sol.end_effector.pose.data.extend(pose.flatten())

        return sol

    def ComputeArmIK(
        self, request: ArmIKRequest, context: grpc.ServicerContext
    ) -> ArmIKSolution:
        arm = self.bridge_node.parts.get_by_part_id(request.id)

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
        return ArmStatus()

    def HeartBeat(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        return Empty()

    def Restart(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        return Empty()

    def ResetDefaultValues(
        self, request: PartId, context: grpc.ServicerContext
    ) -> Empty:
        return Empty()
