import grpc
import rclpy
import asyncio
import threading


from control_msgs.msg import DynamicJointState, InterfaceValue

from google.protobuf.empty_pb2 import Empty
from typing import List, Optional
from action_msgs.msg import GoalStatus


from reachy2_sdk_api.arm_pb2 import (
    Arm,
    ArmCartesianGoal,
    ArmDescription,
    ArmFKRequest,
    ArmFKSolution,
    ArmIKRequest,
    ArmIKSolution,
    ArmJointGoal,
    ArmPosition,
    ArmState,
    ArmStatus,
    ArmTemperatures,
    ArmLimits,
    ListOfArm,
    SpeedLimitRequest,
)
from reachy2_sdk_api.arm_pb2_grpc import (
    add_ArmServiceServicer_to_server,
)
from reachy2_sdk_api.part_pb2 import PartId
from reachy2_sdk_api.kinematics_pb2 import Matrix4x4


from ..abstract_bridge_node import AbstractBridgeNode
from ..conversion import (
    arm_position_to_joint_state,
    joint_state_to_arm_position,
    pose_from_pos_and_ori,
)
from .orbita2d import (
    ComponentId,
    Orbita2dCommand,
    Orbita2dsCommand,
    Orbita2dServicer,
    Orbita2dStateRequest,
)
from .orbita3d import (
    Orbita3dCommand,
    Orbita3dsCommand,
    Orbita3dStateRequest,
    Orbita3dServicer,
)
from ..parts import Part
from ..utils import get_current_timestamp


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

        # goto goals management
        self.goal_manager = GoalManager()

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

    def part_to_list_of_joint_names(self, part: Part) -> List[str]:
        """Return a list of joint names from a part.
        The output names match those of the /joint_state message, e.g.: r_shoulder_pitch
        """
        joint_names = []
        for component in part.components:
            base_name = component.extra["name"]
            for axis in ["axis1", "axis2", "axis3"]:
                if axis in component.extra:
                    joint_name = f"{base_name}_{component.extra[axis]}"
                    joint_names.append(joint_name)
        return joint_names

    def GetAllArms(self, request: Empty, context: grpc.ServicerContext) -> ListOfArm:
        return ListOfArm(arm=[self.get_arm(arm, context) for arm in self.arms])

    def get_arm_part_by_part_id(
        self, part_id: PartId, context: grpc.ServicerContext
    ) -> Part:
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

    # Position and GoTo
    def GoToCartesianPosition(
        self, request: ArmCartesianGoal, context: grpc.ServicerContext
    ) -> Empty:
        arm = self.get_arm_part_by_part_id(request.id, context)
        duration = request.duration.value

        success, joint_position = self.bridge_node.compute_inverse(
            request.id,
            request.target.pose.data,
            arm_position_to_joint_state(request.q0, arm),
        )  # 'joint_position': 'sensor_msgs/JointState'
        # get the joint names and joint positions from the joint_state
        joint_names = []
        goal_positions = []
        for i in range(len(joint_position.name)):
            joint_names.append(joint_position.name[i])
            goal_positions.append(joint_position.position[i])

        self.logger.info(f"goal_positions: {goal_positions}")

        future = asyncio.run_coroutine_threadsafe(
            self.bridge_node.send_goto_goal(
                arm.name,
                joint_names,
                goal_positions,
                duration,
                mode="minimum_jerk",
                feedback_callback=None,
                return_handle=True,
            ),
            self.bridge_node.asyncio_loop,
        )
        if future is None:
            self.logger.info("GotoGoal was rejected")
            ## TODO return -1
            return Empty()

        # Wait for the result and get it => This has to be fast
        goal_handle = future.result()

        goal_id = self.goal_manager.store_goal_handle(goal_handle)
        self.logger.info(f"goal_id: {goal_id}")
        # TODO return unique id that represents this goal handle
        return Empty()

    def GoToJointPosition(
        self, request: ArmJointGoal, context: grpc.ServicerContext
    ) -> Empty:
        arm = self.get_arm_part_by_part_id(request.id, context)
        self.logger.info(f"arm: {arm.name}")

        joint_names = self.part_to_list_of_joint_names(arm)
        self.logger.info(f"joint_names: {joint_names}")

        duration = request.duration.value
        self.logger.info(f"duration: {duration}")

        goal_positions = [
            request.position.shoulder_position.axis_1.value,
            request.position.shoulder_position.axis_2.value,
            request.position.elbow_position.axis_1.value,
            request.position.elbow_position.axis_2.value,
            request.position.wrist_position.rpy.roll,
            request.position.wrist_position.rpy.pitch,
            request.position.wrist_position.rpy.yaw,
        ]
        self.logger.info(f"goal_positions: {goal_positions}")

        future = asyncio.run_coroutine_threadsafe(
            self.bridge_node.send_goto_goal(
                arm.name,
                joint_names,
                goal_positions,
                duration,
                mode="minimum_jerk",
                feedback_callback=None,
                return_handle=True,
            ),
            self.bridge_node.asyncio_loop,
        )
        if future is None:
            self.logger.info("GotoGoal was rejected")
            ## TODO return -1
            return Empty()

        # Wait for the result and get it => This has to be fast
        goal_handle = future.result()

        goal_id = self.goal_manager.store_goal_handle(goal_handle)
        self.logger.info(f"goal_id: {goal_id}")
        # TODO return unique id that represents this goal handle
        return Empty()

    def get_status_string_by_goal_id(self, goal_id: int) -> str:
        """
        Get the status string based on the goto goal ID.
        """
        status_mapping = {
            GoalStatus.STATUS_UNKNOWN: "Unknown",
            GoalStatus.STATUS_ACCEPTED: "Accepted",
            GoalStatus.STATUS_EXECUTING: "Executing",
            GoalStatus.STATUS_SUCCEEDED: "Succeeded",
            GoalStatus.STATUS_CANCELED: "Canceled",
            GoalStatus.STATUS_ABORTED: "Aborted",
        }

        return status_mapping.get(goal_id, "Unknown")

    def cancel_goal_by_goal_id(self, goal_id: int) -> None:
        goal_handle = self.goal_manager.get_goal_handle(goal_id)

        if goal_handle is not None:
            asyncio.run_coroutine_threadsafe(
                goal_handle.cancel_goal_async(),
                self.bridge_node.asyncio_loop,
            )
            # We could check if the cancel request succeeded here.
            # Probably not necessary since the state can be checked by the client.
            return True
        else:
            return False

    def cancel_all_goals(self) -> None:
        for goal_id in self.goal_manager.goal_handles.keys():
            self.cancel_goal_by_goal_id(goal_id)

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
        state = self.GetState(request, context)
        return ArmPosition(
            shoulder_position=state.shoulder_state.present_position,
            elbow_position=state.elbow_state.present_position,
            wrist_position=state.wrist_state.present_position,
        )

    def GetJointGoalPosition(
        self, request: PartId, context: grpc.ServicerContext
    ) -> ArmPosition:
        state = self.GetState(request, context)
        return ArmPosition(
            shoulder_position=state.shoulder_state.goal_position,
            elbow_position=state.elbow_state.goal_position,
            wrist_position=state.wrist_state.goal_position,
        )

    # Compliances
    def set_stiffness(
        self, request: PartId, torque: bool, context: grpc.ServicerContext
    ) -> None:
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
        return Empty()

    def TurnOff(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        self.set_stiffness(request, torque=False, context=context)
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
        arm = self.get_arm_part_by_part_id(request.id, context)
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
        return ArmStatus()

    def HeartBeat(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        return Empty()

    def Restart(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        return Empty()

    def ResetDefaultValues(
        self, request: PartId, context: grpc.ServicerContext
    ) -> Empty:
        return Empty()


class GoalManager:
    def __init__(self):
        self.goal_handles = {}
        self.goal_id_counter = 0
        self.lock = threading.Lock()

    def generate_unique_id(self):
        with self.lock:
            self.goal_id_counter += 1
            return self.goal_id_counter

    def store_goal_handle(self, goal_handle):
        goal_id = self.generate_unique_id()
        self.goal_handles[goal_id] = goal_handle
        return goal_id

    def get_goal_handle(self, goal_id):
        return self.goal_handles.get(goal_id)

    def remove_goal_handle(self, goal_id):
        return self.goal_handles.pop(goal_id, None)
