import asyncio
import copy
import grpc
import rclpy
import threading
import math

from typing import List, Optional
from google.protobuf.empty_pb2 import Empty
from reachy2_sdk_api.goto_pb2_grpc import add_GoToServiceServicer_to_server
from reachy2_sdk_api.arm_pb2 import ArmCartesianGoal
from reachy2_sdk_api.part_pb2 import PartId
from reachy2_sdk_api.goto_pb2 import (
    CartesianGoal,
    JointsGoal,
    GoToId,
    GoToAck,
    GoToGoalStatus,
    GoToRequest,
    InterpolationMode,
)

from action_msgs.msg import GoalStatus
from sensor_msgs.msg import JointState


from ..conversion import (
    pose_from_pos_and_ori,
    arm_position_to_joint_state,
    rotation3d_as_extrinsinc_euler_angles,
)
from ..abstract_bridge_node import AbstractBridgeNode
from ..parts import Part


class GoToServicer:
    def __init__(
        self,
        bridge_node: AbstractBridgeNode,
        logger: rclpy.impl.rcutils_logger.RcutilsLogger,
    ) -> None:
        self.bridge_node = bridge_node
        self.logger = logger
        self.goal_manager = GoalManager()

    def register_to_server(self, server: grpc.Server) -> None:
        self.logger.info("Registering 'GoToServiceServicer' to server.")
        add_GoToServiceServicer_to_server(self, server)

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

    def get_neck_part_by_part_id(
        self, part_id: PartId, context: grpc.ServicerContext
    ) -> Part:
        part = self.bridge_node.parts.get_by_part_id(part_id)

        if part is None:
            context.abort(grpc.StatusCode.NOT_FOUND, f"Part not found (id={part_id}).")

        if part.type != "neck":
            context.abort(
                grpc.StatusCode.INVALID_ARGUMENT,
                f"Part '{part_id}' is not a nack.",
            )

        return part

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

    def CancelAllGoTo(self, request: Empty, context: grpc.ServicerContext) -> Empty:
        self.cancel_all_goals()
        return GoToAck(ack=True)

    def CancelGoTo(self, request: GoToId, context: grpc.ServicerContext) -> GoToAck:
        success = self.cancel_goal_by_goal_id(request.id)
        return GoToAck(ack=success)

    def GetGoToState(
        self, request: GoToId, context: grpc.ServicerContext
    ) -> GoToGoalStatus:
        # GoalStatus is one of:
        # STATUS_UNKNOWN, STATUS_ACCEPTED, STATUS_EXECUTING, STATUS_CANCELING, STATUS_SUCCEEDED, STATUS_CANCELED, STATUS_ABORTED
        goal_handle = self.goal_manager.get_goal_handle(request.id)
        if goal_handle is None:
            self.logger.error(
                f"Goal with id {request.id} not found. Returning:{1+int(GoalStatus.STATUS_UNKNOWN)}"
            )
            return GoToGoalStatus(goal_status=(1 + int(GoalStatus.STATUS_UNKNOWN)))
        else:
            self.logger.info(
                f"Goal with id {request.id} found, status:{goal_handle.status} Returning:{1+int(goal_handle.status)}"
            )
            return GoToGoalStatus(goal_status=(1 + int(goal_handle.status)))

    # Position and GoTo
    def GoToCartesian(
        self, request: GoToRequest, context: grpc.ServicerContext
    ) -> GoToId:

        interpolation_mode = self.get_interpolation_mode(request)

        if request.cartesian_goal.HasField("arm_cartesian_goal"):
            # this is an ArmCartesianGoal
            arm_cartesian_goal = request.cartesian_goal.arm_cartesian_goal

            arm = self.get_arm_part_by_part_id(arm_cartesian_goal.id, context)
            duration = arm_cartesian_goal.duration.value

            q0 = JointState()
            for c in arm.components:
                q0.name.extend(c.get_all_joints())

            if arm_cartesian_goal.HasField("q0"):
                q0_grpc = arm_cartesian_goal.q0
                q0.position = [
                    q0_grpc.shoulder_position.axis_1.value,
                    q0_grpc.shoulder_position.axis_2.value,
                    q0_grpc.elbow_position.axis_1.value,
                    q0_grpc.elbow_position.axis_2.value,
                    q0_grpc.wrist_position.rpy.roll,
                    q0_grpc.wrist_position.rpy.pitch,
                    q0_grpc.wrist_position.rpy.yaw,
                ]
            else:
                default_q0_position = [0.0, 0.0, 0.0, -math.pi / 2, 0.0, 0.0, 0.0]
                q0.position = default_q0_position

            success, joint_position = self.bridge_node.compute_inverse(
                arm_cartesian_goal.id,
                arm_cartesian_goal.goal_pose.data,
                q0,
            )  # 'joint_position': 'sensor_msgs/JointState'
            if not success:
                self.logger.error(
                    f"Could not compute inverse kinematics for arm {arm_cartesian_goal.id}"
                )
                return GoToId(id=-1)

            joint_names = joint_position.name
            goal_positions = joint_position.position

            arm = self.get_arm_part_by_part_id(arm_cartesian_goal.id, context)

            return self.goto_joints(
                arm.name,
                joint_names,
                goal_positions,
                duration,
                mode=interpolation_mode,
            )

        elif request.HasField("neck_cartesian_goal"):
            # this is a NeckCartesianGoal https://github.com/pollen-robotics/reachy2-sdk-api/blob/81-adjust-goto-methods/protos/head.proto
            neck_cartesian_goal = request.neck_cartesian_goal
            self.logger.info(
                f"neck_cartesian_goal: {neck_cartesian_goal}\nTODO IMPLEMENT THIS"
            )
            return GoToId(id=-1)
        else:
            self.logger.error(
                f"{request} is ill formed. Expected arm_cartesian_goal or neck_cartesian_goal"
            )
            return GoToId(id=-1)

    def GoToJoints(self, request: GoToRequest, context: grpc.ServicerContext) -> GoToId:

        self.logger.info(f"GoToJoints: {request}")
        interpolation_mode = self.get_interpolation_mode(request)
        if not interpolation_mode:
            return GoToId(id=-1)

        if request.joints_goal.HasField("arm_joint_goal"):
            # The message contains an arm_joint_goal
            arm_joint_goal = request.joints_goal.arm_joint_goal  # this is an ArmJointGoal
            arm = self.get_arm_part_by_part_id(arm_joint_goal.id, context)

            joint_names = self.part_to_list_of_joint_names(arm)

            duration = arm_joint_goal.duration.value

            goal_positions = [
                arm_joint_goal.joints_goal.shoulder_position.axis_1.value,
                arm_joint_goal.joints_goal.shoulder_position.axis_2.value,
                arm_joint_goal.joints_goal.elbow_position.axis_1.value,
                arm_joint_goal.joints_goal.elbow_position.axis_2.value,
                arm_joint_goal.joints_goal.wrist_position.rpy.roll,
                arm_joint_goal.joints_goal.wrist_position.rpy.pitch,
                arm_joint_goal.joints_goal.wrist_position.rpy.yaw,
            ]

            return self.goto_joints(
                arm.name,
                joint_names,
                goal_positions,
                duration,
                mode="minimum_jerk",
            )

        elif request.HasField("neck_joint_goal"):
            neck_joint_goal = request.neck_joint_goal  # this is a NeckGoal
            self.logger.info(
                f"neck_joint_goal: {neck_joint_goal}\n TODO this is not implemented yet, WIP"
            )
            neck = self.get_neck_part_by_part_id(neck_joint_goal.id, context)

            joint_names = self.part_to_list_of_joint_names(neck)

            duration = neck_joint_goal.duration.value

            goal_positions = rotation3d_as_extrinsinc_euler_angles(
                neck_joint_goal.joints_goal.rotation
            )

            return self.goto_joints(
                neck.name,
                joint_names,
                goal_positions,
                duration,
                mode=interpolation_mode,
            )
        else:
            self.logger.error(
                f"{request} is ill formed. Expected arm_joint_goal or neck_joint_goal"
            )
            return GoToId(id=-1)

    def goto_joints(
        self, part_name, joint_names, goal_positions, duration, mode="minimum_jerk"
    ):
        self.logger.info(f"goto goal_positions: {goal_positions}")
        future = asyncio.run_coroutine_threadsafe(
            self.bridge_node.send_goto_goal(
                part_name,
                joint_names,
                goal_positions,
                duration,
                mode="minimum_jerk",
                feedback_callback=None,
                return_handle=True,
            ),
            self.bridge_node.asyncio_loop,
        )

        # Wait for the result and get it => This has to be fast
        goal_handle = future.result()

        if goal_handle is None:
            self.logger.info("GotoGoal was rejected")
            return GoToId(id=-1)

        goal_id = self.goal_manager.store_goal_handle(goal_handle)
        self.logger.info(f"goal_id: {goal_id}")

        return GoToId(id=goal_id)

    def get_interpolation_mode(self, request: GoToRequest) -> str:
        interpolation_mode = request.interpolation_mode.interpolation_type
        if interpolation_mode == InterpolationMode.LINEAR:
            return "linear"
        elif interpolation_mode == InterpolationMode.MINIMUM_JERK:
            return "minimum_jerk"
        else:
            self.logger.error(
                f"Interpolation mode {interpolation_mode} not supported. Should be one of 'linear' or 'minimum_jerk'."
            )
            return None

    def cancel_goal_by_goal_id(self, goal_id: int) -> None:
        goal_handle = self.goal_manager.get_goal_handle(goal_id)

        if goal_handle is not None:
            self.logger.info(f"Cancelling goal with id {goal_id}")
            goal_handle.cancel_goal()
            self.logger.info(f"Goal with id {goal_id} cancelled")

            # asyncio.run_coroutine_threadsafe(
            #     goal_handle.cancel_goal_async(),
            #     self.bridge_node.asyncio_loop,
            # )
            # We could check if the cancel request succeeded here.
            # Probably not necessary since the state can be checked by the client.
            return True
        else:
            return False

    def cancel_all_goals(self) -> None:
        for goal_id in self.goal_manager.goal_handles.keys():
            self.cancel_goal_by_goal_id(goal_id)


class GoalManager:
    # TODO decide how/when to remove goal handles from the dict. Also investigate the bug that appears when spamming gotos.
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
        return self.goal_handles.get(goal_id, None)

    def remove_goal_handle(self, goal_id):
        return self.goal_handles.pop(goal_id, None)
