import asyncio
import math
import threading
from typing import List, Optional, Tuple

import grpc
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from google.protobuf.empty_pb2 import Empty
from google.protobuf.wrappers_pb2 import FloatValue
from reachy2_sdk_api.arm_pb2 import ArmJointGoal, ArmPosition
from reachy2_sdk_api.goto_pb2 import (
    GoToAck,
    GoToGoalStatus,
    GoToId,
    GoToInterpolation,
    GoToQueue,
    GoToRequest,
    InterpolationMode,
    JointsGoal,
)
from reachy2_sdk_api.goto_pb2_grpc import add_GoToServiceServicer_to_server
from reachy2_sdk_api.head_pb2 import NeckJointGoal, NeckOrientation
from reachy2_sdk_api.kinematics_pb2 import ExtEulerAngles, Quaternion, Rotation3d
from reachy2_sdk_api.orbita2d_pb2 import Pose2d
from reachy2_sdk_api.part_pb2 import PartId
from sensor_msgs.msg import JointState

from ..abstract_bridge_node import AbstractBridgeNode
from ..conversion import pose_matrix_from_quaternion, rotation3d_as_extrinsinc_euler_angles
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

    def get_part_by_part_id(self, part_id: PartId, context: grpc.ServicerContext) -> Optional[Part]:
        part = self.bridge_node.parts.get_by_part_id(part_id)

        if part is None:
            context.abort(grpc.StatusCode.NOT_FOUND, f"Part not found (id={part_id}).")

        return part

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

    def get_head_part_by_part_id(self, part_id: PartId, context: grpc.ServicerContext) -> Part:
        part = self.bridge_node.parts.get_by_part_id(part_id)

        if part is None:
            context.abort(grpc.StatusCode.NOT_FOUND, f"Part not found (id={part_id}).")

        if part.type != "head":
            context.abort(
                grpc.StatusCode.INVALID_ARGUMENT,
                f"Part '{part_id}' is not a head.",
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

    def GetGoToState(self, request: GoToId, context: grpc.ServicerContext) -> GoToGoalStatus:
        # GoalStatus is one of:
        # STATUS_UNKNOWN, STATUS_ACCEPTED, STATUS_EXECUTING, STATUS_CANCELING, STATUS_SUCCEEDED, STATUS_CANCELED, STATUS_ABORTED
        goal_handle = self.goal_manager.get_goal_handle(request.id)
        if goal_handle is None:
            self.logger.error(f"Goal with id {request.id} not found. Returning:{1+int(GoalStatus.STATUS_UNKNOWN)}")
            return GoToGoalStatus(goal_status=(1 + int(GoalStatus.STATUS_UNKNOWN)))
        else:
            self.logger.debug(
                f"Goal with id {request.id} found, status:{goal_handle.status} Returning:{1+int(goal_handle.status)}"
            )
            return GoToGoalStatus(goal_status=(1 + int(goal_handle.status)))

    # Position and GoTo
    def GoToCartesian(self, request: GoToRequest, context: grpc.ServicerContext) -> GoToId:
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
                    q0_grpc.wrist_position.rpy.roll.value,
                    q0_grpc.wrist_position.rpy.pitch.value,
                    q0_grpc.wrist_position.rpy.yaw.value,
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
                self.logger.error(f"Could not compute inverse kinematics for arm {arm_cartesian_goal.id}")
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

        elif request.cartesian_goal.HasField("neck_cartesian_goal"):
            # this is a NeckCartesianGoal https://github.com/pollen-robotics/reachy2-sdk-api/blob/81-adjust-goto-methods/protos/head.proto
            neck_cartesian_goal = request.cartesian_goal.neck_cartesian_goal
            head = self.get_head_part_by_part_id(neck_cartesian_goal.id, context)

            # joint_names = self.part_to_list_of_joint_names(head)

            duration = neck_cartesian_goal.duration.value
            x = neck_cartesian_goal.point.x
            y = neck_cartesian_goal.point.y
            z = neck_cartesian_goal.point.z

            q_numpy = _find_neck_quaternion_transform([1, 0, 0], [x, y, z])
            q_quat = Quaternion(x=q_numpy[0], y=q_numpy[1], z=q_numpy[2], w=q_numpy[3])
            M = pose_matrix_from_quaternion(q_quat)
            q0 = JointState()
            q0.position = [0.0, 0.0, 0.0]

            success, joint_position = self.bridge_node.compute_inverse(
                neck_cartesian_goal.id,
                M,
                q0,
            )

            if not success:
                self.logger.error(f"Could not compute inverse kinematics for arm {arm_cartesian_goal.id}")
                return GoToId(id=-1)

            joint_names = joint_position.name
            goal_positions = joint_position.position

            return self.goto_joints(
                "neck",
                joint_names,
                goal_positions,
                duration,
                mode=interpolation_mode,
            )
        else:
            self.logger.error(f"{request} is ill formed. Expected arm_cartesian_goal or neck_cartesian_goal")
            return GoToId(id=-1)

    def GoToJoints(self, request: GoToRequest, context: grpc.ServicerContext) -> GoToId:
        self.logger.debug(f"GoToJoints: {request}")
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
                arm_joint_goal.joints_goal.wrist_position.rpy.roll.value,
                arm_joint_goal.joints_goal.wrist_position.rpy.pitch.value,
                arm_joint_goal.joints_goal.wrist_position.rpy.yaw.value,
            ]

            return self.goto_joints(
                arm.name,
                joint_names,
                goal_positions,
                duration,
                mode=interpolation_mode,
            )

        elif request.joints_goal.HasField("neck_joint_goal"):
            neck_joint_goal = request.joints_goal.neck_joint_goal  # this is a NeckGoal
            head = self.get_head_part_by_part_id(neck_joint_goal.id, context)

            joint_names = self.part_to_list_of_joint_names(head)

            duration = neck_joint_goal.duration.value

            goal_positions = rotation3d_as_extrinsinc_euler_angles(neck_joint_goal.joints_goal.rotation)

            return self.goto_joints(
                "neck",
                joint_names,
                goal_positions,
                duration,
                mode=interpolation_mode,
            )
        else:
            self.logger.error(f"{request} is ill formed. Expected arm_joint_goal or neck_joint_goal")
            return GoToId(id=-1)

    def GetGoToRequest(self, goto_id: GoToId, context: grpc.ServicerContext) -> GoToRequest:
        return self.get_goal_request_by_goal_id(goto_id.id, context)

    def GetPartGoToPlaying(self, part_id: PartId, context: grpc.ServicerContext) -> GoToId:
        part_name = self.get_part_by_part_id(part_id, context)
        return self.get_part_goto_playing(part_name.name)

    def GetPartGoToQueue(self, part_id: PartId, context: grpc.ServicerContext) -> GoToQueue:
        part_name = self.get_part_by_part_id(part_id, context)
        return self.get_part_queue(part_name.name)

    def CancelPartAllGoTo(self, part_id: PartId, context: grpc.ServicerContext) -> GoToAck:
        part_name = self.get_part_by_part_id(part_id, context)
        self.cancel_part_all_goals(part_name.name)
        return GoToAck(ack=True)

    def goto_joints(self, part_name, joint_names, goal_positions, duration, mode="minimum_jerk"):
        future = asyncio.run_coroutine_threadsafe(
            self.bridge_node.send_goto_goal(
                part_name,
                joint_names,
                goal_positions,
                duration,
                mode=mode,
                feedback_callback=None,
                return_handle=True,
            ),
            self.bridge_node.asyncio_loop,
        )

        # Wait for the result and get it => This has to be fast
        goal_handle = future.result()

        goal_request = {}
        goal_request["goal_positions"] = goal_positions
        goal_request["duration"] = duration
        goal_request["mode"] = mode

        if goal_handle is None:
            self.logger.info("GotoGoal was rejected")
            return GoToId(id=-1)

        if part_name == "neck":
            part_name = "head"
        goal_id = self.goal_manager.store_goal_handle(part_name, goal_handle, goal_request)

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

    def _get_grpc_interpolation_mode(self, interpolation_mode: str) -> InterpolationMode:
        if interpolation_mode == "linear":
            return InterpolationMode.LINEAR
        elif interpolation_mode == "minimum_jerk":
            return InterpolationMode.MINIMUM_JERK
        else:
            self.logger.error(
                f"Interpolation mode {interpolation_mode} not supported. Should be one of 'linear' or 'minimum_jerk'."
            )
            return None

    def get_part_queue(self, part_name: str) -> GoToQueue:
        goal_ids_int = getattr(self.goal_manager, part_name + "_goal")
        goal_ids = [
            GoToId(id=goal_id_int)
            for goal_id_int in goal_ids_int
            if self.goal_manager.goal_handles[goal_id_int].status in [0, 1]
        ]
        return GoToQueue(goto_ids=goal_ids)

    def get_part_goto_playing(self, part_name: str) -> GoToId:
        goal_ids = getattr(self.goal_manager, part_name + "_goal")
        for goal_id in goal_ids:
            if self.goal_manager.goal_handles[goal_id].status == 2:
                return GoToId(id=goal_id)
        return GoToId(id=-1)

    def get_goal_request_by_goal_id(self, goal_id: int, context: grpc.ServicerContext) -> GoToRequest:
        goal_request = self.goal_manager.goal_requests[goal_id]
        mode = self._get_grpc_interpolation_mode(goal_request["mode"])
        duration = goal_request["duration"]
        joints_goal = goal_request["goal_positions"]
        part = None

        if goal_id in self.goal_manager.r_arm_goal:
            part = self.bridge_node.parts.get_by_name("r_arm")
        elif goal_id in self.goal_manager.l_arm_goal:
            part = self.bridge_node.parts.get_by_name("l_arm")
        if part is not None:
            part_id = PartId(id=part.id, name=part.name)
            arm_joint_goal = ArmJointGoal(
                id=part_id,
                joints_goal=ArmPosition(
                    shoulder_position=Pose2d(axis_1=FloatValue(value=joints_goal[0]), axis_2=FloatValue(value=joints_goal[1])),
                    elbow_position=Pose2d(axis_1=FloatValue(value=joints_goal[2]), axis_2=FloatValue(value=joints_goal[3])),
                    wrist_position=Rotation3d(
                        rpy=ExtEulerAngles(
                            roll=FloatValue(value=joints_goal[4]),
                            pitch=FloatValue(value=joints_goal[5]),
                            yaw=FloatValue(value=joints_goal[6]),
                        )
                    ),
                ),
                duration=FloatValue(value=duration),
            )

            request = GoToRequest(
                joints_goal=JointsGoal(arm_joint_goal=arm_joint_goal),
                interpolation_mode=GoToInterpolation(interpolation_type=mode),
            )

            return request

        if goal_id in self.goal_manager.head_goal:
            part = self.bridge_node.parts.get_by_name("head")
        if part is not None:
            part_id = PartId(id=part.id, name=part.name)
            neck_joint_goal = NeckJointGoal(
                id=part_id,
                joints_goal=NeckOrientation(
                    rotation=Rotation3d(
                        rpy=ExtEulerAngles(
                            roll=FloatValue(value=joints_goal[0]),
                            pitch=FloatValue(value=joints_goal[1]),
                            yaw=FloatValue(value=joints_goal[2]),
                        )
                    ),
                ),
                duration=FloatValue(value=duration),
            )

            request = GoToRequest(
                joints_goal=JointsGoal(neck_joint_goal=neck_joint_goal),
                interpolation_mode=GoToInterpolation(interpolation_type=mode),
            )

            return request

        else:
            context.abort(grpc.StatusCode.NOT_FOUND, f"GoalId not found (id={goal_id}).")

    def cancel_goal_by_goal_id(self, goal_id: int) -> bool:
        goal_handle = self.goal_manager.get_goal_handle(goal_id)

        if goal_handle is not None:
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

    def cancel_part_all_goals(self, part_name: str) -> None:
        part_goal_ids = getattr(self.goal_manager, part_name + "_goal")
        for goal_id in part_goal_ids:
            self.cancel_goal_by_goal_id(goal_id)

    def cancel_all_goals(self) -> None:
        for goal_id in self.goal_manager.goal_handles.keys():
            self.cancel_goal_by_goal_id(goal_id)


class GoalManager:
    # TODO decide how/when to remove goal handles from the dict. Also investigate the bug that appears when spamming gotos.
    def __init__(self):
        self.goal_handles = {}
        self.goal_requests = {}
        self.r_arm_goal = []
        self.l_arm_goal = []
        self.head_goal = []
        self.goal_id_counter = 0
        self.lock = threading.Lock()

    def generate_unique_id(self):
        with self.lock:
            self.goal_id_counter += 1
            return self.goal_id_counter

    def store_goal_handle(self, part_name: str, goal_handle, goal_request: dict[str, List[float] | float | str]):
        goal_id = self.generate_unique_id()
        self.goal_handles[goal_id] = goal_handle
        getattr(self, part_name + "_goal").append(goal_id)
        self.goal_requests[goal_id] = goal_request
        return goal_id

    def get_goal_handle(self, goal_id):
        return self.goal_handles.get(goal_id, None)

    def remove_goal_handle(self, goal_id):
        return self.goal_handles.pop(goal_id, None)


def _find_neck_quaternion_transform(
    vect_origin: Tuple[float, float, float],
    vect_target: Tuple[float, float, float],
) -> Tuple[float, float, float, float]:
    vo = _norm(vect_origin)

    # ad-hoc translation to move in the torso frame (from urdf). TODO do better?
    neck_in_torso = (vect_target[0] - 0.015, vect_target[1], vect_target[2] - 0.095)
    # simple approximation, hopefully good enough...
    head_in_torso = (
        neck_in_torso[0] - 0.02,
        neck_in_torso[1],
        neck_in_torso[2] - 0.06105,
    )

    vd = _norm(head_in_torso)

    v = np.cross(vo, vd)
    v = _norm(v)

    alpha = np.arccos(np.dot(vo, vd))
    if np.isnan(alpha) or alpha < 1e-6:
        return (0, 0, 0, 1)

    q = _from_axis_angle(axis=v, angle=alpha)
    return q


def _norm(v):
    v = np.array(v)
    if np.any(v):
        v = v / np.linalg.norm(v)
    return v


def _from_axis_angle(axis, angle):
    mag_sq = np.dot(axis, axis)
    if mag_sq == 0.0:
        raise ValueError("Rotation axis must be non-zero")

    if abs(1.0 - mag_sq) > 1e-12:
        axis = axis / np.sqrt(mag_sq)

    theta = angle / 2.0
    r = np.cos(theta)
    i = axis * np.sin(theta)

    return np.array([i[0], i[1], i[2], r])
