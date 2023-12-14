import asyncio
import grpc
import rclpy
import threading

from typing import List, Optional
from google.protobuf.empty_pb2 import Empty
from reachy2_sdk_api.goto_pb2_grpc import add_GoToServiceServicer_to_server
from reachy2_sdk_api.part_pb2 import PartId
from reachy2_sdk_api.goto_pb2 import (
    CartesianGoal,
    JointsGoal,
    GoToId,
    GoToAck,
    GoToGoalStatus,
)
from action_msgs.msg import GoalStatus


from ..conversion import pose_from_pos_and_ori
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
        goal_handle = self.goal_manager.get_goal_handle(request.id)
        return GoToGoalStatus(goal_status=int(goal_handle.status))

    # Position and GoTo
    def GoToCartesian(
        self, request: CartesianGoal, context: grpc.ServicerContext
    ) -> GoToId:
        # TODO: implement grpc method
        # We do not take the duration or tolerance into account
        # We will develop a more advanced controller to handles this

        return GoToId(id=34)
        if request.HasField("arm_joint_goal"):
            # The message contains an arm_joint_goal
            arm_joint_goal = request.arm_joint_goal
            self.logger.info(f"arm_joint_goal: {arm_joint_goal}")
        elif request.HasField("neck_joint_goal"):
            # The message contains a neck_joint_goal
            neck_joint_goal = request.neck_joint_goal
            self.logger.info(f"neck_joint_goal: {neck_joint_goal}")
        else:
            # Neither arm_joint_goal nor neck_joint_goal is set
            # Handle this case accordingly
            None

        # arm = self.get_arm_part_by_part_id(request.id, context)
        # duration = request.duration.value

        # success, joint_position = self.bridge_node.compute_inverse(
        #     request.id,
        #     request.target.pose.data,
        #     arm_position_to_joint_state(request.q0, arm),
        # )  # 'joint_position': 'sensor_msgs/JointState'
        # # get the joint names and joint positions from the joint_state
        # joint_names = []
        # goal_positions = []
        # for i in range(len(joint_position.name)):
        #     joint_names.append(joint_position.name[i])
        #     goal_positions.append(joint_position.position[i])

        # self.logger.info(f"goal_positions: {goal_positions}")

        # future = asyncio.run_coroutine_threadsafe(
        #     self.bridge_node.send_goto_goal(
        #         arm.name,
        #         joint_names,
        #         goal_positions,
        #         duration,
        #         mode="minimum_jerk",
        #         feedback_callback=None,
        #         return_handle=True,
        #     ),
        #     self.bridge_node.asyncio_loop,
        # )
        # if future is None:
        #     self.logger.info("GotoGoal was rejected")
        #     ## TODO return -1
        #     return Empty()

        # # Wait for the result and get it => This has to be fast
        # goal_handle = future.result()

        # goal_id = self.goal_manager.store_goal_handle(goal_handle)
        # self.logger.info(f"goal_id: {goal_id}")
        # # TODO return unique id that represents this goal handle

    def GoToJoints(self, request: JointsGoal, context: grpc.ServicerContext) -> GoToId:
        if request.HasField("arm_joint_goal"):
            # The message contains an arm_joint_goal
            arm_joint_goal = request.arm_joint_goal  # this is an ArmJointGoal
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

            self.logger.info(f"goto goal_positions: {goal_positions}")

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
                return GoToId(id=-1)

            # Wait for the result and get it => This has to be fast
            goal_handle = future.result()

            goal_id = self.goal_manager.store_goal_handle(goal_handle)
            self.logger.info(f"goal_id: {goal_id}")

            return GoToId(id=goal_id)
        elif request.HasField("neck_joint_goal"):
            # The message contains a neck_joint_goal
            neck_joint_goal = (
                request.neck_joint_goal
            )  # this is a NeckGoal https://github.com/pollen-robotics/reachy2-sdk-api/blob/81-adjust-goto-methods/protos/head.proto
            self.logger.info(f"neck_joint_goal: {neck_joint_goal}\nTODO IMPLEMENT THIS")
            return GoToId(id=-1)
        else:
            self.logger.error(
                f"{request} is ill formed. Expected arm_joint_goal or neck_joint_goal"
            )
            return GoToId(id=-1)

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
