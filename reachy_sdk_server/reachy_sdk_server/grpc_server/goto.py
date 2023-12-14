import asyncio
import grpc
import rclpy

from google.protobuf.empty_pb2 import Empty
from reachy2_sdk_api.goto_pb2_grpc import add_GoToServiceServicer_to_server

from reachy2_sdk_api.goto_pb2 import (
    CartesianGoal,
    JointsGoal,
    GoToId,
    GoToAck,
    GoToGoalStatus,
)

from ..conversion import pose_from_pos_and_ori
from ..abstract_bridge_node import AbstractBridgeNode


class GoToServicer:
    def __init__(
        self,
        bridge_node: AbstractBridgeNode,
        logger: rclpy.impl.rcutils_logger.RcutilsLogger,
    ) -> None:
        self.bridge_node = bridge_node
        self.logger = logger

    def register_to_server(self, server: grpc.Server) -> None:
        self.logger.info("Registering 'GoToServiceServicer' to server.")
        add_GoToServiceServicer_to_server(self, server)

    def CancelAllGoTo(self, request: Empty, context: grpc.ServicerContext) -> Empty:
        # TODO: implement grpc method
        return Empty()

    def CancelGoTo(self, request: GoToId, context: grpc.ServicerContext) -> GoToAck:
        # TODO: implement grpc method
        return GoToAck(ack=True)

    def GetGoToState(self, request: GoToId, context: grpc.ServicerContext) -> GoToGoalStatus:
        # TODO: implement grpc method
        return GoToGoalStatus(status=0)

    # Position and GoTo
    def GoToCartesian(
        self, request: CartesianGoal, context: grpc.ServicerContext
    ) -> GoToId:
        # TODO: implement grpc method
        # We do not take the duration or tolerance into account
        # We will develop a more advanced controller to handles this
        return GoToId(id=34)

        self.bridge_node.publish_target_pose(
            request.id,
            pose_from_pos_and_ori(request.target_position, request.target_orientation),
        )

        return Empty()

    def GoToJoints(
        self, request: JointsGoal, context: grpc.ServicerContext
    ) -> GoToId:
        # TODO: implement grpc method
        return GoToId(id=32)
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

        asyncio.run_coroutine_threadsafe(
            self.bridge_node.send_goto_goal(
                arm.name,
                joint_names,
                goal_positions,
                duration,
                mode="minimum_jerk",
                feedback_callback=None,
                return_handle=False,
            ),
            self.bridge_node.asyncio_loop,
        )

        return Empty()
