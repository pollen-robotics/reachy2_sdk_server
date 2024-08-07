import grpc
import rclpy
from control_msgs.msg import DynamicJointState, InterfaceValue
from google.protobuf.empty_pb2 import Empty
from reachy2_sdk_api.component_pb2 import ComponentId
from reachy2_sdk_api.head_pb2 import (
    Head,
    HeadDescription,
    HeadPosition,
    HeadState,
    HeadStatus,
    HeadTemperatures,
    JointsLimits,
    ListOfHead,
    NeckFKRequest,
    NeckFKSolution,
    NeckIKRequest,
    NeckIKSolution,
    NeckJointGoal,
    NeckOrientation,
    SpeedLimitRequest,
    TorqueLimitRequest,
)
from reachy2_sdk_api.head_pb2_grpc import add_HeadServiceServicer_to_server
from reachy2_sdk_api.kinematics_pb2 import Quaternion, Rotation3d
from reachy2_sdk_api.orbita3d_pb2 import Orbita3dStatus
from reachy2_sdk_api.part_pb2 import PartId
from sensor_msgs.msg import JointState

from ..abstract_bridge_node import AbstractBridgeNode
from ..conversion import (
    extract_quaternion_from_pose_matrix,
    joint_state_to_neck_orientation,
    matrix_to_pose,
    neck_rotation_to_joint_state,
    pose_matrix_from_rotation3d,
    quat_as_rotation3d,
    rotation3d_as_quat,
)
from ..parts import Part
from ..utils import get_current_timestamp
from .orbita3d import Orbita3dCommand, Orbita3dsCommand, Orbita3dServicer, Orbita3dStateRequest

from pollen_msgs.msg import CartTarget
from . import tracing_helper
from opentelemetry import trace


class HeadServicer:
    def __init__(
        self,
        bridge_node: AbstractBridgeNode,
        logger: rclpy.impl.rcutils_logger.RcutilsLogger,
        orbita3d_servicer: Orbita3dServicer,
    ) -> None:
        self.bridge_node = bridge_node
        self.logger = logger

        self.orbita3d_servicer = orbita3d_servicer

        self.heads = self.bridge_node.parts.get_by_type("head")

    def register_to_server(self, server: grpc.Server):
        self.logger.info("Registering 'HeadServiceServicer' to server.")
        add_HeadServiceServicer_to_server(self, server)

    def get_head(self, head: Part, context: grpc.ServicerContext) -> Head:
        return Head(
            part_id=PartId(name=head.name, id=head.id),
            description=HeadDescription(
                neck=Orbita3dServicer.get_info(self.bridge_node.components.get_by_name(head.components[0].name)),
                # l_antenna=DynamixelMotor.get_info(
                #     self.bridge_node.components.get_by_name(head.components[1].name)
                # ),
                # r_antenna=DynamixelMotor.get_info(
                #     self.bridge_node.components.get_by_name(head.components[2].name)
                # ),
            ),
        )

    def get_head_part_from_part_id(self, part_id: PartId, context: grpc.ServicerContext) -> Part:
        part = self.bridge_node.parts.get_by_part_id(part_id)

        if part is None:
            context.abort(grpc.StatusCode.NOT_FOUND, f"Part not found (id={part_id}).")

        if part.type != "head":
            context.abort(
                grpc.StatusCode.INVALID_ARGUMENT,
                f"Part '{part_id}' is not an head.",
            )

        return part

    def GetAllHeads(self, request: Empty, context: grpc.ServicerContext) -> ListOfHead:
        return ListOfHead(head=[self.get_head(head, context) for head in self.heads])

    def GetState(self, request: PartId, context: grpc.ServicerContext) -> HeadState:
        head = self.get_head_part_from_part_id(request, context)

        return HeadState(
            timestamp=get_current_timestamp(self.bridge_node),
            id=request,
            activated=True,
            neck_state=self.orbita3d_servicer.GetState(
                Orbita3dStateRequest(
                    fields=self.orbita3d_servicer.default_fields,
                    id=ComponentId(id=head.components[0].id),
                ),
                context,
            ),
            # l_antenna_state=self.dynamixel_servicer.GetState(
            #     DynamixelStateRequest(
            #         fields=self.dynamixel_servicer.default_fields,
            #         id=ComponentId(id=head.components[1].id),
            #     ),
            #     context,
            # ),
            # r_antenna_state=self.dynamixel_servicer.GetState(
            #     DynamixelStateRequest(
            #         fields=self.dynamixel_servicer.default_fields,
            #         id=ComponentId(id=head.components[2].id),
            #     ),
            #     context,
            # ),
        )

    def ComputeNeckFK(self, request: NeckFKRequest, context: grpc.ServicerContext) -> NeckFKSolution:
        head = self.get_head_part_from_part_id(request.id, context)
        success, pose = self.bridge_node.compute_forward(
            request.id,
            neck_rotation_to_joint_state(request.position.neck_position, head),
        )

        sol = NeckFKSolution()

        if success:
            sol.success = True
            x, y, z, w = extract_quaternion_from_pose_matrix(pose)
            sol.orientation.rotation.q.x = x
            sol.orientation.rotation.q.y = y
            sol.orientation.rotation.q.z = z
            sol.orientation.rotation.q.w = w

        return sol

    def ComputeNeckIK(self, request: NeckIKRequest, context: grpc.ServicerContext) -> NeckIKSolution:
        head = self.get_head_part_from_part_id(request.id, context)

        M = pose_matrix_from_rotation3d(request.target.rotation)

        try:
            q0 = neck_rotation_to_joint_state(request.q0, head)
        except ValueError:
            q0 = JointState()

        success, joint_position = self.bridge_node.compute_inverse(
            request.id,
            M,
            q0,
        )

        sol = NeckIKSolution()

        if success:
            sol.success = True
            sol.position.CopyFrom(joint_state_to_neck_orientation(joint_position, head))

        return sol

    def GetOrientation(self, request: PartId, context: grpc.ServicerContext) -> Rotation3d:
        rot = self.GetState(request, context).neck_state.present_position

        fk_req = NeckFKRequest(
            id=request,
            position=HeadPosition(
                neck_position=rot,
            ),
        )
        resp = self.ComputeNeckFK(fk_req, context)

        if not resp.success:
            context.abort(grpc.StatusCode.INTERNAL, "Could not compute FK.")

        return resp.orientation.rotation

    def Audit(self, request: PartId, context: grpc.ServicerContext) -> HeadStatus:
        head = self.get_head_part_from_part_id(request, context)

        return HeadStatus(
            neck_status=self.orbita3d_servicer.Audit(
                ComponentId(id=head.components[0].id),
                context,
            ),
        )

    def HeartBeat(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        return Empty()

    def Restart(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        return Empty()

    def ResetDefaultValues(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        return Empty()

    # Compliances
    def set_stiffness(self, request: PartId, torque: bool, context: grpc.ServicerContext) -> None:
        # TODO: re-write using self.orbita3d_servicer.SendCommand?
        # TODO: check id
        head = self.get_head_part_from_part_id(request, context)

        cmd = DynamicJointState()
        cmd.joint_names = []

        for c in head.components:
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
        part = self.get_head_part_from_part_id(request, context)
        self.bridge_node.set_all_joints_to_current_position(part.name)
        return Empty()

    def TurnOff(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        self.set_stiffness(request, torque=False, context=context)
        return Empty()

    def GetJointsLimits(self, request: PartId, context: grpc.ServicerContext) -> JointsLimits:
        return JointsLimits()

    def GetTemperatures(self, request: PartId, context: grpc.ServicerContext) -> HeadTemperatures:
        return HeadTemperatures()

    def GetJointGoalPosition(self, request: PartId, context: grpc.ServicerContext) -> Rotation3d:
        rot = self.GetState(request, context).neck_state.goal_position

        fk_req = NeckFKRequest(
            id=request,
            position=HeadPosition(
                neck_position=rot,
            ),
        )
        resp = self.ComputeNeckFK(fk_req, context)

        if not resp.success:
            context.abort(grpc.StatusCode.INTERNAL, "Could not compute FK.")

        return Rotation3d(
            q=resp.orientation.q,
        )

    def SetSpeedLimit(self, request: SpeedLimitRequest, context: grpc.ServicerContext) -> Empty:
        with tracing_helper.PollenSpan(tracer=self.bridge_node.tracer, trace_name=f"SetSpeedLimit"):
            # TODO: re-write using self.orbita2d_servicer.SendCommand?
            part = self.get_head_part_from_part_id(request.id, context)

            cmd = DynamicJointState()
            cmd.joint_names = []

            for c in part.components:
                for i in range(1, 4):
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
            part = self.get_head_part_from_part_id(request.id, context)

            cmd = DynamicJointState()
            cmd.joint_names = []

            for c in part.components:
                for i in range(1, 4):
                    cmd.joint_names.append(f"{c.name}_raw_motor_{i}")

                    cmd.interface_values.append(
                        InterfaceValue(
                            interface_names=["torque_limit"],
                            values=[request.limit / 100],
                        )
                    )

            self.bridge_node.publish_command(cmd)
        return Empty()

    # Note: this version uses an IK service call.
    # --> rlcpy services seem too slow si we're disabling this and using a topic mechnaisms instead.
    # def SendNeckJointGoal(self, request: NeckJointGoal, context: grpc.ServicerContext) -> Empty:
    #     head = self.get_head_part_from_part_id(request.id, context)

    #     q = rotation3d_as_quat(request.joints_goal.rotation)

    #     ik_req = NeckIKRequest(
    #         id=request.id,
    #         target=NeckOrientation(
    #             rotation=quat_as_rotation3d(q),
    #         ),
    #     )
    #     resp = self.ComputeNeckIK(ik_req, context)

    #     if not resp.success:
    #         context.abort(grpc.StatusCode.INTERNAL, "Could not compute IK.")

    #     self.orbita3d_servicer.SendCommand(
    #         Orbita3dsCommand(
    #             cmd=[
    #                 Orbita3dCommand(
    #                     id=ComponentId(id=head.components[0].id),
    #                     goal_position=resp.position,
    #                 ),
    #             ]
    #         ),
    #         context,
    #     )

    #     return Empty()

    def SendNeckJointGoal(self, request: NeckJointGoal, context: grpc.ServicerContext) -> Empty:
        with tracing_helper.PollenSpan(tracer=self.bridge_node.tracer, trace_name=f"SendNeckJointGoal"):
            msg = CartTarget()
            msg.traceparent = tracing_helper.traceparent()

            M = pose_matrix_from_rotation3d(request.joints_goal.rotation)
            msg.pose.pose = matrix_to_pose(M)

            with tracing_helper.PollenSpan(tracer=self.bridge_node.tracer,
                                           trace_name="bridge_node.publish_head_target_pose",
                                           kind=trace.SpanKind.CLIENT):
                msg.pose.header.stamp = self.bridge_node.get_clock().now().to_msg()
                self.bridge_node.publish_head_target_pose(
                    request.id,
                    msg,
                )

        return Empty()
