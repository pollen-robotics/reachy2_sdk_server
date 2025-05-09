"""Expose main mobile base ROS services/topics through gRPC allowing remote client SDK."""

import io
from queue import Empty
from subprocess import PIPE, check_output, run
from typing import Optional

import grpc
import rclpy
import tf_transformations
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, Twist
from google.protobuf.empty_pb2 import Empty
from google.protobuf.wrappers_pb2 import BoolValue, FloatValue
from nav_msgs.msg import Odometry
from PIL import Image as PilImage
from reachy2_sdk_api.mobile_base_lidar_pb2 import (
    LidarMap,
    LidarObstacleDetectionEnum,
    LidarObstacleDetectionStatus,
    LidarSafety,
)
from reachy2_sdk_api.mobile_base_lidar_pb2_grpc import (
    MobileBaseLidarServiceServicer,
    add_MobileBaseLidarServiceServicer_to_server,
)
from reachy2_sdk_api.mobile_base_mobility_pb2 import (
    DirectionVector,
    DistanceToGoalVector,
    MobilityServiceAck,
    SetSpeedVector,
    TargetDirectionCommand,
)
from reachy2_sdk_api.mobile_base_mobility_pb2_grpc import (
    MobileBaseMobilityServiceServicer,
    add_MobileBaseMobilityServiceServicer_to_server,
)
from reachy2_sdk_api.mobile_base_utility_pb2 import (
    BatteryLevel,
    ControlModeCommand,
    ControlModePossiblities,
    ListOfMobileBase,
    MobileBase,
    MobileBaseState,
    MobileBaseStatus,
    OdometryVector,
    ZuuuModeCommand,
    ZuuuModePossiblities,
)
from reachy2_sdk_api.mobile_base_utility_pb2_grpc import (
    MobileBaseUtilityServiceServicer,
    add_MobileBaseUtilityServiceServicer_to_server,
)
from reachy2_sdk_api.part_pb2 import PartId, PartInfo
from reachy2_sdk_api.reachy_pb2 import ReachyCoreMode
from sensor_msgs.msg import Image
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from zuuu_interfaces.srv import (
    DistanceToGoal,
    GetBatteryVoltage,
    GetOdometry,
    GetZuuuMode,
    GetZuuuSafety,
    ResetOdometry,
    SetSpeed,
    SetZuuuMode,
    SetZuuuSafety,
)

from ..abstract_bridge_node import AbstractBridgeNode
from ..parts import Part
from ..utils import get_current_timestamp


class MobileBaseServicer(
    MobileBaseLidarServiceServicer,
    MobileBaseMobilityServiceServicer,
    MobileBaseUtilityServiceServicer,
):
    """Mobile base SDK server node."""

    def __init__(
        self,
        bridge_node: AbstractBridgeNode,
        logger: rclpy.impl.rcutils_logger.RcutilsLogger,
        mobile_base_config: dict,
        core_mode: ReachyCoreMode = ReachyCoreMode.FAKE,
    ) -> None:
        """Set up the node.

        Get mobile base basic info such as its odometry, battery level, drive mode or control mode
        from the mobile base hal.
        Send commands through the SetSpeed service or by publishing to cmd_vel topic.
        """
        self.logger = logger
        self.bridge_node = bridge_node
        self.core_mode = core_mode
        self.mobile_base_enabled = True  # Keep track of mobile base status in order to return None for teleop

        self.info = {
            "serial_number": mobile_base_config["serial_number"],
            "version_hard": mobile_base_config["version_hard"],
            "version_soft": mobile_base_config["version_soft"],
        }

        if not mobile_base_config["enable"]:
            self.logger.info("No mobile base found in the config file. Mobile base server not initialized.")
            self.mobile_base_enabled = False
            self.publish_static_odom()
            return

        self.cmd_vel_pub = self.bridge_node.create_publisher(Twist, "cmd_vel", 10)

        self.bridge = CvBridge()
        self.lidar_img_subscriber = self.bridge_node.create_subscription(Image, "lidar_image", self.get_lidar_img, 1)
        self._last_odom = None
        self._last_direction = TargetDirectionCommand()

        self.odometry_subscriber = self.bridge_node.create_subscription(Odometry, "odom", self.odom_cb, 1)

        self.set_speed_client = self.bridge_node.create_client(SetSpeed, "SetSpeed")
        while not self.set_speed_client.wait_for_service(timeout_sec=1.0):
            self.logger.info("service SetSpeed not available, waiting again...")

        self.distance_to_goal_client = self.bridge_node.create_client(DistanceToGoal, "DistanceToGoal")
        while not self.distance_to_goal_client.wait_for_service(timeout_sec=1.0):
            self.logger.info("service DistanceToGoal not available, waiting again...")

        self.set_zuuu_mode_client = self.bridge_node.create_client(SetZuuuMode, "SetZuuuMode")
        while not self.set_zuuu_mode_client.wait_for_service(timeout_sec=1.0):
            self.logger.info("service SetZuuuMode not available, waiting again...")

        self.get_zuuu_mode_client = self.bridge_node.create_client(GetZuuuMode, "GetZuuuMode")
        while not self.get_zuuu_mode_client.wait_for_service(timeout_sec=1.0):
            self.logger.info("service GetZuuuMode not available, waiting again...")

        self.get_battery_voltage_client = self.bridge_node.create_client(GetBatteryVoltage, "GetBatteryVoltage")
        while not self.get_battery_voltage_client.wait_for_service(timeout_sec=1.0):
            self.logger.info("service GetBatteryVoltage not available, waiting again...")

        self.get_odometry_client = self.bridge_node.create_client(GetOdometry, "GetOdometry")
        while not self.get_odometry_client.wait_for_service(timeout_sec=1.0):
            self.logger.info("service GetOdometry not available, waiting again...")

        self.reset_odometry_client = self.bridge_node.create_client(ResetOdometry, "ResetOdometry")
        while not self.reset_odometry_client.wait_for_service(timeout_sec=1.0):
            self.logger.info("service ResetOdometry not available, waiting again...")

        self.set_zuuu_safety_client = self.bridge_node.create_client(SetZuuuSafety, "SetZuuuSafety")
        while not self.set_zuuu_safety_client.wait_for_service(timeout_sec=1.0):
            self.logger.info("service SetZuuuSafety not available, waiting again...")

        self.get_zuuu_safety_client = self.bridge_node.create_client(GetZuuuSafety, "GetZuuuSafety")
        while not self.get_zuuu_safety_client.wait_for_service(timeout_sec=1.0):
            self.logger.info("service GetZuuuSafety not available, waiting again...")

        # TODO: handle part_id correctly
        self.mobile_bases = self.bridge_node.parts.get_by_type("mobile_base")
        self.logger.info("Initialized mobile base server.")

    def register_to_server(self, server: grpc.Server) -> None:
        """Register the servicer to the server."""
        self.logger.info("Registering 'MobileBaseServicer' to server.")
        add_MobileBaseLidarServiceServicer_to_server(self, server)
        add_MobileBaseMobilityServiceServicer_to_server(self, server)
        add_MobileBaseUtilityServiceServicer_to_server(self, server)

    def publish_static_odom(self):
        """Publish static odometry<->base_link TF and one message on /odom no mobile base is connected.
        odom and base_link should be the same frame since the robot is static.
        """
        self.tf_static_broadcaster = StaticTransformBroadcaster(self.bridge_node)
        self.pub_odom = self.bridge_node.create_publisher(Odometry, "odom", 2)
        # Odom
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.header.stamp = self.bridge_node.get_clock().now().to_msg()
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 0.0

        q = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        self.pub_odom.publish(odom)

        # TF
        t = TransformStamped()
        t.header.stamp = self.bridge_node.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_static_broadcaster.sendTransform(t)

    def get_mobile_base(self, mobile_base: Optional[Part], context: grpc.ServicerContext) -> MobileBase:
        """Get mobile base basic info."""
        if self.mobile_base_enabled:
            if mobile_base is None:
                mobile_base = self.mobile_bases[0]
            return MobileBase(
                part_id=PartId(name=mobile_base.name, id=mobile_base.id),
                info=PartInfo(
                    serial_number=self.info["serial_number"],
                    version_hard=str(self.info["version_hard"]),
                    version_soft=str(self.info["version_soft"]),
                ),
            )
        else:
            return None

    def get_lidar_img(self, msg):
        self.lidar_img = self.bridge.imgmsg_to_cv2(msg)

    def GetAllMobileBases(self, request: Empty, context: grpc.ServicerContext) -> ListOfMobileBase:
        return ListOfMobileBase(mobile_base=[self.GetMobileBase(Empty(), context)])

    def GetMobileBase(self, request: Empty, context) -> PartInfo:
        """Get mobile base basic info."""
        return self.get_mobile_base(None, context)

    def GetState(self, request: PartId, context) -> MobileBaseState:
        """Get mobile base state."""
        if not self.mobile_base_enabled:
            return MobileBaseState()

        res_status = LidarSafety(id=request)
        lidar_info = self.bridge_node.get_safety_status()
        status_number = lidar_info["status"]
        if status_number == 0.0:
            grpc_obstacle_detection_status = LidarObstacleDetectionEnum.DETECTION_ERROR
        elif status_number == 1.0:
            grpc_obstacle_detection_status = LidarObstacleDetectionEnum.NO_OBJECT_DETECTED
        elif status_number == 2.0:
            grpc_obstacle_detection_status = LidarObstacleDetectionEnum.OBJECT_DETECTED_SLOWDOWN
        elif status_number == 3.0:
            grpc_obstacle_detection_status = LidarObstacleDetectionEnum.OBJECT_DETECTED_STOP
        else:
            grpc_obstacle_detection_status = LidarObstacleDetectionEnum.DETECTION_ERROR
            self.logger.warning(f"Unknown LIDAR safety status: {status_number}")

        res_status.safety_on.value = lidar_info["safety_on"]
        res_status.safety_distance.value = lidar_info["safety_distance"]
        res_status.critical_distance.value = lidar_info["critical_distance"]
        res_status.obstacle_detection_status.status = grpc_obstacle_detection_status

        res_zuuu_mode = ZuuuModeCommand(id=request, mode=getattr(ZuuuModePossiblities, self.bridge_node.get_zuuu_mode()))
        res_control_mode = ControlModeCommand(
            id=request, mode=getattr(ControlModePossiblities, self.bridge_node.get_control_mode())
        )

        res_bat = BatteryLevel(level=FloatValue(value=self.bridge_node.get_battery_voltage()))

        req = MobileBaseState(
            timestamp=get_current_timestamp(self.bridge_node),
            id=request,
            activated=True,
            battery_level=res_bat,
            lidar_safety=res_status,
            zuuu_mode=res_zuuu_mode,
            control_mode=res_control_mode,
        )
        return req

    def SendDirection(self, request: TargetDirectionCommand, context) -> MobilityServiceAck:
        """Send a speed command for the mobile base expressed in SI units."""
        self._last_direction = request
        # self.logger.info(f"Sending direction: {request.direction}")

        twist = Twist()
        twist.linear.x = request.direction.x.value
        twist.linear.y = request.direction.y.value
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = request.direction.theta.value
        self.cmd_vel_pub.publish(twist)

        return MobilityServiceAck(success=BoolValue(value=True))

    def GetLastDirection(self, request: PartId, context) -> DirectionVector:
        """Get the last direction sent to the mobile base."""
        return DirectionVector(
            x=self._last_direction.direction.x,
            y=self._last_direction.direction.y,
            theta=self._last_direction.direction.theta,
        )

    def SendSetSpeed(self, request: SetSpeedVector, context) -> MobilityServiceAck:
        """Send a speed command for the mobile base expressed in SI units for a given duration."""
        req = SetSpeed.Request()
        req.duration = request.duration.value
        req.x_vel = request.x_vel.value
        req.y_vel = request.y_vel.value
        req.rot_vel = request.rot_vel.value

        self.set_speed_client.call_async(req)
        return MobilityServiceAck(success=BoolValue(value=True))

    def DistanceToGoal(self, request: PartId, context):
        """Return the distance left to reach the last goto target sent.

        The remaining x, y and theta to get to the target are also returned.
        """
        response = DistanceToGoalVector(
            delta_x=FloatValue(value=0.0),
            delta_y=FloatValue(value=0.0),
            delta_theta=FloatValue(value=0.0),
            distance=FloatValue(value=0.0),
        )

        req = DistanceToGoal.Request()

        result = self.distance_to_goal_client.call(req)

        if result is not None:
            ros_response = result
            # print(ros_response)
            response.delta_x.value = ros_response.delta_x
            response.delta_y.value = ros_response.delta_y
            response.delta_theta.value = ros_response.delta_theta
            response.distance.value = ros_response.distance

        return response

    def SetControlMode(self, request: ControlModeCommand, context) -> MobilityServiceAck:
        """Set mobile base control mode.

        Two valid control modes are available: OPEN_LOOP and PID.
        """
        mode = ControlModePossiblities.keys()[request.mode]

        if mode == "NONE_CONTROL_MODE":
            return MobilityServiceAck(success=BoolValue(value=False))

        # TODO there is a better way to do this
        run(f"ros2 param set /zuuu_hal control_mode {mode}", stdout=PIPE, shell=True)
        return MobilityServiceAck(success=BoolValue(value=True))

    def GetControlMode(self, request: Empty, context) -> ControlModeCommand:
        """Get mobile base control mode."""
        output = check_output(["ros2", "param", "get", "/zuuu_hal", "control_mode"]).decode()

        # Response from ros2 looks like: "String value is: MODE"
        mode = output.split(": ")[-1].split()[0]

        mode_grpc = getattr(ControlModePossiblities, mode)
        return ControlModeCommand(id=PartId(name=self.mobile_bases[0].name, id=self.mobile_bases[0].id), mode=mode_grpc)

    def SetZuuuMode(self, request: ZuuuModeCommand, context) -> MobilityServiceAck:
        """Set mobile base drive mode.

        Six valid drive modes are available: CMD_VEL, BRAKE, FREE_WHEEL, SPEED, GOTO, EMERGENCY_STOP.
        """
        mode = ZuuuModePossiblities.keys()[request.mode]

        if mode == "NONE_ZUUU_MODE":
            return MobilityServiceAck(success=BoolValue(value=False))

        req = SetZuuuMode.Request()
        req.mode = mode
        self.set_zuuu_mode_client.call_async(req)
        return MobilityServiceAck(success=BoolValue(value=True))

    def GetZuuuMode(self, request: Empty, context) -> ZuuuModeCommand:
        """Get mobile base drive mode."""
        req = GetZuuuMode.Request()

        mode = ZuuuModePossiblities.NONE_ZUUU_MODE
        result = self.get_zuuu_mode_client.call(req)

        if result is not None:
            mode = result.mode
            mode = getattr(ZuuuModePossiblities, mode)

        return ZuuuModeCommand(id=PartId(name=self.mobile_bases[0].name, id=self.mobile_bases[0].id), mode=mode)

    def GetBatteryLevel(self, request: PartId, context) -> BatteryLevel:
        """Get mobile base battery level in Volt."""
        req = GetBatteryVoltage.Request()

        response = BatteryLevel(level=FloatValue(value=0.0))

        result = self.get_battery_voltage_client.call(req)

        if result is not None:
            ros_response = result
            response.level.value = ros_response.voltage

        return response

    def odom_cb(self, odom: Odometry):
        self._last_odom = odom

    def GetOdometry(self, request: PartId, context) -> OdometryVector:
        """Get mobile base odometry.

        x, y are in meters and theta is in radian.
        """
        response = OdometryVector(
            x=FloatValue(value=0.0),
            y=FloatValue(value=0.0),
            theta=FloatValue(value=0.0),
            vx=FloatValue(value=0.0),
            vy=FloatValue(value=0.0),
            vtheta=FloatValue(value=0.0),
        )

        if self._last_odom is not None:
            odom = self._last_odom
            response.x.value = odom.pose.pose.position.x
            response.y.value = odom.pose.pose.position.y
            theta = tf_transformations.euler_from_quaternion(
                [
                    odom.pose.pose.orientation.x,
                    odom.pose.pose.orientation.y,
                    odom.pose.pose.orientation.z,
                    odom.pose.pose.orientation.w,
                ]
            )
            response.theta.value = theta[2]

            response.vx.value = odom.twist.twist.linear.x
            response.vy.value = odom.twist.twist.linear.y
            response.vtheta.value = odom.twist.twist.angular.z
        return response

    def ResetOdometry(self, request: PartId, context) -> MobilityServiceAck:
        """Reset mobile base odometry.

        Current position of the mobile_base is taken as new origin of the odom frame.
        """
        req = ResetOdometry.Request()
        self.reset_odometry_client.call_async(req)
        return MobilityServiceAck(success=BoolValue(value=True))

    def GetZuuuSafety(self, request: PartId, context) -> LidarSafety:
        """Get the anti-collision safety status handled by the mobile base hal along with the safety and critical distances."""
        req = GetZuuuSafety.Request()

        result = self.get_zuuu_safety_client.call(req)

        response = LidarSafety(id=request)

        if result is not None:
            ros_response = result
            response.safety_on.value = ros_response.safety_on
            response.safety_distance.value = ros_response.safety_distance
            response.critical_distance.value = ros_response.critical_distance

            ros_obstacle_detection_status = ros_response.obstacle_detection_status
            if ros_obstacle_detection_status == "green":
                grpc_obstacle_detection_status = LidarObstacleDetectionEnum.NO_OBJECT_DETECTED
            elif ros_obstacle_detection_status == "orange":
                grpc_obstacle_detection_status = LidarObstacleDetectionEnum.OBJECT_DETECTED_SLOWDOWN
            elif ros_obstacle_detection_status == "red":
                grpc_obstacle_detection_status = LidarObstacleDetectionEnum.OBJECT_DETECTED_STOP

            response.obstacle_detection_status.status = grpc_obstacle_detection_status

        return response

    def SetZuuuSafety(self, request: LidarSafety, context) -> MobilityServiceAck:
        """Set on/off the anti-collision safety handled by the mobile base hal."""
        req = SetZuuuSafety.Request()
        if request.HasField("safety_on"):
            req.safety_on = request.safety_on.value
        else:
            self.logger.error("Safety_on field is missing in the SetZuuuSafety request.")
            return MobilityServiceAck(success=BoolValue(value=False))

        if request.HasField("safety_distance"):
            req.safety_distance = request.safety_distance.value
        if request.HasField("critical_distance"):
            req.critical_distance = request.critical_distance.value
        self.set_zuuu_safety_client.call_async(req)
        return MobilityServiceAck(success=BoolValue(value=True))

    def GetLidarMap(self, request: PartId, context) -> LidarMap:
        """Get the lidar map."""
        if self.core_mode == ReachyCoreMode.FAKE:
            self.logger.warning("Lidar map not available in FAKE mode.")
            return LidarMap()
        img = PilImage.fromarray(self.lidar_img)

        buf = io.BytesIO()
        img.save(buf, format="JPEG")  # Format can be changed as needed
        uncompressed_bytes = buf.getvalue()
        return LidarMap(data=uncompressed_bytes)

    def Audit(self, request: PartId, context: grpc.ServicerContext) -> MobileBaseStatus:
        return MobileBaseStatus()

    def HeartBeat(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        return Empty()

    def Restart(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        return Empty()

    def ResetDefaultSafetyDistances(self, request: PartId, context: grpc.ServicerContext) -> MobilityServiceAck:
        safety_on = self.GetZuuuSafety(request, context).safety_on
        ack = self.SetZuuuSafety(
            LidarSafety(safety_on=safety_on, critical_distance=FloatValue(value=0.55), safety_distance=FloatValue(value=0.7)),
            context,
        )
        return ack

    def TurnOn(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        zuuu_mode = ZuuuModeCommand(id=request, mode=ZuuuModePossiblities.CMD_GOTO)
        self.SetZuuuMode(zuuu_mode, context)
        return Empty()

    def TurnOff(self, request: PartId, context: grpc.ServicerContext) -> Empty:
        zuuu_mode = ZuuuModeCommand(id=request, mode=ZuuuModePossiblities.FREE_WHEEL)
        self.SetZuuuMode(zuuu_mode, context)
        return Empty()

    def GetLidarObstacleDetectionStatus(self, request: PartId, context: grpc.ServicerContext) -> LidarObstacleDetectionStatus:
        return LidarObstacleDetectionStatus()
