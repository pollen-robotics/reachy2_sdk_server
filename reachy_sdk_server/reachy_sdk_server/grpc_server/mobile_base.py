"""Expose main mobile base ROS services/topics through gRPC allowing remote client SDK."""

import time
from subprocess import run, PIPE, check_output
from concurrent.futures import ThreadPoolExecutor
from queue import Empty

from PIL import Image as PilImage
import io
import zlib

import grpc
from google.protobuf.empty_pb2 import Empty
from google.protobuf.wrappers_pb2 import BoolValue, FloatValue

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

from mobile_base_sdk_api import lidar_pb2, lidar_pb2_grpc
from mobile_base_sdk_api import mobility_pb2, mobility_pb2_grpc
from mobile_base_sdk_api import utility_pb2, utility_pb2_grpc

from zuuu_interfaces.srv import SetZuuuMode, GetZuuuMode, GetOdometry, ResetOdometry
from zuuu_interfaces.srv import GoToXYTheta, DistanceToGoal, GetZuuuSafety, SetZuuuSafety
from zuuu_interfaces.srv import SetSpeed, GetBatteryVoltage

from .utils import parse_reachy_config


class MobileBaseServicer(
    Node,
    lidar_pb2_grpc.MobileBaseLidarServiceServicer,
    mobility_pb2_grpc.MobileBaseMobilityServiceServicer,
    utility_pb2_grpc.MobileBaseUtilityServiceServicer,
):
    """Mobile base SDK server node."""

    def __init__(
            self,
            logger: rclpy.impl.rcutils_logger.RcutilsLogger,
            reachy_config_path: str) -> None:
        """Set up the node.

        Get mobile base basic info such as its odometry, battery level, drive mode or control mode
        from the mobile base hal.
        Send commands through the GoToXYTheta or SetSpeed services or by publishing to cmd_vel topic.
        """
        self.logger = logger

        config = parse_reachy_config(reachy_config_path)
        self.info = {
            "serial_number": config["mobile_base"]["serial_number"],
            "version_hard": config["mobile_base"]["version_hard"],
            "version_soft": config["mobile_base"]["version_soft"],
        }

        if config["mobile_base"]["serial_number"] is None:
            self.logger.info("No mobile base found in the config file. Mobile base server not initialized.")
            return


        super().__init__(node_name="mobile_base_server")

        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.bridge = CvBridge()
        self.lidar_img_subscriber = self.create_subscription(Image, 'lidar_image', self.get_lidar_img, 1)

        self.set_speed_client = self.create_client(SetSpeed, "SetSpeed")
        while not self.set_speed_client.wait_for_service(timeout_sec=1.0):
            self.logger.info("service SetSpeed not available, waiting again...")

        self.go_to_client = self.create_client(GoToXYTheta, "GoToXYTheta")
        while not self.go_to_client.wait_for_service(timeout_sec=1.0):
            self.logger.info("service GoToXYTheta not available, waiting again...")

        self.distance_to_goal_client = self.create_client(DistanceToGoal, "DistanceToGoal")
        while not self.distance_to_goal_client.wait_for_service(timeout_sec=1.0):
            self.logger.info("service DistanceToGoal not available, waiting again...")

        self.set_zuuu_mode_client = self.create_client(SetZuuuMode, "SetZuuuMode")
        while not self.set_zuuu_mode_client.wait_for_service(timeout_sec=1.0):
            self.logger.info("service SetZuuuMode not available, waiting again...")

        self.get_zuuu_mode_client = self.create_client(GetZuuuMode, "GetZuuuMode")
        while not self.get_zuuu_mode_client.wait_for_service(timeout_sec=1.0):
            self.logger.info("service GetZuuuMode not available, waiting again...")

        self.get_battery_voltage_client = self.create_client(GetBatteryVoltage, "GetBatteryVoltage")
        while not self.get_battery_voltage_client.wait_for_service(timeout_sec=1.0):
            self.logger.info("service GetBatteryVoltage not available, waiting again...")

        self.get_odometry_client = self.create_client(GetOdometry, "GetOdometry")
        while not self.get_odometry_client.wait_for_service(timeout_sec=1.0):
            self.logger.info("service GetOdometry not available, waiting again...")

        self.reset_odometry_client = self.create_client(ResetOdometry, "ResetOdometry")
        while not self.reset_odometry_client.wait_for_service(timeout_sec=1.0):
            self.logger.info("service ResetOdometry not available, waiting again...")

        self.set_zuuu_safety_client = self.create_client(SetZuuuSafety, 'SetZuuuSafety')
        while not self.set_zuuu_safety_client.wait_for_service(timeout_sec=1.0):
            self.logger.info('service SetZuuuSafety not available, waiting again...')

        self.get_zuuu_safety_client = self.create_client(GetZuuuSafety, 'GetZuuuSafety')
        while not self.get_zuuu_safety_client.wait_for_service(timeout_sec=1.0):
            self.logger.info('service GetZuuuSafety not available, waiting again...')
        self.logger.info("Initialized mobile base server.")

    def register_to_server(self, server: grpc.Server) -> None:
        """Register the servicer to the server."""
        self.logger.info("Registering 'MobileBaseServicer' to server.")
        lidar_pb2_grpc.add_MobileBaseLidarServiceServicer_to_server(self, server)
        mobility_pb2_grpc.add_MobileBaseMobilityServiceServicer_to_server(self, server)
        utility_pb2_grpc.add_MobileBaseUtilityServiceServicer_to_server(self, server)

    def get_mobile_base(self):
        """Get mobile base basic info."""
        return utility_pb2.MobileBase(
            info=utility_pb2.MobileBaseInfo(
                serial_number=self.info["serial_number"],
                version_hard=self.info["version_hard"],
                version_soft=self.info["version_soft"],
            )
        )

    def get_lidar_img(self, msg):
        self.lidar_img = self.bridge.imgmsg_to_cv2(msg)

    def GetMobileBase(
            self, request: Empty, context
    ) -> utility_pb2.MobileBaseInfo:
        """Get mobile base basic info."""
        return self.get_mobile_base()

    def GetState(
        self, request: Empty, context
    ) -> utility_pb2.MobileBaseState:
        """Get mobile base state."""
        if self.info["serial_number"] is None:
            return utility_pb2.MobileBaseState()

        req = utility_pb2.MobileBaseState(
            battery_level=self.GetBatteryLevel(request, context),
            lidar_safety_state=
        )
        return req

    def SendDirection(
        self, request: mobility_pb2.TargetDirectionCommand, context
    ) -> utility_pb2.MobilityServiceAck:
        """Send a speed command for the mobile base expressed in SI units."""
        twist = Twist()
        twist.linear.x = request.direction.x.value
        twist.linear.y = request.direction.y.value
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = request.direction.theta.value
        self.cmd_vel_pub.publish(twist)

        return utility_pb2.MobilityServiceAck(success=BoolValue(value=True))

    def SendSetSpeed(
        self, request: mobility_pb2.SetSpeedVector, context
    ) -> utility_pb2.MobilityServiceAck:
        """Send a speed command for the mobile base expressed in SI units for a given duration."""
        req = SetSpeed.Request()
        req.duration = request.duration.value
        req.x_vel = request.x_vel.value
        req.y_vel = request.y_vel.value
        req.rot_vel = request.rot_vel.value

        self.set_speed_client.call_async(req)
        return utility_pb2.MobilityServiceAck(success=BoolValue(value=True))

    def SendGoTo(
        self, request: mobility_pb2.GoToVector, context
    ) -> utility_pb2.MobilityServiceAck:
        """Send a target to the mobile base in the odom frame.

        The origin of the frame is initialised when the hal is started or whenever the odometry
        is reset. x and y are in meters and theta in radian.
        """
        req = GoToXYTheta.Request()
        req.x_goal = request.x_goal.value
        req.y_goal = request.y_goal.value
        req.theta_goal = request.theta_goal.value

        self.go_to_client.call_async(req)
        return utility_pb2.MobilityServiceAck(success=BoolValue(value=True))

    def DistanceToGoal(self, request, context):
        """Return the distance left to reach the last goto target sent.

        The remaining x, y and theta to get to the target are also returned.
        """
        response = mobility_pb2.DistanceToGoalVector(
            delta_x=FloatValue(value=0.0),
            delta_y=FloatValue(value=0.0),
            delta_theta=FloatValue(value=0.0),
            distance=FloatValue(value=0.0),
        )

        req = DistanceToGoal.Request()

        future = self.distance_to_goal_client.call_async(req)
        for _ in range(1000):
            if future.done():
                ros_response = future.result()
                print(ros_response)
                response.delta_x.value = ros_response.delta_x
                response.delta_y.value = ros_response.delta_y
                response.delta_theta.value = ros_response.delta_theta
                response.distance.value = ros_response.distance
                break
            time.sleep(0.001)
        return response

    def SetControlMode(
        self, request: utility_pb2.ControlModeCommand, context
    ) -> utility_pb2.MobilityServiceAck:
        """Set mobile base control mode.

        Two valid control modes are available: OPEN_LOOP and PID.
        """
        mode = utility_pb2.ControlModePossiblities.keys()[request.mode]

        if mode == "NONE_CONTROL_MODE":
            return utility_pb2.MobilityServiceAck(success=BoolValue(value=False))

        run(f"ros2 param set /zuuu_hal control_mode {mode}", stdout=PIPE, shell=True)
        return utility_pb2.MobilityServiceAck(success=BoolValue(value=True))

    def GetControlMode(self, request: Empty, context) -> utility_pb2.ControlModeCommand:
        """Get mobile base control mode."""
        output = check_output(["ros2", "param", "get", "/zuuu_hal", "control_mode"]).decode()

        # Response from ros2 looks like: "String value is: MODE"
        mode = output.split(": ")[-1].split()[0]

        mode_grpc = getattr(utility_pb2.ControlModePossiblities, mode)
        return utility_pb2.ControlModeCommand(mode=mode_grpc)

    def SetZuuuMode(
        self, request: utility_pb2.ZuuuModeCommand, context
    ) -> utility_pb2.MobilityServiceAck:
        """Set mobile base drive mode.

        Six valid drive modes are available: CMD_VEL, BRAKE, FREE_WHEEL, SPEED, GOTO, EMERGENCY_STOP.
        """
        mode = utility_pb2.ZuuuModePossiblities.keys()[request.mode]

        if mode == "NONE_ZUUU_MODE":
            return utility_pb2.MobilityServiceAck(success=BoolValue(value=False))

        req = SetZuuuMode.Request()
        req.mode = mode
        self.set_zuuu_mode_client.call_async(req)
        return utility_pb2.MobilityServiceAck(success=BoolValue(value=True))

    def GetZuuuMode(self, request: Empty, context) -> utility_pb2.ZuuuModeCommand:
        """Get mobile base drive mode."""
        req = GetZuuuMode.Request()

        future = self.get_zuuu_mode_client.call_async(req)
        for _ in range(1000):
            if future.done():
                mode = future.result().mode
                break
            time.sleep(0.001)
        if not future.done():
            mode = utility_pb2.ZuuuModePossiblities.NONE_ZUUU_MODE
            return utility_pb2.ZuuuModeCommand(mode=mode)

        mode_grpc = getattr(utility_pb2.ZuuuModePossiblities, mode)
        return utility_pb2.ZuuuModeCommand(mode=mode_grpc)

    def GetBatteryLevel(self, request: Empty, context) -> utility_pb2.BatteryLevel:
        """Get mobile base battery level in Volt."""
        req = GetBatteryVoltage.Request()

        response = utility_pb2.BatteryLevel(level=FloatValue(value=0.0))

        future = self.get_battery_voltage_client.call_async(req)
        for _ in range(1000):
            if future.done():
                ros_response = future.result()
                response.level.value = ros_response.voltage
                break
            time.sleep(0.001)
        return response

    def GetOdometry(self, request: Empty, context) -> utility_pb2.OdometryVector:
        """Get mobile base odometry.

        x, y are in meters and theta is in radian.
        """
        req = GetOdometry.Request()
        response = utility_pb2.OdometryVector(
            x=FloatValue(value=0.0),
            y=FloatValue(value=0.0),
            theta=FloatValue(value=0.0),
        )

        future = self.get_odometry_client.call_async(req)
        for _ in range(1000):
            if future.done():
                ros_response = future.result()
                response.x.value = ros_response.x
                response.y.value = ros_response.y
                response.theta.value = ros_response.theta
                break
            time.sleep(0.001)
        return response

    def ResetOdometry(self, request: Empty, context) -> utility_pb2.MobilityServiceAck:
        """Reset mobile base odometry.

        Current position of the mobile_base is taken as new origin of the odom frame.
        """
        req = ResetOdometry.Request()
        self.reset_odometry_client.call_async(req)
        return utility_pb2.MobilityServiceAck(success=BoolValue(value=True))

    def GetZuuuSafety(
                    self,
                    request: Empty,
                    context) -> lidar_pb2.LidarSafety:
        """Get the anti-collision safety status handled by the mobile base hal
        along with the safety and critical distances.
        """
        req = GetZuuuSafety.Request()

        future = self.get_zuuu_safety_client.call_async(req)
        for _ in range(1000):
            if future.done():
                ros_response = future.result()
                safety_on = ros_response.safety_on
                safety_distance = ros_response.safety_distance
                critical_distance = ros_response.critical_distance
                obstacle_detection_status = ros_response.obstacle_detection_status
                break
            time.sleep(0.001)
        if obstacle_detection_status == "green":
            obstacle_detection_status = lidar_pb2.LidarObstacleDetectionEnum.NO_OBJECT_DETECTED
        elif obstacle_detection_status == "orange":
            obstacle_detection_status = lidar_pb2.LidarObstacleDetectionEnum.OBJECT_DETECTED_SLOWDOWN
        elif obstacle_detection_status == "red":
            obstacle_detection_status = lidar_pb2.LidarObstacleDetectionEnum.OBJECT_DETECTED_STOP

        return lidar_pb2.LidarSafety(
            safety_on=BoolValue(value=safety_on),
            safety_distance=FloatValue(value=safety_distance),
            critical_distance=FloatValue(value=critical_distance),
            obstacle_detection_status=lidar_pb2.LidarObstacleDetectionStatus(status=obstacle_detection_status),
        )

    def SetZuuuSafety(
        self, request: lidar_pb2.LidarSafety, context
    ) -> utility_pb2.MobilityServiceAck:
        """Set on/off the anti-collision safety handled by the mobile base hal."""
        req = SetZuuuSafety.Request()
        req.safety_on = request.safety_on.value
        req.safety_distance = request.safety_distance.value
        req.critical_distance = request.critical_distance.value
        self.set_zuuu_safety_client.call_async(req)
        return utility_pb2.MobilityServiceAck(success=BoolValue(value=True))

    def GetLidarMap(
        self, request: Empty, context
    ) -> lidar_pb2.LidarMap:
        """Get the lidar map."""
        img = PilImage.fromarray(self.lidar_img)

        buf = io.BytesIO()
        img.save(buf, format="JPEG")  # Format can be changed as needed
        uncompressed_bytes = buf.getvalue()
        compressed_bytes = zlib.compress(uncompressed_bytes)
        return lidar_pb2.LidarMap(data=compressed_bytes)
