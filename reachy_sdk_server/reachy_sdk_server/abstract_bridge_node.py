from asyncio.events import AbstractEventLoop
from collections import deque
from functools import partial
from threading import Event, Lock
from typing import Any, Dict, List, Tuple

import numpy as np
import prometheus_client as pc
import rclpy
import reachy2_monitoring as rm
from control_msgs.msg import DynamicJointState, InterfaceValue
from geometry_msgs.msg import Pose, PoseStamped
from pollen_msgs.action import Goto
from zuuu_interfaces.action import ZuuuGoto
from pollen_msgs.msg import CartTarget, IKRequest, MobileBaseState, ReachabilityState
from pollen_msgs.srv import GetForwardKinematics, GetInverseKinematics
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from reachy2_sdk_api.component_pb2 import ComponentId
from reachy2_sdk_api.part_pb2 import PartId
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float32MultiArray

from .components import ComponentsHolder
from .conversion import matrix_to_pose, pose_to_matrix
from .parts import PartsHolder
from .utils import parse_reachy_config


class AbstractBridgeNode(Node):
    def __init__(self, reachy_config_path: str = None, asyncio_loop: AbstractEventLoop = None, port=0) -> None:
        super().__init__(node_name="reachy_abstract_bridge_node")

        self.logger = self.get_logger()

        NODE_NAME = f"grpc-server_SDK{'.' + str(port) if port != 0 else ''}"
        rm.configure_pyroscope(
            NODE_NAME,
            tags={
                "server": "false",
                "client": "true",
            },
        )
        self.tracer = rm.tracer(NODE_NAME, grpc_type="server")

        metrics_port = 10000 + int(port)
        self.logger.info(f"Start port:{port}, metrics_port:{metrics_port} (port+10000).")

        self.asyncio_loop = asyncio_loop
        self.config = parse_reachy_config(reachy_config_path)
        self.components = ComponentsHolder(self.config)

        self.got_first_state = Event()
        self.joint_state_ready = Event()
        self.got_first_mb_state_status = Event()
        self.reachability_deque = {}

        self.create_subscription(
            msg_type=DynamicJointState,
            topic="/dynamic_joint_states",
            callback=self.update_state,
            qos_profile=10,
        )

        self.mobile_base_enabled = True

        config = parse_reachy_config(reachy_config_path)
        self.info = {
            "serial_number": config["mobile_base"]["serial_number"],
            "version_hard": config["mobile_base"]["version_hard"],
            "version_soft": config["mobile_base"]["version_soft"],
        }

        if not config["mobile_base"]["serial_number"]:
            self.logger.info("No mobile base found in the config file. Mobile base server not initialized.")
            self.mobile_base_enabled = False

        # TODO create publisher
        # self.create_subscription(
        #     msg_type=ReachabilityState,
        #     topic="/ReachabilityState",
        #     callback=self.update_reachability_state,
        #     qos_profile=10,
        # )

        for arm in ["r_arm", "l_arm"]:
            self.create_subscription(
                msg_type=ReachabilityState,
                topic=f"/{arm}_reachability_states",
                qos_profile=10,
                callback=partial(
                    self.update_reachability_state,
                    name=arm,
                ),
            )
            self.reachability_deque[arm] = deque(maxlen=100)

        self.command_pub_lock = Lock()
        self.joint_command_pub = self.create_publisher(
            msg_type=DynamicJointState,
            topic="/dynamic_joint_commands",
            qos_profile=10,
        )

        self.wait_for_setup()
        self.logger.info(f"Joint state setup (nb_joints={len(self.components.components)}).")
        for c in self.components.components:
            self.logger.info(f"\t - {c}")

        # Now that we have components, setup parts
        self.parts = PartsHolder(self.logger, self.config, self.components)

        # Register to services for kinematics
        self.setup_kinematics()

        self.create_subscription(
            msg_type=JointState,
            topic="/joint_commands",
            callback=self.update_command,
            qos_profile=10,
        )

        # create a subscriber to the safety status topic
        # defined in the zuu_hal.py file
        self.create_subscription(
            msg_type=MobileBaseState,
            topic="/mobile_base_state",
            callback=self.update_mobile_base_state,
            qos_profile=10,
        )
        # dictionary that contains the mirror of the LidarSafety class from the zuu_hal.py file
        # and in the GetZuuuSafety service
        # "safety_on":          safety on/off flag
        # "safety_distance":      obstacle safety distance
        # "critical_distance":  obstacle critical distance
        # "status":             safety status [0: detection error, 1: no obstacle, 2: obstacle detected slowing down, 3: obstacle detected stopping]
        self.lidar_safety = {"safety_on": False, "safety_distance": 0.0, "critical_distance": 0.0, "status": 0}
        self.battery_voltage = 0.0
        self.zuuu_mode = "NONE_ZUUU_MODE"
        self.control_mode = "NONE_CONTROL_MODE"

        # Setup goto action clients
        self.prefixes = ["r_arm", "l_arm", "neck"]
        self.goto_action_client = {}
        for prefix in self.prefixes:
            self.goto_action_client[prefix] = ActionClient(self, Goto, f"{prefix}_goto")
            self.get_logger().info(f"Waiting for action server {prefix}_goto...")
            self.goto_action_client[prefix].wait_for_server()

        if self.mobile_base_enabled:
            self.goto_zuuu_action_client = ActionClient(self, ZuuuGoto, "mobile_base_goto")
            self.get_logger().info(f"Waiting for action server mobile_base_goto...")
            self.goto_zuuu_action_client.wait_for_server()

        # Start up the server to expose the metrics.
        pc.start_http_server(metrics_port)
        self.sum_getreachystate = pc.Summary("sdkserver_GetReachyState_time", "Time spent during bridge reachy.GetReachyState")
        self.sum_spin = pc.Summary("sdkserver_spin_once_time", "Time spent during bridge spin_once")
        self.sum_spin_sanity = pc.Summary("sdkserver_time_reference_1s", "Sanity check spin, sleeps 1s")
        self.get_logger().info(f"Setup complete.")

    def wait_for_setup(self) -> None:
        # Wait for a first /dynamic_joint_state message to get a list of all joints
        while not self.got_first_state.is_set():
            rclpy.spin_once(self)

        # call service to get all value for each joint
        for name in self.joint_names:
            self.components.add_component(name, node_delegate=self)

        self.joint_state_ready.set()

    # State updates
    def update_state(self, msg: DynamicJointState) -> None:
        if not self.got_first_state.is_set():
            self.joint_names = msg.joint_names
            self.got_first_state.set()
            return

        if not self.joint_state_ready.is_set():
            return

        for name, kv in zip(msg.joint_names, msg.interface_values):
            state = dict(zip(kv.interface_names, kv.values))
            self.components.get_by_name(name).update_state(state)

    def update_reachability_state(self, msg: ReachabilityState, name: str) -> None:
        order_id = msg.order_id
        state = msg.state
        is_reachable = msg.is_reachable
        self.reachability_deque[name].append((order_id, is_reachable, state))

    # Command updates
    def update_command(self, msg: JointState) -> None:
        for name, target in zip(msg.name, msg.position):
            self.components.get_by_name(name).update_command({"target_position": target})

    # getter for the battery voltage
    # returns the battery voltage value [V]
    # returns 0 if no battery voltage has been received yet
    def get_battery_voltage(self) -> float:
        if not self.got_first_mb_state_status.is_set():
            self.logger.error("No battery voltage received yet.")
        return self.battery_voltage

    # getter for zuuu mode
    # returns the string of the mode
    # returns 'NONE_ZUUU_MODE' if no info has been received yet
    def get_zuuu_mode(self) -> str:
        if not self.got_first_mb_state_status.is_set():
            self.logger.error("No zuuu mode received yet.")
        return self.zuuu_mode

    # getter for mobile base control mode
    # returns the string of the mode
    # returns 'NONE_CONTROL_MODE' if no info has been received yet
    def get_control_mode(self) -> str:
        if not self.got_first_mb_state_status.is_set():
            self.logger.error("No controle mode received yet.")
        return self.control_mode

    # function which is run when the safety status message is received
    def update_mobile_base_state(self, msg: MobileBaseState) -> None:
        if not self.got_first_mb_state_status.is_set():
            self.got_first_mb_state_status.set()
        # save the safety status value to the class variable
        # that contains
        # 0: safety on/off flag
        # 1: obstacle safety distance
        # 2: obstacle critical distance
        # 3: safety status [0: detection error, 1: no obstacle, 2: obstacle detected slowing down, 3: obstacle detected stopping]
        self.battery_voltage = msg.battery_voltage.data
        self.lidar_safety["safety_on"] = msg.safety_on.data
        self.lidar_safety["safety_distance"] = msg.mobile_base_safety_status.data[0]
        self.lidar_safety["critical_distance"] = msg.mobile_base_safety_status.data[1]
        self.lidar_safety["status"] = int(msg.mobile_base_safety_status.data[2])

        self.zuuu_mode = msg.zuuu_mode
        self.control_mode = msg.control_mode

    # getter for the safety status
    # returns the safety status
    # mirroring the `LidarObstacleDetectionEnum` from `mobile_base_lidar.proto`
    # [0: detection error, 1: no obstacle, 2: obstacle detected slowing down, 3: obstacle detected stopping]
    def get_safety_status(self) -> Dict[str, Any]:
        if not self.got_first_mb_state_status.is_set():
            self.logger.error("No safety status received yet.")
            return {"safety_on": False, "safety_distance": 0.0, "critical_distance": 0.0, "status": 0}
        return self.lidar_safety

    def publish_command(self, msg: DynamicJointState) -> None:
        self.joint_command_pub.publish(msg)

    # Misc utils
    def get_component(self, component_id: ComponentId) -> dict:
        return self.components.get_by_component_id(component_id)

    # Kinematics
    def setup_kinematics(self):
        # Register to forward/inverse kinematics ROS services
        # And to /{side}_arm/target_pose
        self.forward_kinematics_clients = {}
        self.inverse_kinematics_clients = {}
        # self.target_pose_pubs = {}
        self.head_target_pose_pubs = {}
        self.arm_target_pose_pubs = {}

        self.command_target_pub_lock = Lock()

        for part in self.parts:
            if part.type not in ("arm", "head"):
                continue

            c = self.create_client(
                srv_type=GetForwardKinematics,
                srv_name=f"/{part.name}/forward_kinematics",
            )
            self.logger.info(f"Subscribing for service '{c.srv_name}'...")
            c.wait_for_service()
            self.forward_kinematics_clients[part.id] = c

            c = self.create_client(
                srv_type=GetInverseKinematics,
                srv_name=f"/{part.name}/inverse_kinematics",
            )
            self.logger.info(f"Subscribing for service '{c.srv_name}'...")
            c.wait_for_service()
            self.inverse_kinematics_clients[part.id] = c

            # High frequency QoS profile
            high_freq_qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,  # Prioritizes speed over guaranteed delivery
                history=HistoryPolicy.KEEP_LAST,  # Keeps only a fixed number of messages
                depth=1,  # Minimal depth, for the latest message
                # Other QoS settings can be adjusted as needed
            )

            self.head_target_pose_pubs[part.id] = self.create_publisher(
                msg_type=CartTarget,
                topic=f"/{part.name}/cart_target_pose",
                qos_profile=high_freq_qos_profile,
            )

            self.arm_target_pose_pubs[part.id] = self.create_publisher(
                msg_type=IKRequest,
                topic=f"/{part.name}/ik_target_pose",
                qos_profile=high_freq_qos_profile,
            )

            self.logger.info(f"Publisher to topic '{self.head_target_pose_pubs[part.id].topic_name}' ready.")
            self.logger.info(f"Publisher to topic '{self.arm_target_pose_pubs[part.id].topic_name}' ready.")

    def compute_forward(self, id: PartId, joint_position: JointState) -> Tuple[bool, np.array]:
        id = self.parts.get_by_part_id(id).id

        req = GetForwardKinematics.Request()
        req.joint_position = joint_position

        resp = self.forward_kinematics_clients[id].call(req)

        if resp.success:
            return True, pose_to_matrix(resp.pose)
        else:
            return False, None

    def compute_inverse(self, id: PartId, target: np.array, q0: JointState) -> Tuple[bool, JointState]:
        id = self.parts.get_by_part_id(id).id

        req = GetInverseKinematics.Request()
        req.pose = matrix_to_pose(target)
        req.q0 = q0

        resp = self.inverse_kinematics_clients[id].call(req)
        return resp.success, resp.joint_position

    def publish_head_target_pose(self, id: PartId, msg: CartTarget) -> None:
        id = self.parts.get_by_part_id(id).id
        self.head_target_pose_pubs[id].publish(msg)

    def publish_arm_target_pose(self, id: PartId, msg: IKRequest) -> None:
        id = self.parts.get_by_part_id(id).id
        self.arm_target_pose_pubs[id].publish(msg)

    async def send_goto_goal(
        self,
        part: str,
        joint_names: List[str],
        goal_positions: List[float],
        duration: float,
        goal_velocities: List[float] = [],
        mode: str = "minimum_jerk",  # "linear" or "minimum_jerk"
        sampling_freq: float = 150.0,
        feedback_callback=None,
        return_handle=False,
    ):
        goal_msg = Goto.Goal()
        request = goal_msg.request  # This is of type pollen_msgs/GotoRequest

        request.duration = duration
        request.mode = mode
        request.sampling_freq = sampling_freq
        request.safety_on = False

        request.goal_joints = JointState()
        request.goal_joints.name = joint_names
        request.goal_joints.position = goal_positions
        request.goal_joints.velocity = goal_velocities
        request.goal_joints.effort = []  # Not implemented for now

        self.get_logger().debug("Sending goal request...")

        goal_handle = await self.goto_action_client[part].send_goal_async(goal_msg, feedback_callback=feedback_callback)
        self.get_logger().debug("feedback_callback setuped")

        if not goal_handle.accepted:
            self.get_logger().warning("Goal rejected!")
            return None

        self.get_logger().debug("Goal accepted")

        if return_handle:
            return goal_handle
        else:
            res = await goal_handle.get_result_async()
            result = res.result
            status = res.status
            self.get_logger().debug(f"Goto finished. Result: {result.result.status}")
            return result, status

    async def send_zuuu_goto_goal(
        self,
        x_goal,
        y_goal,
        theta_goal,
        dist_tol=0.05,
        angle_tol=np.deg2rad(5),
        timeout=10.0,
        keep_control_on_arrival=True,
        distance_p=5.0,
        distance_i=0.0,
        distance_d=0.0,
        distance_max_command=0.4,
        angle_p=5.0,
        angle_i=0.0,
        angle_d=0.0,
        angle_max_command=1.0,
        feedback_callback=None,
        return_handle=False,
    ):
        """Send a goal to the zuuu_goto action server.
        x_goal (m), y_goal (m), theta_goal (rads) ->  goal pose in the odom frame
        dist_tol (m), angle_tol (rads) -> distance and angle tolerance for the goal
        timeout (s) -> timeout for the goal, the goto action will end if the goal is not reached in time
        keep_control_on_arrival (bool) -> if True, the goto control loop will keep running even if the goal is reached
        distance_p, distance_i, distance_d -> PID gains for the xy distance control loop
        distance_max_command (m/s - ish) -> limits the maximum command for the xy distance PID controller
        angle_p, angle_i, angle_d -> PID gains for the angle control loop
        angle_max_command (rads/s -ish) -> limits the maximum command for the angle PID controller
        feedback_callback -> callback function to be called when feedback is received
        return_handle (bool) -> if True, the function will return the goal handle, else it will wait for the goal to finish and return the result and status
        """
        goal_msg = ZuuuGoto.Goal()

        request = goal_msg.request  # This is of type zuuu_interfaces/ZuuuGotoRequest

        request.x_goal = x_goal
        request.y_goal = y_goal
        request.theta_goal = theta_goal
        request.dist_tol = dist_tol
        request.angle_tol = angle_tol
        request.timeout = timeout
        request.keep_control_on_arrival = keep_control_on_arrival
        request.distance_p = distance_p
        request.distance_i = distance_i
        request.distance_d = distance_d
        request.distance_max_command = distance_max_command
        request.angle_p = angle_p
        request.angle_i = angle_i
        request.angle_d = angle_d
        request.angle_max_command = angle_max_command

        self.get_logger().warning(f"Sending zuuu goto goal request: {request}")

        goal_handle = await self.goto_zuuu_action_client.send_goal_async(goal_msg, feedback_callback=feedback_callback)

        self.get_logger().warning("zuuu goto feedback_callback setuped")

        if not goal_handle.accepted:
            self.get_logger().error("zuuu goto Goal rejected!")
            return None

        self.get_logger().warning("zuuu goto goal accepted")

        if return_handle:
            return goal_handle
        else:
            res = await goal_handle.get_result_async()
            result = res.result
            status = res.status
            self.get_logger().warning(f"zuuu goto goto finished. Result: {result.result.status}")
            return result, status

    def set_all_joints_to_current_position(self, part_name: str = "") -> None:
        """Set all joints to their current position if part_name is an empty string,
        else only the joints of the given part_name (e.g. r_arm) are set."""
        if not self.got_first_state.is_set():
            self.logger.error("No joint state received yet, cannot set all joints to current position.")
            return
        self.logger.debug(f"Setting all joints to their current position for part '{part_name}'.")

        joint_prefix = part_name.split("_")[0]
        if joint_prefix == "head":
            joint_prefix = "neck"

        cmd = DynamicJointState()
        cmd.joint_names = []

        for name in self.joint_names:
            if name.startswith(joint_prefix):
                # Note: any string starts with the empty string
                component = self.components.get_by_name(name)
                if "position" in component.state and "target_position" in component.state:
                    # These are all the "ROS joints" such as "r_arm_shoulder_pitch"
                    self.logger.debug(
                        f"\t Setting -{name} goal_position to its current position: {component.state['position']}"
                    )
                    component.state["target_position"] = component.state["position"]
                    cmd.joint_names.append(name)
                    cmd.interface_values.append(
                        InterfaceValue(
                            interface_names=["position"],
                            values=[component.state["position"]],
                        )
                    )

        self.publish_command(cmd)
