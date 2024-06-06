from asyncio.events import AbstractEventLoop
from threading import Event, Lock
from typing import List, Tuple


import prometheus_client as pc
import numpy as np
import rclpy
from control_msgs.msg import DynamicJointState, InterfaceValue
from geometry_msgs.msg import Pose, PoseStamped
from pollen_msgs.action import Goto
from pollen_msgs.srv import GetForwardKinematics, GetInverseKinematics
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from reachy2_sdk_api.component_pb2 import ComponentId
from reachy2_sdk_api.part_pb2 import PartId
from sensor_msgs.msg import JointState

from .components import ComponentsHolder
from .conversion import matrix_to_pose, pose_to_matrix
from .parts import PartsHolder
from .utils import parse_reachy_config


class AbstractBridgeNode(Node):
    def __init__(self, reachy_config_path: str = None, asyncio_loop: AbstractEventLoop = None) -> None:
        super().__init__(node_name="reachy_abstract_bridge_node")

        self.logger = self.get_logger()

        self.asyncio_loop = asyncio_loop
        self.config = parse_reachy_config(reachy_config_path)
        self.components = ComponentsHolder(self.config)

        self.got_first_state = Event()
        self.joint_state_ready = Event()

        self.create_subscription(
            msg_type=DynamicJointState,
            topic="/dynamic_joint_states",
            callback=self.update_state,
            qos_profile=10,
        )

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

        # Setup goto action clients
        self.prefixes = ["r_arm", "l_arm", "neck"]
        self.goto_action_client = {}
        for prefix in self.prefixes:
            self.goto_action_client[prefix] = ActionClient(self, Goto, f"{prefix}_goto")
            self.get_logger().info(f"Waiting for action server {prefix}_goto...")
            self.goto_action_client[prefix].wait_for_server()


        # Start up the server to expose the metrics.
        pc.start_http_server(10000)
        self.sum_getreachystate = pc.Summary('sdkserver_GetReachyState_time', 'Time spent during bridge reachy.GetReachyState')
        self.sum_spin = pc.Summary('sdkserver_spin_once_time', 'Time spent during bridge spin_once')
        self.sum_spin2 = pc.Summary('sdkserver_time_reference_1s', 'Time sleep 1s')
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

    # Command updates
    def update_command(self, msg: JointState) -> None:
        for name, target in zip(msg.name, msg.position):
            self.components.get_by_name(name).update_command({"target_position": target})

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
        self.target_pose_pubs = {}

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

            self.target_pose_pubs[part.id] = self.create_publisher(
                msg_type=PoseStamped,
                topic=f"/{part.name}/target_pose",
                qos_profile=high_freq_qos_profile,
            )
            self.logger.info(f"Publisher to topic '{self.target_pose_pubs[part.id].topic_name}' ready.")

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

    def publish_target_pose(self, id: PartId, pose: Pose) -> None:
        id = self.parts.get_by_part_id(id).id

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = pose

        self.target_pose_pubs[id].publish(msg)

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
