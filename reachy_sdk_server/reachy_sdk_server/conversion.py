import numpy as np
from geometry_msgs.msg import Pose
from reachy_sdk_api_v2.arm_pb2 import ArmPosition
from reachy_sdk_api_v2.kinematics_pb2 import ExtEulerAngles, Rotation3D
from reachy_sdk_api_v2.orbita2d_pb2 import Pose2D
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from typing import Tuple

from .parts import Part


def rotation3d_as_extrinsinc_euler_angles(
    rot: Rotation3D,
) -> Tuple[float, float, float]:
    if rot.HasField("q"):
        return Rotation.from_quat([rot.q.x, rot.q.y, rot.q.z, rot.q.w]).as_euler(
            "xyz",
            degrees=False,
        )
    elif rot.HasField("rpy"):
        return rot.rpy.roll, rot.rpy.pitch, rot.rpy.yaw
    elif rot.HasField("matrix"):
        return Rotation.from_matrix(np.array(rot.matrix.data).reshape((3, 3))).as_euler(
            "xyz",
            degrees=False,
        )
    else:
        raise ValueError("Unknown rotation type.")


def extrinsic_euler_angles_as_rotation3d(
    roll: float,
    pitch: float,
    yaw: float,
) -> Rotation3D:
    return Rotation3D(
        rpy=ExtEulerAngles(
            roll=roll,
            pitch=pitch,
            yaw=yaw,
        ),
    )


def pose_to_matrix(
    pose: Pose,
) -> Tuple[
    float,
    float,
    float,
    float,
    float,
    float,
    float,
    float,
    float,
    float,
    float,
    float,
    float,
    float,
    float,
    float,
]:
    M = np.eye(4)

    M[0, 3] = pose.position.x
    M[1, 3] = pose.position.y
    M[2, 3] = pose.position.z

    q = pose.orientation
    M[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

    return M.flatten()


def matrix_to_pose(
    M: Tuple[
        float,
        float,
        float,
        float,
        float,
        float,
        float,
        float,
        float,
        float,
        float,
        float,
        float,
        float,
        float,
        float,
    ],
) -> Pose:
    pose = Pose()

    M = np.array(M).reshape((4, 4))

    pose.position.x = M[0, 3]
    pose.position.y = M[1, 3]
    pose.position.z = M[2, 3]

    q = Rotation.from_matrix(M[:3, :3]).as_quat()
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]

    return pose


def arm_position_to_joint_state(position: ArmPosition, arm: Part) -> JointState:
    js = JointState()

    for c in arm.components:
        js.name.extend(c.get_all_joints())

    shoulder_pos = position.shoulder_position
    elbow_pos = position.elbow_position
    wrist_pos = rotation3d_as_extrinsinc_euler_angles(position.wrist_position)

    js.position = [
        shoulder_pos.axis_1,
        shoulder_pos.axis_2,
        elbow_pos.axis_1,
        elbow_pos.axis_2,
        wrist_pos[0],
        wrist_pos[1],
        wrist_pos[2],
    ]

    return js


def joint_state_to_arm_position(js: JointState, arm: Part) -> ArmPosition:
    arm_name = []
    for c in arm.components:
        arm_name.extend(c.get_all_joints())
    assert js.name == arm_name

    assert len(js.position) == len(arm_name)

    return ArmPosition(
        shoulder_position=Pose2D(
            axis_1=js.position[0],
            axis_2=js.position[1],
        ),
        elbow_position=Pose2D(
            axis_1=js.position[2],
            axis_2=js.position[3],
        ),
        wrist_position=extrinsic_euler_angles_as_rotation3d(
            roll=js.position[4],
            pitch=js.position[5],
            yaw=js.position[6],
        ),
    )
