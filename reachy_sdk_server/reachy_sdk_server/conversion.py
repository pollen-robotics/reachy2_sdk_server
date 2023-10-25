import numpy as np
from geometry_msgs.msg import Pose
from reachy_sdk_api_v2.arm_pb2 import ArmPosition
from reachy_sdk_api_v2.head_pb2 import HeadPosition
from reachy_sdk_api_v2.kinematics_pb2 import (
    ExtEulerAngles,
    Point,
    Quaternion,
    Rotation3D,
)
from reachy_sdk_api_v2.orbita2d_pb2 import Pose2D
from google.protobuf.wrappers_pb2 import FloatValue
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from typing import Tuple

from .parts import Part


def rotation3d_as_extrinsinc_euler_angles(
    rot: Rotation3D,
) -> Tuple[float, float, float]:
    q = rotation3d_as_quat(rot)
    return Rotation.from_quat(q).as_euler("xyz")


def rotation3d_as_quat(
    rot: Rotation3D,
) -> Tuple[float, float, float, float]:
    if rot.HasField("q"):
        return rot.q.x, rot.q.y, rot.q.z, rot.q.w
    elif rot.HasField("rpy"):
        return Rotation.from_euler(
            "xyz", [rot.rpy.roll.value, rot.rpy.pitch.value, rot.rpy.yaw.value], degrees=False
        ).as_quat()
    elif rot.HasField("matrix"):
        return Rotation.from_matrix(np.array(rot.matrix.data).reshape((3, 3))).as_quat()
    else:
        raise ValueError("Unknown rotation type.")


def quat_as_rotation3d(
    q: Tuple[float, float, float, float],
) -> Rotation3D:
    return Rotation3D(
        q=Quaternion(
            x=q[0],
            y=q[1],
            z=q[2],
            w=q[3],
        ),
    )


def extrinsic_euler_angles_as_rotation3d(
    roll: float,
    pitch: float,
    yaw: float,
) -> Rotation3D:
    return Rotation3D(
        rpy=ExtEulerAngles(
            roll=FloatValue(value=roll),
            pitch=FloatValue(value=pitch),
            yaw=FloatValue(value=yaw),
        ),
    )


def pose_to_matrix(
    pose: Pose,
):
    M = np.eye(4)

    M[0, 3] = pose.position.x
    M[1, 3] = pose.position.y
    M[2, 3] = pose.position.z

    q = pose.orientation
    M[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

    return M


def matrix_to_pose(M) -> Pose:
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
        shoulder_pos.axis_1.value,
        shoulder_pos.axis_2.value,
        elbow_pos.axis_1.value,
        elbow_pos.axis_2.value,
        wrist_pos[0],
        wrist_pos[1],
        wrist_pos[2],
    ]

    return js


def head_position_to_joint_state(position: HeadPosition, head: Part) -> JointState:
    js = JointState()

    for c in head.components:
        js.name.extend(c.get_all_joints())

    neck_pos = rotation3d_as_extrinsinc_euler_angles(position.neck_position)

    js.position = [
        neck_pos[0],
        neck_pos[1],
        neck_pos[2],
        position.l_antenna_position,
        position.r_antenna_position,
    ]

    return js


def neck_rotation_to_joint_state(rot: Rotation3D, head: Part) -> JointState:
    js = JointState()

    for c in head.components:
        js.name.extend(c.get_all_joints())

    neck_pos = rotation3d_as_extrinsinc_euler_angles(rot)

    js.position = [
        neck_pos[0],
        neck_pos[1],
        neck_pos[2],
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


def joint_state_to_neck_orientation(js: JointState, head: Part) -> Rotation3D:
    head_name = []
    for c in head.components:
        head_name.extend(c.get_all_joints())
    assert js.name == head_name

    assert len(js.position) == len(head_name)

    return extrinsic_euler_angles_as_rotation3d(
        roll=js.position[0],
        pitch=js.position[1],
        yaw=js.position[2],
    )


def pose_from_pos_and_ori(pos: Point, ori: Rotation3D) -> Pose:
    p = Pose()

    p.position.x = pos.x
    p.position.y = pos.y
    p.position.z = pos.z

    q = rotation3d_as_quat(ori)
    p.orientation.x = q[0]
    p.orientation.y = q[1]
    p.orientation.z = q[2]
    p.orientation.w = q[3]

    return p


def extract_quaternion_from_pose(pose: Pose) -> Tuple[float, float, float, float]:
    return (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )


def extract_quaternion_from_pose_matrix(
    M: np.array,
) -> Tuple[float, float, float, float]:
    return Rotation.from_matrix(M[:3, :3]).as_quat()


def pose_matrix_from_quaternion(q: Quaternion) -> np.array:
    M = np.eye(4)
    M[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    return M


def pose_matrix_from_rotation3D(rot: Rotation3D) -> np.array:
    q = rotation3d_as_quat(rot)
    return pose_matrix_from_quaternion(Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))
