import numpy as np
from geometry_msgs.msg import Pose
from reachy_sdk_api_v2.kinematics_pb2 import Rotation3D
from scipy.spatial.transform import Rotation
from typing import Tuple


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
