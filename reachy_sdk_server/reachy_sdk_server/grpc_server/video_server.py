import subprocess
import threading
from enum import Enum
from functools import partial
from typing import Dict, Optional

import grpc
import numpy as np
import rclpy
from google.protobuf.timestamp_pb2 import Timestamp
from reachy2_sdk_api.video_pb2 import CameraFeatures, CameraParameters, Frame, FrameRaw, ListOfCameraFeatures, View, ViewRequest
from reachy2_sdk_api.video_pb2_grpc import add_VideoServiceServicer_to_server
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg._compressed_image import CompressedImage
from sensor_msgs.msg._image import Image


class CameraType(Enum):
    TELEOP = "teleop_head"
    DEPTH = "depth_camera"


class ROSFrame:
    def __init__(self, timestamp, data):
        self.timestamp = timestamp
        self.data = data


class ROSFrameRaw(ROSFrame):
    def __init__(self, timestamp, data, height, width, encoding, step, isbigendian):
        super().__init__(timestamp, data)
        self.height = height
        self.width = width
        self.encoding = encoding
        self.step = step
        self.isbigendian = isbigendian


class ROSCamInfo:
    def __init__(self, height, width, distortion_model, D, K, R, P):
        self.height = height
        self.width = width
        self.distortion_model = distortion_model
        self.D = D
        self.K = K
        self.R = R
        self.P = P


class ReachyGRPCVideoSDKServicer:
    def __init__(self, gazebo: bool = False) -> None:
        rclpy.init()
        self.node = rclpy.create_node("ReachyGRPCVideoSDKServicer_node")
        self._logger = self.node.get_logger()
        self._gazebo_mode = gazebo

        if self._gazebo_mode:
            self._logger.info("Reachy GRPC Video SDK Servicer initialized (Gazebo mode).")
        else:
            self._logger.info("Reachy GRPC Video SDK Servicer initialized.")
        self._list_cam = []
        self._init_cameras()

        self.cams_frame: Dict[CameraType, Dict[View, Optional[ROSFrame]]] = {
            CameraType.TELEOP: {View.LEFT: None, View.RIGHT: None},
            CameraType.DEPTH: {View.LEFT: None, View.DEPTH: None},
        }
        self.cams_info: Dict[CameraType, Dict[View, Optional[ROSCamInfo]]] = {
            CameraType.TELEOP: {View.LEFT: None, View.RIGHT: None},
            CameraType.DEPTH: {View.LEFT: None, View.DEPTH: None},
        }

        self.ros_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.ros_thread.start()

    def __del__(self) -> None:
        self.node.destroy_node()
        rclpy.shutdown()

    def _init_cameras(self) -> None:
        self._list_cam.clear()
        if self._find_device("Luxonis") or self._gazebo_mode:
            self._list_cam.append(self._configure_teleop_camera())
        if self._find_device("Orbbec") or self._gazebo_mode:
            self._list_cam.append(self._configure_depth_camera())

    def _find_device(self, name: str) -> bool:
        devices = subprocess.check_output("lsusb").decode().split("\n")
        for device in devices:
            if name in device:
                return True
        return False

    def _configure_teleop_camera(self) -> CameraFeatures:
        ci = CameraFeatures(name=CameraType.TELEOP.value, stereo=True, depth=False)
        self._left_camera_sub = self.node.create_subscription(
            CompressedImage,
            "teleop_camera/left_image/image_raw/compressed",
            partial(self.on_image_update, cam_type=CameraType.TELEOP, side=View.LEFT),
            1,
        )

        self._right_camera_sub = self.node.create_subscription(
            CompressedImage,
            "teleop_camera/right_image/image_raw/compressed",
            partial(self.on_image_update, cam_type=CameraType.TELEOP, side=View.RIGHT),
            1,
        )

        self._left_camera_info_sub = self.node.create_subscription(
            CameraInfo,
            "teleop_camera/left_image/camera_info",
            partial(self.on_info_update, cam_type=CameraType.TELEOP, side=View.LEFT),
            1,
        )

        self._right_camera_info_sub = self.node.create_subscription(
            CameraInfo,
            "teleop_camera/right_image/camera_info",
            partial(self.on_info_update, cam_type=CameraType.TELEOP, side=View.RIGHT),
            1,
        )

        return ci

    def _configure_depth_camera(self) -> CameraFeatures:
        ci = CameraFeatures(name=CameraType.DEPTH.value, stereo=False, depth=True)
        self._depth_rgb_camera_sub = self.node.create_subscription(
            CompressedImage,
            "camera/color/image_raw/compressed",
            partial(self.on_image_update, cam_type=CameraType.DEPTH, side=View.LEFT),
            1,
        )

        self._depth_rgb_camera_info_sub = self.node.create_subscription(
            CameraInfo,
            "camera/color/camera_info",
            partial(self.on_info_update, cam_type=CameraType.DEPTH, side=View.LEFT),
            1,
        )

        self._depth_camera_sub = self.node.create_subscription(
            Image,
            "camera/depth/image_raw",
            partial(self.on_raw_image_update, cam_type=CameraType.DEPTH, side=View.DEPTH),
            1,
        )

        self._depth_camera_info_sub = self.node.create_subscription(
            CameraInfo,
            "camera/depth/camera_info",
            partial(self.on_info_update, cam_type=CameraType.DEPTH, side=View.DEPTH),
            1,
        )

        return ci

    def spin_ros(self) -> None:
        self._logger.info("Spin node")
        rclpy.spin(self.node)

    def on_image_update(self, msg, cam_type: CameraFeatures, side: View):
        """Get data from image. Callback for "/'side'_image "subscriber."""
        frame = ROSFrame(Timestamp(seconds=msg.header.stamp.sec, nanos=msg.header.stamp.nanosec), msg.data.tobytes())
        self.cams_frame[cam_type][side] = frame

    def on_raw_image_update(self, msg, cam_type: CameraFeatures, side: View):
        """Get data from image. Callback for "/'side'_image "subscriber."""

        data = msg.data.tobytes()
        encoding = msg.encoding

        if cam_type == CameraType.DEPTH and self._gazebo_mode:
            data = np.frombuffer(data, dtype=np.float32)  # specific conversion from Gazebo 32FC1 (in m) to 16UC1 (in mm)
            data = data * 1000.0
            data = data.astype(np.uint16)
            data = data.tobytes()
            encoding = "16UC1"
        frame = ROSFrameRaw(
            timestamp=Timestamp(seconds=msg.header.stamp.sec, nanos=msg.header.stamp.nanosec),
            data=data,
            height=msg.height,
            width=msg.width,
            encoding=encoding,
            step=msg.step,
            isbigendian=msg.is_bigendian,
        )

        self._logger.warning(f"Image send")
        self.cams_frame[cam_type][side] = frame

    def on_info_update(self, msg, cam_type: CameraFeatures, side: View):
        """Get data from image. Callback for "/'side'_image "subscriber."""
        cam_info = ROSCamInfo(msg.height, msg.width, msg.distortion_model, msg.d, msg.k, msg.r, msg.p)
        self.cams_info[cam_type][side] = cam_info

    def register_to_server(self, server: grpc.Server):
        self._logger.info("Registering 'VideoServiceServicer' to server.")
        add_VideoServiceServicer_to_server(self, server)

    def GetAvailableCameras(self, request: CameraInfo, context: grpc.ServicerContext) -> ListOfCameraFeatures:
        self._init_cameras()
        return ListOfCameraFeatures(camera_feat=self._list_cam)

    def GetFrame(self, request: ViewRequest, context: grpc.ServicerContext) -> Frame:
        """
        Frames are encoded in JPG to save bandwidth
        """
        if request.camera_feat.name not in [c.name for c in self._list_cam]:
            self._logger.warning(f"Camera {request.camera_info.name} not opened")
            return Frame(data=None, timestamp=None)
        elif not request.camera_feat.stereo and request.view == View.RIGHT:
            self._logger.warning(f"Camera {request.camera_feat.name} has no stereo feature. Returning mono view")
            request.view = View.LEFT

        camtype = CameraType.TELEOP
        if request.camera_feat.name == CameraType.DEPTH.value:
            camtype = CameraType.DEPTH

        frame = self.cams_frame[camtype][request.view]

        if frame is None:
            self._logger.warning(f"No camera data published for {request.camera_feat.name}")
            return Frame(data=None, timestamp=None)

        return Frame(data=frame.data, timestamp=frame.timestamp)

    def GetDepth(self, request: ViewRequest, context: grpc.ServicerContext) -> FrameRaw:
        if request.camera_feat.name not in [c.name for c in self._list_cam]:
            self._logger.warning(f"Camera {request.camera_feat.name} not opened")
            return FrameRaw(data=None, timestamp=None, height=0, width=0)
        elif not request.camera_feat.depth or request.view != View.DEPTH:
            self._logger.warning(f"Camera {request.camera_feat.name} has no depth feature.")
            return FrameRaw(data=None, timestamp=None, height=0, width=0)

        frame = self.cams_frame[CameraType.DEPTH][request.view]

        if frame is None:
            self._logger.warning(f"No depth data published for {request.camera_feat.name}")
            return FrameRaw(data=None, timestamp=None, height=0, width=0)

        return FrameRaw(
            data=frame.data,
            timestamp=frame.timestamp,
            height=frame.height,
            width=frame.width,
            step=frame.step,
            encoding=frame.encoding,
            isbigendian=frame.isbigendian,
        )

    def GetParameters(self, request: ViewRequest, context: grpc.ServicerContext) -> CameraParameters:
        """
        Get camera parameters as defined in https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
        """
        if request.camera_feat.name not in [c.name for c in self._list_cam]:
            self._logger.warning(f"Camera {request.camera_feat.name} not opened")
            return CameraParameters()
        elif not request.camera_feat.stereo and request.view == View.RIGHT:
            self._logger.warning(f"Camera {request.camera_feat.name} has no stereo feature. Returning mono view")
            request.view = View.LEFT

        camtype = CameraType.TELEOP
        if request.camera_feat.name == CameraType.DEPTH.value:
            camtype = CameraType.DEPTH

        cam_param = self.cams_info[camtype][request.view]

        if cam_param is None:
            self._logger.warning(f"No camera parameters published for {request.camera_feat.name}")
            return CameraParameters()

        return CameraParameters(
            height=cam_param.height,
            width=cam_param.width,
            distortion_model=cam_param.distortion_model,
            D=cam_param.D,
            K=cam_param.K,
            R=cam_param.R,
            P=cam_param.P,
        )


def main():
    import argparse
    from concurrent import futures

    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=50065)
    parser.add_argument("--max-workers", type=int, default=10)
    parser.add_argument("--ros-args", action="store_true")
    parser.add_argument("--gazebo", default=False, action="store_true")

    args = parser.parse_args()

    servicer = ReachyGRPCVideoSDKServicer(args.gazebo)
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=args.max_workers))

    servicer.register_to_server(server)

    server.add_insecure_port(f"[::]:{args.port}")
    server.start()

    servicer.node.get_logger().info(f"Server started on port {args.port}.")
    server.wait_for_termination()


if __name__ == "__main__":
    main()
