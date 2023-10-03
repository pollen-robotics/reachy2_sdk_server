import argparse
import grpc
from concurrent import futures
import logging
import sys


from reachy_sdk_api_v2 import orbita2d_pb2_grpc, orbita3d_pb2_grpc, arm_pb2_grpc, reachy_pb2_grpc, dynamixel_motor_pb2_grpc, head_pb2_grpc
from .arm import ArmServicer, FakeArm
from .head import HeadServicer, FakeHead
from .orbita2d import Orbita2DServicer
from .orbita3d import Orbita3DServicer
from .dynamixel_motor import DynamixelMotorServicer
from .reachy import ReachyServicer


def main(args: argparse.Namespace) -> int:
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))

    o2d_servicer = Orbita2DServicer()
    o3d_servicer = Orbita3DServicer()
    dm_servicer = DynamixelMotorServicer()
    fake_arm = FakeArm(
        side="right",
        orbita2ds=o2d_servicer.orbitas,
        orbita3ds=o3d_servicer.orbitas,
    )
    fake_head = FakeHead(
        orbita3ds=o3d_servicer.orbitas,
        dynamixelmotors=dm_servicer.antennas,
    )

    orbita2d_pb2_grpc.add_Orbita2DServiceServicer_to_server(o2d_servicer, server)
    print("Orbita2DServiceServicer_to_server")
    orbita3d_pb2_grpc.add_Orbita3DServiceServicer_to_server(o3d_servicer, server)
    print("Orbita3DServiceServicer_to_server")
    dynamixel_motor_pb2_grpc.add_DynamixelMotorServiceServicer_to_server(dm_servicer, server)
    print("DynamixelMotorServiceServicer_to_server")
    arm_pb2_grpc.add_ArmServiceServicer_to_server(ArmServicer({"right_arm": fake_arm}), server)
    head_pb2_grpc.add_HeadServiceServicer_to_server(HeadServicer({"head": fake_head}), server)
    reachy_pb2_grpc.add_ReachyServiceServicer_to_server(ReachyServicer(fake_arm, fake_head), server)

    server.add_insecure_port(f"[::]:{args.port}")
    server.start()
    print("server started")

    server.wait_for_termination()

    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=50051)
    parser.add_argument("--verbose", action="store_true", default=False)
    args = parser.parse_args()

    if args.verbose:
        logging.basicConfig(level=logging.INFO)

    sys.exit(main(args))
