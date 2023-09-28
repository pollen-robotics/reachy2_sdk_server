import argparse
import grpc
from concurrent import futures
import logging
import sys


from reachy_sdk_api_v2 import orbita2d_pb2_grpc, orbita3d_pb2_grpc, arm_pb2_grpc, reachy_pb2_grpc
from .arm import ArmServicer, FakeArm
from .orbita2d import Orbita2DServicer
from .orbita3d import Orbita3DServicer
from .reachy import ReachyServicer


def main(args: argparse.Namespace) -> int:
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))

    o2d_servicer = Orbita2DServicer()
    o3d_servicer = Orbita3DServicer()
    fake_arm = FakeArm(
        side="right",
        orbita2ds=o2d_servicer.orbitas,
        orbita3ds=o3d_servicer.orbitas,
    )

    orbita2d_pb2_grpc.add_Orbita2DServiceServicer_to_server(o2d_servicer, server)
    print("Orbita2DServiceServicer_to_server")
    orbita3d_pb2_grpc.add_Orbita3DServiceServicer_to_server(o3d_servicer, server)
    print("Orbita3DServiceServicer_to_server")
    arm_pb2_grpc.add_ArmServiceServicer_to_server(ArmServicer({"right_arm": fake_arm}), server)
    reachy_pb2_grpc.add_ReachyServiceServicer_to_server(ReachyServicer(fake_arm), server)

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
