import argparse
import grpc
from concurrent import futures
import logging
import sys


from reachy_sdk_api_v2 import orbita2d_pb2_grpc
from .orbita2d import Orbita2D


def main(args: argparse.Namespace) -> int:
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))

    orbita2d_pb2_grpc.add_Orbita2DServiceServicer_to_server(Orbita2D(), server)

    server.add_insecure_port(f"[::]:{args.port}")
    server.start()

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
