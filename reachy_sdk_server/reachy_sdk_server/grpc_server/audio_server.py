import grpc
import rclpy
from sound_play.libsoundplay import SoundClient


class ReachyGRPCAudioSDKServicer:
    def __init__(self) -> None:
        rclpy.init()

    def test(self) -> None:
        node = rclpy.create_node("soundclient_example")

        node.get_logger().info("Example: Playing sounds in *blocking* mode.")
        soundhandle = SoundClient(node, blocking=True)

        node.get_logger().info("Playing say-beep at full volume.")
        soundhandle.playWave("say-beep.wav")

        node.get_logger().info("Playing say-beep at volume 0.3.")
        soundhandle.playWave("say-beep.wav", volume=0.3)

        node.get_logger().info("Speaking some long string.")
        soundhandle.say("It was the best of times, it was the worst of times.")


def main():
    import argparse
    from concurrent import futures

    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=50051)
    parser.add_argument("--max-workers", type=int, default=10)
    parser.add_argument("reachy_config", type=str)
    args = parser.parse_args()

    servicer = ReachyGRPCAudioSDKServicer()

    servicer.test()

    return

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=args.max_workers))

    servicer.register_all_services(server)

    server.add_insecure_port(f"[::]:{args.port}")
    server.start()

    servicer.logger.info(f"Server started on port {args.port}.")
    server.wait_for_termination()


if __name__ == "__main__":
    main()
