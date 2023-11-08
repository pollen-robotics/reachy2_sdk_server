import time

import grpc
import rclpy
from sound_play.libsoundplay import SoundClient

from ..utils import get_list_audio_files
from .audio_capture_action_client import AudioCaptureActionClient
from .audio_recorder import AudioRecorder


# ToDo : move this code to actual functions called by grpc
class ReachyGRPCAudioSDKServicer:
    def __init__(self) -> None:
        rclpy.init()
        # Dummy node
        self.node = rclpy.create_node("soundclient_example")
        # note: non blocking mode for gprc?
        self.soundhandle = SoundClient(self.node, blocking=True)
        self._audiocaptureclient = AudioCaptureActionClient()
        self._audiorecorder = AudioRecorder()

    def test(self) -> None:
        # look into its default sounds folder if there is no path
        # https://github.com/ros-drivers/audio_common/tree/ros2/sound_play/sounds
        self.node.get_logger().info("Playing say-beep at full volume.")
        self.soundhandle.playWave("say-beep.wav")

        self.node.get_logger().info("Playing say-beep at volume 0.3.")
        self.soundhandle.playWave("say-beep.wav", volume=0.3)

    def say_text(self, text: str) -> None:
        self.node.get_logger().info(f"Say {text}")
        self.soundhandle.say(text)

    def play_sound(self, file_name: str) -> None:
        self.node.get_logger().info(f"Playing {file_name}")
        # could be a wav or ogg
        self.soundhandle.playWave(file_name)

    def stop(self) -> None:
        # note: not sure it is working. Need to double check ROS package
        self.node.get_logger().info(f"Stop playing")
        self.soundhandle.stopAll()

    def start_capture(self, filename: str) -> None:
        self.node.get_logger().info(f"Start recording {filename}")
        self._audiorecorder.make_pipe(filename)
        self._audiorecorder.start()

    def stop_capture(self) -> None:
        self.node.get_logger().info("Stop recording")
        self._audiorecorder.stop()


def main():
    import argparse
    from concurrent import futures

    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=50051)
    parser.add_argument("--max-workers", type=int, default=10)
    args = parser.parse_args()

    servicer = ReachyGRPCAudioSDKServicer()

    servicer.test()

    servicer.say_text("Hello I'm Reachy!")

    # path defined in the docker-compose (or when creating container)
    audiofiles = get_list_audio_files("/root/sounds/")
    print(audiofiles)
    # servicer.play_sound(audiofiles[0])

    time.sleep(1)

    servicer.stop()

    servicer.start_capture("/root/sounds/record.ogg")

    time.sleep(3)

    servicer.stop_capture()

    servicer.play_sound("/root/sounds/record.ogg")

    return

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=args.max_workers))

    servicer.register_all_services(server)

    server.add_insecure_port(f"[::]:{args.port}")
    server.start()

    servicer.logger.info(f"Server started on port {args.port}.")
    server.wait_for_termination()


if __name__ == "__main__":
    main()
