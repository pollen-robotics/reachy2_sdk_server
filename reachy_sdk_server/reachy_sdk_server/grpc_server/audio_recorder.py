import logging

import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst


class AudioRecorder:
    def __init__(self) -> None:
        self._logger = logging.getLogger(__name__)
        self._pipeline = None
        Gst.init(None)

    def make_pipe(self, filename):
        self._logger.info("Creating pipeline")
        self._pipeline = Gst.Pipeline.new()
        alsasrc = self._add_alsasrc()
        queue = self._add_queue()
        audioconvert = self._add_audioconvert()
        # encoder = self._add_mp3encoder()
        encoder = self._add_opusencoder()
        mux = self._add_oggmux()
        filesink = self._add_filesink(filename)

        if not Gst.Element.link(alsasrc, queue):
            self._logger.error("Failed to link alsasrc -> queue")
        if not Gst.Element.link(queue, audioconvert):
            self._logger.error("Failed to link queue -> audioconvert")
        if not Gst.Element.link(audioconvert, encoder):
            self._logger.error("Failed to link  audioconvert -> encoder")
        if not Gst.Element.link(encoder, mux):
            self._logger.error("Failed to link  encoder -> mux")
        if not Gst.Element.link(mux, filesink):
            self._logger.error("Failed to link mux -> filesink")

    def _add_alsasrc(self):  # type: ignore[no-untyped-def]
        assert self._pipeline is not None
        alsasrc = Gst.ElementFactory.make("alsasrc")
        self._pipeline.add(alsasrc)
        return alsasrc

    def _add_audioconvert(self):  # type: ignore[no-untyped-def]
        assert self._pipeline is not None
        audioconvert = Gst.ElementFactory.make("audioconvert")
        self._pipeline.add(audioconvert)
        return audioconvert

    def _add_mp3encoder(self):  # type: ignore[no-untyped-def]
        assert self._pipeline is not None
        encoder = Gst.ElementFactory.make("lamemp3enc")
        self._pipeline.add(encoder)
        return encoder

    def _add_opusencoder(self):
        assert self._pipeline is not None
        encoder = Gst.ElementFactory.make("vorbisenc")
        self._pipeline.add(encoder)
        return encoder

    def _add_oggmux(self):
        assert self._pipeline is not None
        mux = Gst.ElementFactory.make("oggmux")
        self._pipeline.add(mux)
        return mux

    def _add_queue(self):  # type: ignore[no-untyped-def]
        assert self._pipeline is not None
        queue = Gst.ElementFactory.make("queue")
        self._pipeline.add(queue)
        return queue

    def _add_filesink(self, filename):  # type: ignore[no-untyped-def]
        assert self._pipeline is not None
        filesink = Gst.ElementFactory.make("filesink")
        filesink.set_property("location", filename)
        self._pipeline.add(filesink)
        return filesink

    def start(self):
        if self._pipeline is not None:
            ret = self._pipeline.set_state(Gst.State.PLAYING)
            if ret not in [Gst.StateChangeReturn.SUCCESS, Gst.StateChangeReturn.ASYNC]:
                self._logger.error(f"Failed to transition pipeline to PLAYING: {ret}")
            else:
                self._logger.info("Pipeline started")
        else:
            self._logger.warning("Pipeline not created. Nothing to do.")

    def stop(self):
        if self._pipeline is not None:
            self._pipeline.set_state(Gst.State.NULL)
            self._logger.info("Pipeline stopped")


def main():
    import os
    import time

    logging.basicConfig(level=logging.DEBUG)
    os.environ["GST_DEBUG"] = "2"

    audiorecorder = AudioRecorder()
    audiorecorder.make_pipe("test.ogg")
    audiorecorder.start()

    time.sleep(2)

    audiorecorder.stop()


if __name__ == "__main__":
    main()
