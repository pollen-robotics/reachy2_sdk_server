import os
from opentelemetry import trace, context
from opentelemetry.exporter.otlp.proto.grpc.trace_exporter import OTLPSpanExporter
from opentelemetry.sdk.resources import Resource
from opentelemetry.sdk.trace import TracerProvider
from opentelemetry.sdk.trace.export import (  # ConsoleSpanExporter,; SimpleSpanProcessor,
    BatchSpanProcessor, )

from opentelemetry.trace.propagation.tracecontext import TraceContextTextMapPropagator
from opentelemetry.instrumentation import grpc as grpc_instrumentation

import pyroscope
from pyroscope import otel
import contextlib
from .tracing_helper_constants import localhoststr
from viztracer import VizTracer
from dataclasses import dataclass

otel_rootctx = context.get_current()
first_spans = {}

VIZTRACER_REPORTSDIR = "/home/reachy/viztracer_reports"


def pyroscope_enabled():
    return os.environ.get("REACHY_ENABLE_PROFILING_PYROSCOPE") is not None


def viztracer_enabled():
    return os.environ.get("REACHY_ENABLE_PROFILING_VIZTRACER") is not None


def otel_spans_enabled():
    return os.environ.get("REACHY_ENABLE_OTEL_SPANS") is not None


if viztracer_enabled():
    os.makedirs(VIZTRACER_REPORTSDIR, exist_ok=True)


@dataclass
class DummyContext:
    """Class for keeping track of an item in inventory."""

    trace_id: int
    span_id: float


class DummySpan:
    """
    Used when spans are disabled
    """

    N = 0

    def get_span_context(self):
        self.N += 1
        return DummyContext(trace_id=self.N, span_id=self.N)

    def set_attributes(_):
        pass

    def end(_):
        pass


class PollenSpan(contextlib.ExitStack):

    def __init__(
        self,
        tracer,
        trace_name,
        kind=trace.SpanKind.INTERNAL,
        context=None,
        with_pyroscope=True,
        with_viztracer=False,
        pyroscope_tags={},
    ):
        super().__init__()
        self.tracer = tracer
        self.trace_name = trace_name
        self.context = context
        self.kind = kind
        self.with_viztracer = with_viztracer
        self.with_pyroscope = with_pyroscope
        self.pyroscope_tags = pyroscope_tags

    def __enter__(self):
        """
        Returns only the opentelemetry Span obj for now
        """
        stack = super().__enter__()
        self.span = self.enter_context(
            self.tracer.start_as_current_span(
                self.trace_name, kind=self.kind, context=self.context
            ) if otel_spans_enabled() else contextlib.nullcontext(DummySpan))

        if pyroscope_enabled() and self.with_pyroscope:
            self.pyroscope = self.enter_context(
                pyroscope.tag_wrapper(self.pyroscope_tags))
        if viztracer_enabled() and self.with_viztracer:
            ctx = self.span.get_span_context()
            self.viztracer = VizTracer(
                # verbose=True,
                output_file=
                f"{VIZTRACER_REPORTSDIR}/{ctx.trace_id}-{ctx.span_id}-{self.trace_name}.json",
                log_async=True,
                log_gc=True,
            )
        return stack


def tracer(service_name, grpc_type=""):
    if otel_spans_enabled():
        match grpc_type:
            case "":
                # not auto-instrumentate
                pass
            case "server":
                grpc_instrumentation.GrpcInstrumentorServer().instrument()
            case "client":
                grpc_instrumentation.GrpcInstrumentorClient().instrument()
            case _:
                ValueError("Sorry, no numbers below zero")

        # resource = Resource(attributes={"service.name": "grpc_server"})
        resource = Resource(attributes={"service.name": service_name})
        provider = TracerProvider(resource=resource)

        if pyroscope_enabled():
            provider.add_span_processor(otel.PyroscopeSpanProcessor())
        provider.add_span_processor(
            BatchSpanProcessor(
                OTLPSpanExporter(endpoint=f"http://{localhoststr}:4317")))

        trace.set_tracer_provider(provider)
        # trace.get_tracer_provider().add_span_processor(
        #     BatchSpanProcessor(OTLPSpanExporter(endpoint="http://localhost:4317"))
        # )

        return trace.get_tracer(service_name)
    return None


def span_links(span, spans=[]):
    spans.append(trace.Link(span.get_span_context()))
    return spans


def configure_pyroscope(service_name, tags={}):
    if pyroscope_enabled():
        pyroscope.configure(
            application_name=
            service_name,  # replace this with some name for your application
            server_address=
            f"http://{localhoststr}:4040",  # replace this with the address of your Pyroscope server
            sample_rate=5000,  # default is 100
            detect_subprocesses=
            True,  # detect subprocesses started by the main process; default is False
            oncpu=False,  # report cpu time only; default is True
            gil_only=
            False,  # only include traces for threads that are holding on to the Global Interpreter Lock; default is True
            # enable_logging=True,  # does enable logging facility; default is False
            enable_logging=
            False,  # does enable logging facility; default is False
            report_pid=True,  # default False
            report_thread_id=True,  # default False
            report_thread_name=True,  # default False
            tags=tags,
        )


def first_span(key):
    if key not in first_spans:
        first_spans[key] = trace.get_current_span()
        print("first_span key:", key, first_spans[key])
    return first_spans[key]


#######################################################################################
# dummy function to be disabled when otel spans off


def real_travel_span(name, start_time, tracer, context=None):
    """
    Creates a span with a provided start_time.
    This is a workaround to simulate having started the span in the past.
    It's used to create a "fake" span for messages traveling between processes.
    """
    with tracer.start_span(name, start_time=start_time, context=context):
        pass


def dummy_travel_span(name, start_time, tracer, context=None):
    pass


TRACEPARENT_STR = "traceparent"


def real_traceparent():
    carrier = {}
    TraceContextTextMapPropagator().inject(carrier)
    return carrier[TRACEPARENT_STR]


def dummy_traceparent():
    return ""


def real_ctx_from_traceparent(traceparent):
    carrier = {TRACEPARENT_STR: traceparent}
    return TraceContextTextMapPropagator().extract(carrier=carrier)


def dummy_ctx_from_traceparent(_):
    return None


travel_span = real_travel_span
traceparent = real_traceparent
ctx_from_traceparent = real_ctx_from_traceparent
if not otel_spans_enabled():
    travel_span = dummy_travel_span
    traceparent = dummy_traceparent
    ctx_from_traceparent = dummy_ctx_from_traceparent
#######################################################################################
