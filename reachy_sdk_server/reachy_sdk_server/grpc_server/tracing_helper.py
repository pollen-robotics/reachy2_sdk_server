import os
from opentelemetry import trace, context
from opentelemetry.exporter.otlp.proto.grpc.trace_exporter import OTLPSpanExporter
from opentelemetry.sdk.resources import Resource
from opentelemetry.sdk.trace import TracerProvider
from opentelemetry.sdk.trace.export import (  # ConsoleSpanExporter,; SimpleSpanProcessor,
    BatchSpanProcessor,
)

from opentelemetry.trace.propagation.tracecontext import TraceContextTextMapPropagator

import pyroscope
from pyroscope import otel
from contextlib import nullcontext

otel_rootctx = context.get_current()
first_spans = {}

def tracer(service_name):
    # resource = Resource(attributes={"service.name": "grpc_server"})
    resource = Resource(attributes={"service.name": service_name})
    provider = TracerProvider(resource=resource)

    if profiling_enabled():
        provider.add_span_processor(otel.PyroscopeSpanProcessor())  # this can be commented
    provider.add_span_processor(
        BatchSpanProcessor(OTLPSpanExporter(endpoint="http://localhost:4317"))
    )

    trace.set_tracer_provider(provider)
    # trace.get_tracer_provider().add_span_processor(
    #     BatchSpanProcessor(OTLPSpanExporter(endpoint="http://localhost:4317"))
    # )

    return trace.get_tracer(service_name)

def span_links(span, spans=[]):
    spans.append(trace.Link(span.get_span_context()))
    return spans

TRACEPARENT_STR = "traceparent"
def traceparent():
    carrier = {}
    TraceContextTextMapPropagator().inject(carrier)
    return carrier[TRACEPARENT_STR]

def profiling_enabled():
    return os.environ.get("REACHY_MONITORING_ENABLE_PROFILING") is not None

def ctx_from_traceparent(traceparent):
    carrier = {TRACEPARENT_STR: traceparent}
    return TraceContextTextMapPropagator().extract(carrier=carrier)

def configure_pyroscope(service_name, tags={}):
    if profiling_enabled():
        pyroscope.configure(
            application_name=service_name,  # replace this with some name for your application
            server_address="http://localhost:4040",  # replace this with the address of your Pyroscope server
            sample_rate=5000,  # default is 100
            detect_subprocesses=True,  # detect subprocesses started by the main process; default is False
            oncpu=False,  # report cpu time only; default is True
            gil_only=False,  # only include traces for threads that are holding on to the Global Interpreter Lock; default is True
            # enable_logging=True,  # does enable logging facility; default is False
            enable_logging=False,  # does enable logging facility; default is False
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
