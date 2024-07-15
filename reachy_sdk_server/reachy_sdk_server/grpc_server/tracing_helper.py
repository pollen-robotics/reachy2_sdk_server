from opentelemetry import trace
from opentelemetry.exporter.otlp.proto.grpc.trace_exporter import OTLPSpanExporter
from opentelemetry.sdk.resources import Resource
from opentelemetry.sdk.trace import TracerProvider
from opentelemetry.sdk.trace.export import (  # ConsoleSpanExporter,; SimpleSpanProcessor,
    BatchSpanProcessor,
)

from opentelemetry.trace.propagation.tracecontext import TraceContextTextMapPropagator


def tracer(service_name):
    # resource = Resource(attributes={"service.name": "grpc_server"})
    resource = Resource(attributes={"service.name": service_name})

    # trace.set_tracer_provider(TracerProvider())
    trace.set_tracer_provider(TracerProvider(resource=resource))
    trace.get_tracer_provider().add_span_processor(
        BatchSpanProcessor(OTLPSpanExporter(endpoint="http://localhost:4317"))
    )
    return trace.get_tracer(service_name)


TRACEPARENT_STR = "traceparent"
def traceparent():
    carrier = {}
    TraceContextTextMapPropagator().inject(carrier)
    return carrier[TRACEPARENT_STR]

def ctx_from_traceparent(traceparent):
    carrier = {TRACEPARENT_STR: traceparent}
    return TraceContextTextMapPropagator().extract(carrier=carrier)
