import uuid

from reachy_sdk_api_v2.component_pb2 import ComponentId

_component = {}


def make_random_component(component_cls, component_name):
    uid = len(_component)

    id = f"{component_name}-{uid}"
    serial_number = uuid.uuid4().hex

    c = component_cls(
        id=ComponentId(id=id),
        serial_number=serial_number,
    )

    _component[id] = c

    return c


def check_component_id(id, component_name, context):
    if id.id not in _component:
        context.abort(404, f"{component_name} {id.id} not found")


def get_component(id):
    return _component[id.id]


def get_all_components_by_type(component_cls):
    return [c for c in _component.values() if isinstance(c, component_cls)]
