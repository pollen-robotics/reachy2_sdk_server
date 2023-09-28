import time
import yaml


def endless_get_stream(func, request, context, period):
    while True:
        yield func(request, context)
        time.sleep(period)


def read_config_file(config_file):
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    return config
