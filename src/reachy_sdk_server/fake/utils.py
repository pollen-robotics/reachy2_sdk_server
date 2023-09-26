import time
import uuid


def endless_get_stream(func, request, context, period):
    while True:
        yield func(request, context)
        time.sleep(period)
