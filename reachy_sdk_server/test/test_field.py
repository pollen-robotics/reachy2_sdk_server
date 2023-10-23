import pytest

from reachy_sdk_server.grpc_server import orbita2d


def test_cleanup_fields():
    assert orbita2d.cleanup_fields([]) == []
    assert orbita2d.cleanup_fields([orbita2d.Orbita2DField.NONE]) == []

    assert orbita2d.cleanup_fields(
        [orbita2d.Orbita2DField.NAME, orbita2d.Orbita2DField.PRESENT_POSITION]
    ) == ["name", "present_position"]

    assert set(orbita2d.cleanup_fields([orbita2d.Orbita2DField.ALL])) == set(
        orbita2d.conversion_table.keys()
    )
