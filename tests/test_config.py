# tests/test_config.py
from main import CameraConfig


def test_camera_config_resolution() -> None:
    cfg = CameraConfig(width=1280, height=720)
    assert cfg.width == 1280
    assert cfg.height == 720


def test_zarr_shapes() -> None:
    cfg = CameraConfig(width=640, height=480)
    # Check color shape: (0, H, W, C)
    assert cfg.color_shape == (0, 480, 640, 3)
    # Check depth shape: (0, H, W)
    assert cfg.depth_shape == (0, 480, 640)
    # Check rotation shape: (0, 4)
    assert cfg.rot_shape == (0, 4)


def test_fps_logic() -> None:
    cfg = CameraConfig(fps=60)
    assert cfg.fps == 60
