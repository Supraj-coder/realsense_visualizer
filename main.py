import argparse
import queue
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Tuple
import numpy as np
import pyrealsense2 as rs
import rerun as rr
import zarr
from numcodecs import Blosc
from scipy.spatial.transform import Rotation as R


@dataclass(frozen=True)
class CameraConfig:
    # 1. Capture Settings
    width: int = 640
    height: int = 480
    fps: int = 30

    # 2. Storage Settings
    compressor = Blosc(cname="lz4", clevel=3, shuffle=Blosc.SHUFFLE)
    chunk_size_frames: int = 1
    meta_chunk_size: int = 100

    # 3. Component Constants
    color_channels: int = 3
    quat_components: int = 4

    # Properties for Zarr dataset shapes
    @property
    def color_shape(self)-> Tuple[int, int, int, int]:
        return (0, self.height, self.width, self.color_channels)

    @property
    def color_chunks(self)-> Tuple[int, int, int, int]:
        return (self.chunk_size_frames, self.height, self.width, self.color_channels)

    @property
    def depth_shape(self)-> Tuple[int, int, int]:
        return (0, self.height, self.width)

    @property
    def depth_chunks(self)-> Tuple[int, int, int]:
        return (self.chunk_size_frames, self.height, self.width)

    @property
    def rot_shape(self)-> Tuple[int, int]:
        return (0, self.quat_components)

    @property
    def rot_chunks(self)-> Tuple[int, int]:
        return (self.meta_chunk_size, self.quat_components)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", type=str, choices=["record", "play"], required=True)
    parser.add_argument("--path", type=str, required=True)
    args = parser.parse_args()

    cfg = CameraConfig()
    path = Path(args.path).with_suffix(".zarr")

    rr.init("Realsense_visualization", spawn=True)
    rr.log("world", rr.ViewCoordinates.RDF, static=True)
    # record block
    if args.mode == "record":
        pipeline = rs.pipeline()
        rs_config = rs.config()
        rs_config.enable_stream(rs.stream.depth, cfg.width, cfg.height, rs.format.z16, cfg.fps)
        rs_config.enable_stream(rs.stream.color, cfg.width, cfg.height, rs.format.rgb8, cfg.fps)
        rs_config.enable_stream(rs.stream.gyro)

        profile = pipeline.start(rs_config)
        align = rs.align(rs.stream.color)
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

        rr.log(
            "world/camera",
            rr.Pinhole(
                resolution=[intr.width, intr.height],
                focal_length=[intr.fx, intr.fy],
                principal_point=[intr.ppx, intr.ppy],
            ),
            static=True,
        )

        current_rotation = R.identity()
        last_ts_gyro = None
        root = zarr.open(str(path), mode="w")
        color_ds = root.create_dataset(
            "color", shape=cfg.color_shape, chunks=cfg.color_chunks, dtype="uint8", compressor=cfg.compressor
        )
        depth_ds = root.create_dataset(
            "depth", shape=cfg.depth_shape, chunks=cfg.depth_chunks, dtype="uint16", compressor=cfg.compressor
        )
        ts_ds = root.create_dataset("timestamp", shape=(0,), chunks=(cfg.meta_chunk_size,), dtype="float64")
        rot_ds = root.create_dataset("rotation", shape=cfg.rot_shape, chunks=cfg.rot_chunks, dtype="float32")

        # Saved FPS to metadata so Playback knows the speed
        root.attrs.update(
            {
                "intrinsics": {
                    "width": intr.width,
                    "height": intr.height,
                    "fx": intr.fx,
                    "fy": intr.fy,
                    "ppx": intr.ppx,
                    "ppy": intr.ppy,
                },
                "depth_scale": depth_scale,
                "fps": cfg.fps,
                "imu_enabled": True,
            }
        )

        frame_queue: queue.Queue[Any] = queue.Queue(maxsize=128)
        stop_event = threading.Event()

        def writer_worker() -> None:
            count = 0
            while not stop_event.is_set() or not frame_queue.empty():
                try:
                    item = frame_queue.get(timeout=0.1)
                    color_ds.append(item["color"][None])
                    depth_ds.append(item["depth"][None])
                    ts_ds.append([item["ts"]])
                    rot_ds.append(item["rot"][None])
                    count += 1
                    frame_queue.task_done()
                except queue.Empty:
                    continue
            print(f"\n✅ Total Saved: {count} frames to {path}")

        t = threading.Thread(target=writer_worker, daemon=True)
        t.start()

        try:
            print(f"🔴 Recording {cfg.width}x{cfg.height} @ {cfg.fps}fps. Press Ctrl+C to stop.")
            while True:
                frames = pipeline.wait_for_frames()
                ts = frames.get_timestamp() / 1000.0
                rr.set_time_seconds("log_time", ts)

                gyro_f = frames.first_or_default(rs.stream.gyro)
                if gyro_f:
                    g_data = gyro_f.as_motion_frame().get_motion_data()
                    current_ts = gyro_f.get_timestamp() / 1000.0
                    if last_ts_gyro is not None:
                        dt = current_ts - last_ts_gyro
                        gyro_delta = R.from_rotvec(np.array([g_data.x, g_data.y, g_data.z]) * dt)
                        current_rotation = current_rotation * gyro_delta
                    last_ts_gyro = current_ts

                current_q = current_rotation.as_quat()
                rr.log("world/camera", rr.Transform3D(rotation=rr.components.RotationQuat(xyzw=current_q)))

                if frames.is_frameset():
                    aligned = align.process(frames)
                    depth_f = aligned.get_depth_frame()
                    color_f = aligned.get_color_frame()

                    if depth_f and color_f:
                        c_np, d_np = np.asanyarray(color_f.get_data()), np.asanyarray(depth_f.get_data())
                        rr.log("world/camera/rgb", rr.Image(c_np))
                        rr.log("world/camera/depth", rr.DepthImage(d_np, meter=1.0 / depth_scale))
                        frame_queue.put({"color": c_np, "depth": d_np, "ts": ts, "rot": current_q}, timeout=0.5)

        except KeyboardInterrupt:
            print("\n⏹️  Interrupt received. Finishing data write...")
        finally:
            stop_event.set()
            pipeline.stop()
            t.join()

        while not frame_queue.empty():
            time.sleep(0.1)

        print("🏁 Done.")
    # play block
    elif args.mode == "play":
        if not path.exists():
            print(f"❌ Error: Path {path} does not exist.")
            return
        with zarr.open(str(path), mode="r") as root:
            if "timestamp" in root and len(root.timestamp) > 0:
                meta = root.attrs
                intr = meta["intrinsics"]
                depth_scale = meta["depth_scale"]
                file_fps = meta.get("fps", cfg.fps)

                print(f"📂 Playing: {path} ({len(root.timestamp)} frames @ {file_fps}fps)")

                rr.log(
                    "world/camera",
                    rr.Pinhole(
                        resolution=[intr["width"], intr["height"]],
                        focal_length=[intr["fx"], intr["fy"]],
                        principal_point=[intr["ppx"], intr["ppy"]],
                    ),
                    static=True,
                )

                for i, timestamp in enumerate(root.timestamp):
                    rr.set_time_seconds("log_time", timestamp)
                    rr.set_time_sequence("frame_idx", i)
                    if "rotation" in root:
                        rr.log(
                            "world/camera", rr.Transform3D(rotation=rr.components.RotationQuat(xyzw=root.rotation[i]))
                        )
                    rr.log("world/camera/rgb", rr.Image(root.color[i]))
                    rr.log("world/camera/depth", rr.DepthImage(root.depth[i], meter=1.0 / depth_scale))
                    time.sleep(1 / file_fps)
                print("Playback finished.")


if __name__ == "__main__":
    main()
