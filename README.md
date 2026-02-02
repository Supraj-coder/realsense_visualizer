# RealSense Depth Camera Data Visualizer (IMU inclusive)

A high-performance visualizer for Intel® RealSense™ Depth Camera (D435i) using Rerun. 

This tool provides synchronized logging of Color, Depth, and IMU data to Zarr format, enabling high-fidelity recording and playback.

## Features
- **Real-time Visualization**: Live stream camera data to Rerun during recording.
- **Efficient Storage**: Saves data to Zarr format with Blosc LZ4 compression.
- **IMU Integration**: Logs camera orientation (RotationQuat) from the Gyroscope.
- **Synchronized Playback**: Replays saved `.zarr` datasets at the original recorded FPS.

## Installation

1. **Install uv**:
   This project uses `uv` for lightning-fast dependency management.
   ```bash
   curl -sSf [https://astral.sh/uv/install.sh](https://astral.sh/uv/install.sh) | sh

2. **Install System Dependencies (Linux only): Required for high-performance compression used by Zarr.**
   ```bash
   sudo apt-get update && sudo apt-get install -y libblosc-dev liblz4-dev

3. **Setup Environment:**
   ```bash
   uv sync

4. **Record Data**
   Connect your RealSense camera (must use a USB 3.0 and above cable) and run the following to visualize and save data:
   (Make sure to orient the camera properly before starting to record)
   ```bash
   uv run main.py --mode record --path /path/to/my_data.zarr
  Press Ctrl+C to stop recording and finish writing data to disk.
  
5. **Playback Data**
   ```bash
   uv run main.py --mode play --path /path/to/my_data.zarr
   
6. **Running tests**
Ruff:
   ```bash
   uv run ruff format .
   uv run ruff check . --fix
   ```

mypy
   ```bash
   uv run mypy .
   ```

pytest (Note: we use PYTHONPATH="." to prevent ROS 2 system paths from interfering with the test environment)
  ```bash
  PYTHONPATH="." uv run pytest
  ```

<img width="1847" height="1054" alt="image" src="https://github.com/user-attachments/assets/6a0028cc-49f8-4201-8cf2-d61030380f73" />

















   

