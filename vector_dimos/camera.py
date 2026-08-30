"""RealSenseCamera whose IMU streams from the already-open device.

dimOS's ``_start_imu`` opens a SECOND ``rs.pipeline`` on the same camera.
On our source-built RSUSB librealsense that second enumeration fails with
"No device connected" (seen 23/08), which is why VECTOR ran without its IMU
- and why wheel slip could drag the odometry (the doctrine since day
one: dead reckoning = IMU + lidar + depth, never the wheels).

This subclass starts the motion module through the sensor-level API on the
device handle the main pipeline already owns: no second pipeline, no second
enumeration. The frame callback, gyro/accel pairing and ``imu`` publishing
are inherited unchanged.
"""

from __future__ import annotations

import os

from dimos.core.core import rpc
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class VectorCamera(RealSenseCamera):
    _motion_sensor = None

    def _start_imu(self) -> None:
        import pyrealsense2 as rs

        # VECTOR_CAM_IMU=0: diagnostic switch, 30/08. The blink was born with
        # this very stream (24/08, 964a0a5: motion at 200 Hz interleaved with
        # depth on the ONE RSUSB pipeline) - the A/B that proves or clears it
        # is a stack with vs without the IMU, camera_floor gaps compared.
        # Cost while off: no gyro prior for kiss-icp, no ImuSlipDetector.
        if os.environ.get("VECTOR_CAM_IMU", "1").strip().lower() in ("0", "false", "off", "no"):
            logger.warning("VECTOR_CAM_IMU=0: IMU stream NOT started (diagnostic run - "
                           "no gyro prior, no slip detector)")
            return

        if self._profile is None:
            raise RuntimeError("main pipeline must be started before the IMU")
        device = self._profile.get_device()
        motion_sensor = None
        for sensor in device.query_sensors():
            if any(p.stream_type() == rs.stream.gyro for p in sensor.get_stream_profiles()):
                motion_sensor = sensor
                break
        if motion_sensor is None:
            raise RuntimeError("no motion sensor on this camera")

        def pick(stream_type, wanted_hz=None):
            profiles = [p for p in motion_sensor.get_stream_profiles() if p.stream_type() == stream_type]
            if wanted_hz is not None:
                exact = [p for p in profiles if p.fps() == wanted_hz]
                if exact:
                    return exact[0]
            return max(profiles, key=lambda p: p.fps())

        gyro = pick(rs.stream.gyro, self.config.imu_hz)
        accel = pick(rs.stream.accel)
        motion_sensor.open([gyro, accel])
        motion_sensor.start(self._on_motion_frame)
        self._motion_sensor = motion_sensor
        logger.info(f"VectorCamera IMU on the main device handle: gyro {gyro.fps()} Hz, accel {accel.fps()} Hz")

    # @rpc is not decoration for its own sake: RealSenseCamera.stop carries it,
    # and an override that drops it falls out of the class's rpcs table.
    # dimOS then proxies the call by pickling the module across the worker
    # pipe, which dies on the pipeline's lock ('cannot pickle _thread.lock'),
    # so the camera is never stopped and the device stays streaming.
    @rpc
    def stop(self) -> None:  # type: ignore[override]
        if self._motion_sensor is not None:
            try:
                self._motion_sensor.stop()
                self._motion_sensor.close()
            except Exception:  # noqa: BLE001
                logger.exception("VectorCamera: motion sensor stop failed")
            self._motion_sensor = None
        super().stop()
