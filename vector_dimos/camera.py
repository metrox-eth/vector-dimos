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

from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class VectorCamera(RealSenseCamera):
    _motion_sensor = None

    def _start_imu(self) -> None:
        import pyrealsense2 as rs

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

    def stop(self) -> None:  # type: ignore[override]
        if self._motion_sensor is not None:
            try:
                self._motion_sensor.stop()
                self._motion_sensor.close()
            except Exception:  # noqa: BLE001
                logger.exception("VectorCamera: motion sensor stop failed")
            self._motion_sensor = None
        super().stop()
