"""GS130 smoke sample.

Usage on the board:

    python3 test_gs130.py GS130WI resize 640 480 30 200

It walks the public API once and prints what it finds: configuration, device
identity, calibration, extrinsics, and then a short capture run that saves the
frames as PNG.  It is a sample to read and eyeball, not an assertion suite.

Sections that need hardware this unit may not have are skipped with a note
rather than failing the run, so the script still reaches the camera.
"""

import shutil
import sys
import time
from pathlib import Path

import cv2
import gs130


SEPARATOR = "=" * 72
CAMERA_ROUNDS = 10
IMU_ROUNDS = 10


def section(title):
    print("\n" + SEPARATOR)
    print(title)
    print(SEPARATOR)


def mode_from_text(text):
    modes = {
        "raw": gs130.CameraMode.RAW,
        "resize": gs130.CameraMode.RESIZE,
        "rect": gs130.CameraMode.RECT,
    }
    try:
        return modes[text.lower()]
    except KeyError:
        raise ValueError("mode must be raw, resize, or rect")


# ---------------------------------------------------------------------------
# Capture helpers
# ---------------------------------------------------------------------------


def next_image(device):
    """Wait for the next frame, then take it.

    available_camera() is the queue depth, so ask it first: read_image() on an
    empty queue returns None, and polling it blindly burns almost every call.
    """
    while device.available_camera() == 0:
        time.sleep(0.005)
    return device.read_image()


def next_imu(device):
    while device.available_imu() == 0:
        time.sleep(0.001)
    return device.read_imu()


def imu_fill_rate(device, seconds):
    """How fast the IMU queue fills over a window.

    The camera queue caps at a few frames, so its depth stops growing once it
    is full and a fill rate would say nothing.  The IMU queue caps far later,
    so this is a real measurement.
    """
    before = device.available_imu()
    start = time.time()
    time.sleep(seconds)
    elapsed = time.time() - start
    return (device.available_imu() - before) / elapsed


def collect_frames(device, count):
    """Read ``count`` frames as they arrive, without any per-frame work."""
    while device.available_camera() > 0:  # start from an empty queue
        device.read_image()

    timestamps = []
    for _ in range(count):
        while device.available_camera() == 0:
            time.sleep(0.002)
        images = device.read_image()
        timestamps.append(next(iter(images.values())).timestamp_ns)
    return timestamps


def collect_imu(device, count):
    """Read ``count`` IMU packets as they arrive, starting from an empty queue."""
    while device.available_imu() > 0:
        device.read_imu()

    timestamps = []
    for _ in range(count):
        while device.available_imu() == 0:
            time.sleep(0.001)
        timestamps.append(device.read_imu().timestamp_ns)
    return timestamps


def rate_from(timestamps):
    """Rate from the median gap, so a burst of queued samples cannot distort it."""
    if len(timestamps) < 2:
        return 0.0
    gaps = sorted(
        (timestamps[i + 1] - timestamps[i]) / 1e9
        for i in range(len(timestamps) - 1)
    )
    median = gaps[len(gaps) // 2]
    return 1.0 / median if median > 0 else 0.0


# ---------------------------------------------------------------------------
# Report sections
# ---------------------------------------------------------------------------


def report_calibration(device):
    """Print the intrinsic data, or explain why there is none.

    Returns the full calibration, or None when the unit carries no EEPROM.
    Note that the IMU intrinsics come out of the EEPROM too, so they are
    available even on a unit with no IMU fitted.
    """
    left = device.camera_intrinsics(gs130.CameraIndex.LEFT)
    right = device.camera_intrinsics(gs130.CameraIndex.RIGHT)
    imu = device.imu_intrinsics()

    print("left intrinsics:\n", left.K)
    print("left distortion:", left.dist_model.name, left.dist_coeffs)
    print("right intrinsics:\n", right.K)
    print("right distortion:", right.dist_model.name, right.dist_coeffs)
    print("accelerometer misalign:\n", imu.accel_misalign)
    print("accelerometer scale:", imu.accel_scale)
    print("accelerometer bias:", imu.accel_bias)
    print("accelerometer noise:", imu.accel_noise)
    print("accelerometer random walk:", imu.accel_random_walk)
    print("gyroscope misalign:\n", imu.gyro_misalign)
    print("gyroscope scale:", imu.gyro_scale)
    print("gyroscope bias:", imu.gyro_bias)
    print("gyroscope noise:", imu.gyro_noise)
    print("gyroscope random walk:", imu.gyro_random_walk)

    return device.calibration()


def report_extrinsics(device):
    """Every device pose relative to every other one, both ways."""
    for source in gs130.ReferenceFrame:
        for target in gs130.ReferenceFrame:
            print("%s -> %s" % (source.name, target.name))
            print("  R =")
            print(device.relative_R(source, target))
            print("  T =")
            print(device.relative_T(source, target))


def capture_camera(device, output_dir):
    """Save CAMERA_ROUNDS frames as PNG and report the achieved rate."""
    timestamps = []
    for index in range(CAMERA_ROUNDS):
        print("round %d: queue depth=%d" % (index + 1, device.available_camera()))
        images = next_image(device)
        timestamps.append(next(iter(images.values())).timestamp_ns)
        for name, image in images.items():
            bgr = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_NV12)
            output = output_dir / ("%s_%02d.png" % (name, index + 1))
            if not cv2.imwrite(str(output), bgr):
                raise RuntimeError("failed to save %s" % output)
            print(
                " %s: shape=%s dtype=%s timestamp_ns=%d y_plane=%s png=%s"
                % (
                    name,
                    image.shape,
                    image.dtype,
                    image.timestamp_ns,
                    image.y_plane().shape,
                    output,
                )
            )

    print(
        "camera rate from %d frames with PNG saving: %.1f Hz (limited by the PNG"
        " write, not the sensor; the queue caps at 4 and drops the rest)"
        % (len(timestamps), rate_from(timestamps))
    )


def capture_imu(device):
    """Print IMU_ROUNDS packets and report the achieved rate."""
    timestamps = []
    for index in range(IMU_ROUNDS):
        depth = device.available_imu()
        packet = next_imu(device)
        delta_ms = (
            (packet.timestamp_ns - timestamps[-1]) / 1e6 if timestamps else 0.0
        )
        timestamps.append(packet.timestamp_ns)
        print(
            "round %d: queue depth=%d delta_ms=%.3f timestamp_ns=%d"
            " accel=%s gyro=%s temp=%.2f fsync=%s"
            % (
                index + 1,
                depth,
                delta_ms,
                packet.timestamp_ns,
                packet.accel,
                packet.gyro,
                packet.temp,
                packet.is_fsync,
            )
        )

    print(
        "imu rate from %d packets: %.1f Hz"
        % (len(timestamps), rate_from(timestamps))
    )


def main():
    if len(sys.argv) != 7:
        print("usage: test_gs130.py DEVICE MODE WIDTH HEIGHT FPS ODR")
        return 2

    device_name, mode_name = sys.argv[1:3]
    width, height, fps, odr = (int(value) for value in sys.argv[3:7])
    mode = mode_from_text(mode_name)

    section("Configuration")
    print("device:", device_name)
    print("mode:", mode_name)
    print("size: %d x %d" % (width, height))
    print("fps:", fps)
    print("imu odr:", odr)
    print("python package:", gs130.__version__)
    print("libgs130:", gs130.library_version(), "(%s)" % gs130.library_platform())

    config = gs130.preset(device_name, mode, width, height, fps, odr)

    with gs130.Device(config) as dev:
        section("Device")
        print("stitched:", dev.stitched)
        print("imu name:", dev.imu_name)
        print("eeprom name:", dev.eeprom_name)
        if dev.imu_info:
            print("imu info:\n", dev.imu_info)
        if dev.eeprom_info:
            print("eeprom info:\n", dev.eeprom_info)

        section("Reads before start")
        # Capture has not been started, so the SDK answers THREAD_CLOSED here.
        # The read functions turn any non-OK code into None, which is what a
        # capture loop wants; only start/close and the calibration calls raise.
        print("read_image(): %r" % dev.read_image())
        print("read_imu(): %r" % dev.read_imu())

        calibration = None
        section("Calibration")
        try:
            calibration = report_calibration(dev)
        except gs130.GS130Error as error:
            print("no calibration available: %s" % error)

        section("Extrinsics")
        if calibration is None:
            print("skipped, no calibration")
        else:
            report_extrinsics(dev)
            dev.convert_calibration(
                gs130.ReferenceFrame.IMU,
                calibration.imu_R,
                calibration.imu_T,
            )
            print("reference frame moved to the IMU")

        dev.start()
        section("Streaming")

        # Both queues are empty until the stream runs, so available_camera()
        # and available_imu() only mean something from here on.
        print(
            "queue depth at start: camera=%d imu=%d"
            % (dev.available_camera(), dev.available_imu())
        )
        time.sleep(1.0)
        print(
            "queue depth after 1 s: camera=%d imu=%d"
            % (dev.available_camera(), dev.available_imu())
        )

        if dev.imu_name is None:
            print("no IMU on this unit, skipping the IMU rate")
        else:
            print("imu queue-fill rate over 2 s: %.1f Hz" % imu_fill_rate(dev, 2.0))

        print(
            "camera rate from 20 frames: %.1f Hz"
            % rate_from(collect_frames(dev, 20))
        )
        if dev.imu_name:
            print(
                "imu rate from 200 packets: %.1f Hz"
                % rate_from(collect_imu(dev, 200))
            )

        section("Camera")
        output_dir = Path("gs130_images")
        if output_dir.exists():
            shutil.rmtree(output_dir)
        output_dir.mkdir()
        print("saving PNG images to:", output_dir.resolve())
        capture_camera(dev, output_dir)

        section("IMU")
        if dev.imu_name is None:
            print("skipped, no IMU on this unit")
        else:
            capture_imu(dev)

        section("Cleanup")
        dev.stop()
        print("saved PNG count:", len(list(output_dir.glob("*.png"))))

    print("closed after the with block:", dev.closed)
    print("GS130 smoke test passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
