"""GS130 smoke sample.

Usage on the board:

    python3 test_gs130.py RDKX5 GS130WI resize 640 480 30 200
"""

import shutil
import sys
import time
from pathlib import Path

import cv2
import gs130


SEPARATOR = "=" * 72


def section(title):
    print("\n" + SEPARATOR)
    print(title)
    print(SEPARATOR)


def show_errors(device):
    """Show what a failed SDK call raises, so callers know what to catch."""
    try:
        device.read_image()
    except gs130.GS130Error as error:
        print("read_image() before start() raised:")
        print(" %s" % error)
        print(" code=%s (%s) func=%s"
              % (error.code, gs130.ErrorCode(error.code).name, error.func))
    else:
        raise RuntimeError("reading before start() should not have succeeded")


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


def next_image(device):
    """Wait for the next stereo frame pair.

    available_camera() is the queue depth, so ask it first: read_image() on an
    empty queue just returns None, and polling it blindly burns almost every
    call on a timeout.
    """
    while device.available_camera() == 0:
        time.sleep(0.005)
    return device.read_image()


def next_imu(device):
    while device.available_imu() == 0:
        time.sleep(0.001)
    return device.read_imu()


def measure_rate(device, seconds):
    """The two queues need different treatment to report an honest rate.

    The camera queue caps at a few frames, so once it is full its depth stops
    growing and a fill-rate tells you nothing. The IMU queue caps much later,
    so its fill-rate over a window is a real measurement.
    """
    before = device.available_imu()
    start = time.time()
    time.sleep(seconds)
    elapsed = time.time() - start
    return (device.available_imu() - before) / elapsed, device.available_imu()


def collect_frames(device, count):
    """Read `count` frames as they arrive, without any per-frame work."""
    while device.available_camera() == 0:
        time.sleep(0.002)
    device.read_image()
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
    """Read `count` IMU packets as they arrive, starting from an empty queue."""
    while device.available_imu() > 0:
        device.read_imu()

    timestamps = []
    for _ in range(count):
        while device.available_imu() == 0:
            time.sleep(0.001)
        timestamps.append(device.read_imu().timestamp_ns)
    return timestamps


def rate_from(timestamps):
    """Rate from the median gap, so a burst of queued frames cannot distort it."""
    if len(timestamps) < 2:
        return 0.0
    gaps = sorted(
        (timestamps[i + 1] - timestamps[i]) / 1e9 for i in range(len(timestamps) - 1)
    )
    median = gaps[len(gaps) // 2]
    return 1.0 / median if median > 0 else 0.0


def main():
    if len(sys.argv) != 8:
        print("usage: test_gs130.py PLATFORM DEVICE MODE WIDTH HEIGHT FPS ODR")
        return 2

    platform, device_name, mode_name = sys.argv[1:4]
    width, height, fps, odr = (int(value) for value in sys.argv[4:8])
    mode = mode_from_text(mode_name)

    section("Configuration")
    print("platform:", platform)
    print("device:", device_name)
    print("mode:", mode_name)
    print("size: %d x %d" % (width, height))
    print("fps:", fps)
    print("imu odr:", odr)
    print("python package:", gs130.__version__)
    print("libgs130:", gs130.library_version())

    config = gs130.Config.preset(
        platform, device_name, mode, width, height, fps, odr
    )

    with gs130.Device(config) as dev:
        section("Device")
        print("running:", dev.running)
        print("closed:", dev.closed)
        print("stitched:", dev.stitched)
        print("imu name:", dev.imu_name)
        print("imu info:\n", dev.imu_info)
        print("eeprom name:", dev.eeprom_name)
        print("eeprom info:\n", dev.eeprom_info)

        section("Errors")
        show_errors(dev)

        section("Calibration")
        left = dev.camera_intrinsics(gs130.CameraIndex.LEFT)
        right = dev.camera_intrinsics(gs130.CameraIndex.RIGHT)
        imu = dev.imu_intrinsics()
        calibration = dev.calibration()

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
        print("full calibration: loaded")

        section("Extrinsics")

        for source in gs130.ReferenceFrame:
            for target in gs130.ReferenceFrame:
                print("%s -> %s" % (source.name, target.name))
                print("  R =")
                print(dev.relative_R(source, target))
                print("  T =")
                print(dev.relative_T(source, target))

        dev.convert_calibration(
            gs130.ReferenceFrame.IMU,
            calibration.imu_R,
            calibration.imu_T,
        )
        print("calibration converted")

        dev.start()
        section("Streaming")

        # Both queues start empty and fill only while the stream runs, so
        # available_camera() / available_imu() are only meaningful from here on.
        # They are queue depths: read when they are non-zero rather than
        # calling the read functions blindly.
        print("queue depth at start: camera=%d imu=%d"
              % (dev.available_camera(), dev.available_imu()))
        time.sleep(1.0)
        print("queue depth after 1 s: camera=%d imu=%d"
              % (dev.available_camera(), dev.available_imu()))

        imu_hz, imu_depth = measure_rate(dev, 2.0)
        print("imu queue-fill rate over 2 s: %.1f Hz (depth now %d)"
              % (imu_hz, imu_depth))

        print("camera rate from 20 frames: %.1f Hz" % rate_from(collect_frames(dev, 20)))
        print("imu rate from 200 packets: %.1f Hz" % rate_from(collect_imu(dev, 200)))

        section("Camera")
        output_dir = Path("gs130_images")
        if output_dir.exists():
            shutil.rmtree(output_dir)
        output_dir.mkdir()
        print("saving PNG images to:", output_dir.resolve())

        timestamps = []
        for index in range(10):
            print("camera round %d: queue depth=%d" % (index + 1, dev.available_camera()))
            images = next_image(dev)
            timestamps.append(next(iter(images.values())).timestamp_ns)
            for name, image in images.items():
                bgr = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_NV12)
                output = output_dir / ("%s_%02d.png" % (name, index + 1))
                if not cv2.imwrite(str(output), bgr):
                    raise RuntimeError("failed to save %s" % output)
                print(
                    " %s: shape=%s dtype=%s timestamp_ns=%d png=%s"
                    % (name, image.shape, image.dtype, image.timestamp_ns, output)
                )

        print("camera rate from %d frames with PNG saving: %.1f Hz (rate is limited by"
              " the PNG write, not by the sensor; the queue caps at 4 and drops the rest)"
              % (len(timestamps), rate_from(timestamps)))

        section("IMU")
        imu_timestamps = []
        for index in range(10):
            depth = dev.available_imu()
            packet = next_imu(dev)
            delta_ms = (
                (packet.timestamp_ns - imu_timestamps[-1]) / 1e6
                if imu_timestamps
                else 0.0
            )
            imu_timestamps.append(packet.timestamp_ns)
            print(
                "imu round %d: queue depth=%d delta_ms=%.3f timestamp_ns=%d"
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

        print("imu rate from %d packets: %.1f Hz" % (len(imu_timestamps), rate_from(imu_timestamps)))

        section("Cleanup")
        dev.stop()
        print("running after stop:", dev.running)
        print("saved PNG count:", len(list(output_dir.glob("*.png"))))

    print("closed after with:", dev.closed)
    print("GS130 smoke test passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
