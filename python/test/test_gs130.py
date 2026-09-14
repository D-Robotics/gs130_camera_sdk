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
    while True:
        image = device.read_image()
        if image is not None:
            return image
        time.sleep(0.01)


def next_imu(device):
    while True:
        packet = device.read_imu()
        if packet is not None:
            return packet
        time.sleep(0.005)


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
        print("camera available:", dev.available_camera())
        print("imu available:", dev.available_imu())
        print("imu name:", dev.imu_name)
        print("imu info:\n", dev.imu_info)
        print("eeprom name:", dev.eeprom_name)
        print("eeprom info:\n", dev.eeprom_info)

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
        section("Camera")
        output_dir = Path("gs130_images")
        if output_dir.exists():
            shutil.rmtree(output_dir)
        output_dir.mkdir()
        print("saving PNG images to:", output_dir.resolve())

        for index in range(10):
            images = next_image(dev)
            print("camera round %d" % (index + 1))
            for name, image in images.items():
                bgr = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_NV12)
                output = output_dir / ("%s_%02d.png" % (name, index + 1))
                if not cv2.imwrite(str(output), bgr):
                    raise RuntimeError("failed to save %s" % output)
                print(
                    " %s: shape=%s dtype=%s timestamp_ns=%d png=%s"
                    % (name, image.shape, image.dtype, image.timestamp_ns, output)
                )

        section("IMU")
        for index in range(10):
            packet = next_imu(dev)
            print(
                "imu round %d: timestamp_ns=%d accel=%s gyro=%s temp=%.2f fsync=%s"
                % (
                    index + 1,
                    packet.timestamp_ns,
                    packet.accel,
                    packet.gyro,
                    packet.temp,
                    packet.is_fsync,
                )
            )

        section("Cleanup")
        dev.stop()
        print("running after stop:", dev.running)
        print("saved PNG count:", len(list(output_dir.glob("*.png"))))

    print("closed after with:", dev.closed)
    print("GS130 smoke test passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
