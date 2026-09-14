from setuptools import find_packages, setup

package_name = "gs130_ros"

setup(
    name=package_name,
    version="0.1.0",
    description="ROS 2 interface for the GS130 stereo camera and IMU on RDK X5",
    license="MIT",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/gs130_camera.launch.py",
            "launch/gs130_web.launch.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": ["camera_node = gs130_ros.camera_node:main"]},
)
