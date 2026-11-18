from setuptools import setup

package_name = "lx_ros2_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Duckietown",
    maintainer_email="info@duckietown.com",
    description="Helpers shared by the Duckietown Learning Experiences on ROS 2.",
    license="Duckietown License 2.0",
    entry_points={"console_scripts": []},
)
