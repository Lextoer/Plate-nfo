from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        # Plate Server
        ExecuteProcess(
            cmd=[
                "gnome-terminal", "--", "python3", "/home/yahya/ros2_ws/src/py_srvcli/py_srvcli/plate_server.py"
            ],
            output="screen",
            shell=True
        ),
        # Plate Subscriber
        ExecuteProcess(
            cmd=[
                "gnome-terminal", "--", "python3", "/home/yahya/ros2_ws/src/py_srvcli/py_srvcli/plate_subscriber.py"
            ],
            output="screen",
            shell=True
        ),
        # Plate Client
        ExecuteProcess(
            cmd=[
                "gnome-terminal", "--", "python3", "/home/yahya/ros2_ws/src/py_srvcli/py_srvcli/plate_client.py"
            ],
            output="screen",
            shell=True
        ),
    ])
