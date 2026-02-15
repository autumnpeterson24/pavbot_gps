"""
Launch file for ros node!
You don't technically need this but it is an easy way to get the node up and running super quickly and be able to set the parameters the way you want them!

Run instructions:
    ros2 launch pavbot_gps gps.launch.py

Check your nodes are working and publishing:
    (in another terminal)
    ros2 topic list
    ros2 topic echo safety/estop

    *should see a bunch of data:true*


    ROS commands to run: 
    ros2 topic echo /gps/fix
    ros2 topic echo /gps/cog_deg
        * Direction PAVBot is moving across the earth (0 deg = North, 90 deg = East, 180 deg = South, 270 deg = West)
        * only computed when moving so not good for stationary will change when have IMU
        * Use as heading for testing

    ros2 topic echo /gps/sog_mps
        * SOG: speed over ground (mps)
        * 1 knot = 0.514444 m/s
        * 


"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pavbot_gps',
            executable='pavbot_gps',
            name='pavbot_gps',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                # 'baud': 115200,
                'timeout_ms': 500
            }]
        )
    ])
