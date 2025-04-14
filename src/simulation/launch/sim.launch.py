from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
import os

def generate_launch_description():
    return LaunchDescription([
        # Set Gazebo model path
        SetEnvironmentVariable(
            name="GAZEBO_MODEL_PATH",
            value=os.path.join(os.environ["HOME"], "ros2_ws/src/simulation/models")
        ),

        # Launch Gazebo with world file
        ExecuteProcess(
            cmd=["ros2", "launch", "gazebo_ros", "gazebo.launch.py", 
                 "world:=/home/juka/ros2_ws/src/simulation/worlds/air_defense_world.sdf"],
            output="screen"
        ),

        # Spawn balloons separately
        ExecuteProcess(
            cmd=["ros2", "run", "simulation", "spawn_multiple_balloons"],
            output="screen"
        ),
    ])
