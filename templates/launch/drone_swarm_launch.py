from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    """
    Generate the launch description.
    * launch description is a list of actions that are executed in order
    """
    # Validate arguments

    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_drone_swarm_auto_setup_pkg = get_package_share_directory("drone_swarm_auto_setup_pkg")
    robots_number = 2
    robot_type = "iris"

    # create  a list of drones
    robots = []
    for i in range(1, int(robots_number) + 1):
         port =  2009 + (i * 10)
         instance = i
         sim_address = f"127.0.0.1:{9002 + ( (i-1) * 10)}" # first time will  be 9002, second time will be 9012
         master = f"tcp:127.0.0.1:{5750+ (i * 10)}"
         sitl = f"127.0.0.1:{5491 + (i * 10)}" # first time will  be 5501
         out = f"127.0.0.1:{14540  + (i * 10)}"
         # Calculate unique MAVROS FCU URLs for each drone
         # Format: udp://<local_ip>:<local_port>@<remote_port>
         mavros_local_port = 14551 + ((i-1) * 10)  # local port for MAVROS
         mavros_remote_port = 14555 + ((i-1) * 10)  # remote port for MAVROS
         fcu_url = f"udp://:14540@127.0.0.1:{14540 + (i * 10)}"        
         
         robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
            PathJoinSubstitution(
                [
                    pkg_drone_swarm_auto_setup_pkg,
                    "templates",
                    "launch",
                    "robots",
                    f"{robot_type}.launch.py",
                ]
            )
          ]
        ),
        launch_arguments={
            "namespace": f"drone{instance}",
            "port": str(port),
            "instance": str(instance),
            "sim_address": sim_address,
            "master": master,
            "sitl": sitl,
            "out": out,
            "fcu_url": fcu_url,  # Add FCU URL for MAVROS
        }.items(),
        )
        
       
         robots.append(robot)

    # Gazebo simulation server
    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": "-v4 -s -r " + str(Path(pkg_drone_swarm_auto_setup_pkg) /"created_resources"/ "worlds" / "iris_runway.sdf"),
        }.items(),
    )
    # Gazebo GUI
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-v4 -g"}.items(),
    )

    # RViz.
    rviz_config_DIR_path =  os.path.join(
        pkg_drone_swarm_auto_setup_pkg, "created_resources", "rviz",
    )   
    
    rviz_config_file_path = rviz_config_DIR_path +"/"+ f"rviz_cameras_config.rviz"  
    print ("rviz_config_DIR_path :", rviz_config_file_path)
    print( rviz_config_file_path)
    print( rviz_config_file_path)
    print("------------------------------------" * 3)
    # RViz.
    # rviz = Node(
    #         package="rviz2",
    #         executable="rviz2",
    #         arguments=[
    #             "-d", rviz_config_DIR_path +"/"+ f"rviz_cameras.rviz"
    #         ],
    #         name=f"drones_cameras_rviz",
    #         condition=IfCondition(LaunchConfiguration("rviz")),
    #         output="screen",
    #     )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d",  rviz_config_file_path],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true", description="Open RViz."),
            gz_sim_server,
            gz_sim_gui,
            *robots, # Unpack the list of robots
            rviz,
        ]
    )
