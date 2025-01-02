from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import psutil
from time import sleep

def kill_sitl_processes():
    """
    Terminate existing SITL processes
    """
    for proc in psutil.process_iter(['name', 'cmdline']):
        try:
            print(f"Checking process: {proc.info['name']}")
            # Check for ardupilot_sitl processes
            if 'ardupilot_sitl' in (proc.info['name'] or ''):
                print(f"Found process: {proc.info['name']}")
                try:
                    proc.terminate()
                    print(f"Terminated SITL process: {proc.info['name']}")
                except Exception as e:
                    print(f"Could not terminate process: {e}")
            else:
                print(f"Skipping process: {proc.info['name']}, because it's not ardupilot_sitl")
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
    

def generate_launch_description():
    """
    Generate the launch description.
    * launch description is a list of actions that are executed in order
    """
    # kill all the processes
    kill_sitl_processes()
    sleep(2)
    
    pkg_drone_swarm_auto_setup_pkg = get_package_share_directory("drone_swarm_auto_setup_pkg")
    robots_number = 3
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
        }.items(),
        )
        
       
         robots.append(robot)


    return LaunchDescription(
        [
            *robots, # Unpack the list of robots
        ]
    )
