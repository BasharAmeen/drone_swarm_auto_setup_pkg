#!/usr/bin/env python3
import os
import time
import rclpy
from rclpy.node import Node
import subprocess
import signal
import psutil

class DroneSwarmResetNode(Node):
    def __init__(self):
        super().__init__('drone_swarm_reset_node')
        
        # Configuration parameters
        self.declare_parameter('num_drones', 1)
        self.declare_parameter('base_mavlink_port', 14540)
        
        # Read parameters
        self.num_drones = self.get_parameter('num_drones').value
        self.base_mavlink_port = self.get_parameter('base_mavlink_port').value

    def kill_sitl_processes(self):
        """
        Terminate existing SITL processes
        """
        for proc in psutil.process_iter(['name', 'cmdline']):
            try:
                # Check for ardupilot_sitl processes
                if 'ardupilot_sitl' in (proc.info['name'] or ''):
                    print(f"Found process: {proc.info['name']}")
                    try:
                        proc.terminate()
                        self.get_logger().info(f"Terminated SITL process: {proc.info['name']}")
                    except Exception as e:
                        self.get_logger().warn(f"Could not terminate process: {e}")
                else:
                    print(f"Skipping process: {proc.info['name']}, because it's not ardupilot_sitl")
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
        
        # Give some time for processes to terminate
        time.sleep(2)

    def restart_sitl_instances(self):
        # List to store robot launch actions
        robots = []
        
        # Get package paths
        pkg_ardupilot_gazebo = get_package_share_directory('ardupilot_gazebo')
        pkg_ardupilot_sitl = get_package_share_directory('ardupilot_sitl')

        # Create SITL instances for each drone
        for drone_num in range(1, self.num_drones + 1):
            # Calculate ports and addresses
            port = 2009 + (drone_num * 10)
            instance = str(drone_num)
            sim_address = f"127.0.0.1:{9002 + ((drone_num-1) * 10)}"
            master = f"tcp:127.0.0.1:{5750 + (drone_num * 10)}"
            sitl_addr = f"127.0.0.1:{5491 + (drone_num * 10)}"
            out = f"127.0.0.1:{14540 + (drone_num * 10)}"

            # Create SITL launch description
            robot = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("ardupilot_sitl"),
                                "launch",
                                "sitl_dds_udp.launch.py",
                            ]
                        ),
                    ]
                ),
                launch_arguments={
                    "transport": "udp4",
                    "port": str(port),
                    "synthetic_clock": "True",
                    "wipe": "False",
                    "model": "json",
                    "speedup": "1",
                    "slave": "0",
                    "instance": str(int(instance) - 1),
                    "defaults": (
                        os.path.join(
                            pkg_ardupilot_gazebo,
                            "config",
                            "gazebo-iris-gimbal.parm",
                        )
                        + ","
                        + os.path.join(
                            pkg_ardupilot_sitl,
                            "config",
                            "default_params",
                            "dds_udp.parm",
                        )
                    ),
                    "sim_address": sim_address,
                    "master": master,
                    "sitl": sitl_addr,
                    "out": out,
                }.items(),
            )
            
            # Add the robot launch action to the list
            robots.append(robot)

        # Create launch description and add robot actions
        launch_description = LaunchDescription()
        for robot in robots:
            launch_description.add_action(robot)

        return launch_description

    def reset_simulation(self):
        """
        Comprehensive SITL reset procedure
        """
        self.get_logger().info(f"Starting SITL reset for {self.num_drones} drones...")
        
        # 1. Kill existing SITL processes
        self.kill_sitl_processes()
        
        # 2. Restart SITL Instances
        self.restart_sitl_instances()
        
        # Wait for systems to stabilize
        time.sleep(3)
        
        self.get_logger().info("SITL reset complete!")

def main(args=None):
    rclpy.init(args=args)
    reset_node = DroneSwarmResetNode()
    
    try:
        reset_node.reset_simulation()
    except Exception as e:
        reset_node.get_logger().error(f"Reset failed: {e}")
    finally:
        reset_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()