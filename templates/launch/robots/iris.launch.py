import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler

from launch.conditions import IfCondition

from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    """Setup launch configuration based on runtime context."""
    # Retrieve launch configurations with context
    namespace = LaunchConfiguration('namespace').perform(context)
    port = LaunchConfiguration('port').perform(context)
    instance = LaunchConfiguration('instance').perform(context)
    sim_address = LaunchConfiguration('sim_address').perform(context)
    master = LaunchConfiguration('master').perform(context)
    sitl = LaunchConfiguration('sitl').perform(context)
    out = LaunchConfiguration('out').perform(context)
    fcu_url = LaunchConfiguration('fcu_url').perform(context)

    # Package directories
    pkg_ardupilot_sitl = get_package_share_directory("ardupilot_sitl")
    pkg_drone_swarm_auto_setup_pkg = get_package_share_directory("drone_swarm_auto_setup_pkg")
    pkg_ardupilot_gazebo = get_package_share_directory("ardupilot_gazebo")
    pkg_mavros = get_package_share_directory("mavros")

    print("str (int(instance) - 1) :", str (int(instance) - 1))
    # SITL DDS UDP launch


    sitl_dds = IncludeLaunchDescription(
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
            "transport": "udp4", # use UDP transport (other options: udp6, tcp)
            "port": port, # port used to communicate with ardupilot
            "synthetic_clock": "True", 
            "wipe": "False", # wipe ardupilot logs
            "model": "json", # use JSON model to configure ardupilot
            "speedup": "1", # speed up simulation
            "slave": "0", # use slave mode (not used in this case)
            "instance":str (int(instance) - 1), # instance number (used to distinguish between multiple instances of ardupilot_sitl)
            "defaults": os.path.join(
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
            ),
            "sim_address": sim_address, # address of the Gazebo simulation server
            "master": master, # address of the Gazebo master server
            "sitl": sitl, # address of the SITL server
            "out": out,
        }.items(),
    )
    # Manage SDF resources path
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_sim_resource_path = os.environ["GZ_SIM_RESOURCE_PATH"]

        if "SDF_PATH" in os.environ:
            sdf_path = os.environ["SDF_PATH"]
            os.environ["SDF_PATH"] = sdf_path + ":" + gz_sim_resource_path
        else:
            os.environ["SDF_PATH"] = gz_sim_resource_path

    # Path to model SDF file
    model_sdf_file = os.path.join(
        pkg_drone_swarm_auto_setup_pkg, 
        "created_resources", 
        "models", 
        f"drone{instance}", 
        "model.sdf"
    )

    # Read model SDF
    with open(model_sdf_file, "r") as file:
        model_sdf = file.read()


    # Robot state publisher with proper namespace
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=f"drone{instance}",  # Add namespace
        name=f"robot_state_publisher_drone{instance}",
        output="both",
        parameters=[{
            "robot_description": model_sdf,
            "frame_prefix": f"drone{instance}/",
            "publish_frequency": 30.0,
            "use_tf_static": True
        }],
        remappings=[
            # Remap the robot_description topic to be unique for each drone
            ("/robot_description", f"/drone{instance}/robot_description"),
        ]
    )

    # Gazebo ROS bridge
    gazebo_ros_bridge_config_file = os.path.join(
        pkg_drone_swarm_auto_setup_pkg, 
        "created_resources", 
        "config", 
        "iris_bridge", 
        f"iris_bridge_{instance}.yaml"
    )   

    gazebo_ros_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=f"ros_gz_bridge_drone{instance}",
        parameters=[{
            "config_file": gazebo_ros_bridge_config_file,
            "qos_overrides./tf_static.publisher.durability": "transient_local",
            "tf_message_filter_queue_size": 100,
            "publish_rate": 50.0  # Reduce from default if needed
        }],
        output="screen"
    )
        # MAVROS node for this drone instance
    print(f"pkg_mavros: {pkg_mavros}")
        # MAVROS launch configuration
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mavros, 'launch', 'apm.launch.py')
        ),
        launch_arguments={
            'namespace': f'mavros_drone{instance}',
            'fcu_url': fcu_url,
            'tgt_system': str(instance),  # Convert to string
            'tgt_component': '1',
            'fcu_protocol': 'v2.0',
            'respawn_mavros': 'false',
        }.items(),
    )
    # Run mavros_node
    # mav_ros_node = Node(
    #     package='mavros',
    #     executable='mavros_node',
    #     name=f'mavros_node_drone{instance}',
    #     namespace=f'drone{instance}',
    #         parameters=[{
    #             'fcu_url': fcu_url,
    #             'tgt_system': int(instance),
    #             'tgt_component': "1",
    #             'fcu_protocol': 'v2.0',
    #             'respawn_mavros': False,
    #         }],        output='screen',
    #     emulate_tty=True
    # )

    # topic_tools_tf = Node(
    #     package="topic_tools",
    #     executable="relay",
    #     name=f"tf_relay_drone{instance}",
    #     arguments=[
    #         f"/drone{instance}/gz/tf",
    #         f"/drone{instance}/tf"  # Keep transforms namespaced
    #     ],
    #     parameters=[{
    #         "reliability": "reliable",
    #         "durability": "volatile"
    #     }],
    #     output="screen",
    #     respawn=False,
    #     condition=IfCondition(LaunchConfiguration("use_gz_tf")),
    # )

    return [
        sitl_dds,
        mavros_launch,
        # mav_ros_node,
        robot_state_publisher,
        gazebo_ros_bridge,
        # RegisterEventHandler(
        #     OnProcessStart(
        #         target_action=gazebo_ros_bridge,
        #         on_start=[
        #             topic_tools_tf
        #         ]
        #     ),
        # ),
        ]


def generate_launch_description():
    """Generate launch description with arguments."""
    # Declare launch arguments
    launch_args = [
        DeclareLaunchArgument(
            'namespace', 
            default_value='drone1',
            description='Namespace for the drone'
        ),
        DeclareLaunchArgument(
            'port', 
            default_value='2019',
            description='Port for communication'
        ),
        DeclareLaunchArgument(
            'instance', 
            default_value='1',
            description='Drone instance number'
        ),
        DeclareLaunchArgument(
            'sim_address', 
            default_value='127.0.0.1:9002',
            description='Simulation address'
        ),
        DeclareLaunchArgument(
            'master', 
            default_value='tcp:127.0.0.1:5760',
            description='Master address'
        ),
        DeclareLaunchArgument(
            'sitl', 
            default_value='127.0.0.1:5501',
            description='SITL address'
        ),
        DeclareLaunchArgument(
            'out', 
            default_value='127.0.0.1:14550',
            description='Output address'
        ),
        DeclareLaunchArgument(
            "use_gz_tf", 
            default_value="true", 
            description="Use Gazebo TF."
        ),
    ]

    # Create OpaqueFunction to handle runtime context
    opaque_func = OpaqueFunction(function=launch_setup)
    # Create LaunchDescription
    ld = LaunchDescription(launch_args)
    ld.add_action(opaque_func)
    return ld