# this script is used to automate the setup for the drone swarm
# every drone needs:
# 1. sitl_dds: which is the drone's ardupilot_sitl instance
# unique launch arguments:
# launch_arguments={
#             "namespace": "drone1_namespace",
#             "transport": "udp4",
#             "port": "2019",
#             "synthetic_clock": "True",
#             "wipe": "False",
#             "model": "json",
#             "speedup": "1",
#             "slave": "0",
#             "instance": "0",  # First instance
#             "defaults": os.path.join(
#                 pkg_ardupilot_gazebo,
#                 "config",
#                 "gazebo-iris-gimbal.parm",
#             )
#             + ","
#             + os.path.join(
#                 pkg_ardupilot_sitl,
#                 "config",
#                 "default_params",
#                 "dds_udp.parm",
#             ),
#             "sim_address": "127.0.0.1:9002",
#             "master": "tcp:127.0.0.1:5760",
#             "sitl": "127.0.0.1:5501",
#             "out":"127.0.0.1:14550"
#         }
# the args that should be unique for each drone are:
# "namespace": "drone1_namespace",
# "port": "2019",
# "instance": "0",  # First instance
# "sim_address": "127.0.0.1:9002",
# "out":"127.0.0.1:14550"

# 2- iris_with_gimbal.sdf: which is the drone's gazebo model
# it should have a unique gazebo model directory and unique fdm_port_in (line 228 in iris_with_gimbal at ardupilot_gazebo/modles)
# 3- robot_state_publisher: which is the drone's robot_state_publisher used to publish the drone's pose to the gazebo world
# it is using the drone's gazebo model directory (ardupilot_gazebo/models e.x. iris_with_gimbal)
# 4- ROS-Gazebo bridge
# it needs a unique iris_bridge.yaml file in the ardupilot_gz_bringup/config directory for each drone
# the file should have a unique name for the drone model


import os
from ament_index_python.packages import get_package_share_directory # used to get the package share directory (ardupilot_sitl)
from launch_ros.substitutions import FindPackageShare # used to get the package share directory (ardupilot_sitl)
from launch.substitutions import PathJoinSubstitution # used to join paths together
from launch.substitutions import LaunchConfiguration # used to get the launch arguments
from launch.conditions import IfCondition # used to conditionally launch a launch file
from launch.event_handlers import OnProcessStart # used to launch a launch file when another launch file is started
from launch.actions import RegisterEventHandler # used to register an event handler
from launch import LaunchDescription # used to create a launch description


# 1- define directories
# the script assumes that you have built the ardupilot_sitl, ardupilot_gazebo, and ardupilot_gz packages
# get the packages share directory
# TODO(basha): use package base (before installitation) nested of shared packages (installed packages) 
pkg_ardupilot_sitl = get_package_share_directory("ardupilot_sitl")
pkg_ardupilot_gazebo = get_package_share_directory("ardupilot_gazebo")
pkg_ardupilot_gz_bringup = get_package_share_directory("ardupilot_gz_bringup")
pkg_ardupilot_gz_gazebo = get_package_share_directory("ardupilot_gz_gazebo") # contains the gazebo worlds

# check if the packages are built
if not os.path.exists(pkg_ardupilot_sitl):
    raise FileNotFoundError(f"The ardupilot_sitl package is not built. Please build it first.")
if not os.path.exists(pkg_ardupilot_gazebo):
    raise FileNotFoundError(f"The ardupilot_gazebo package is not built. Please build it first.")
if not os.path.exists(pkg_ardupilot_gz_bringup):
        raise FileNotFoundError(f"The ardupilot_gz_bringup package is not built. Please build it first.")

# 2- define the constants
DRONES_NUMBER = 2 # number of drones
SCRIPT_PATH = os.path.abspath(__file__) # get the script path
# SCRIPT_PATH is under drone_swarm_auto_setup_pkg/scripts/
# TEMPLATES_DIR is under drone_swarm_auto_setup_pkg/templates/
TEMPLATES_DIR = os.path.join(os.path.dirname(os.path.dirname(SCRIPT_PATH)), "templates")
MODELS_TEMPLATE_DIR = os.path.join(TEMPLATES_DIR, "models")
WORLDS_TEMPLATE_DIR = os.path.join(TEMPLATES_DIR, "worlds")
CONFIG_TEMPLATE_DIR = os.path.join(TEMPLATES_DIR, "config")
IRIS_BRIDGE_TEMPLATE_DIR = os.path.join(CONFIG_TEMPLATE_DIR, "iris_bridge")
LAUNCH_TEMPLATE_DIR = os.path.join(TEMPLATES_DIR, "launch")
LAUNCH_ROBOTS_TEMPLATE_DIR = os.path.join(LAUNCH_TEMPLATE_DIR, "robots")
IRIS_RViz_CONFIG_TEMPLATE_DIR = os.path.join(TEMPLATES_DIR, "rviz")

# - define the created resources directories
CREATED_RESOURCES_DIR = os.path.join(os.path.dirname(os.path.dirname(SCRIPT_PATH)), "created_resources")
CREATED_MODELS_DIR = os.path.join(CREATED_RESOURCES_DIR, "models")
CREATED_WORLDS_DIR = os.path.join(CREATED_RESOURCES_DIR, "worlds")
CREATED_CONFIG_DIR = os.path.join(CREATED_RESOURCES_DIR, "config")
CREATED_IRIS_BRIDGE_DIR = os.path.join(CREATED_CONFIG_DIR, "iris_bridge")
CREATED_LAUNCH_DIR = os.path.join(CREATED_RESOURCES_DIR, "launch")
CREATERD_LAUNCH_ROBOTS_DIR = os.path.join(CREATED_LAUNCH_DIR, "robots")
CREATED_IRIS_RViz_CONFIG_DIR = os.path.join(CREATED_RESOURCES_DIR, "rviz")
CREATED_RVIZ_CONFIG_DIR = os.path.join(CREATED_RESOURCES_DIR, "rviz")
def init_created_resources_dir():
    """Create the created resources directory if it doesn't exist"""
    if not os.path.exists(CREATED_RESOURCES_DIR):
        os.makedirs(CREATED_RESOURCES_DIR)
    if not os.path.exists(CREATED_MODELS_DIR):
        os.makedirs(CREATED_MODELS_DIR)
    if not os.path.exists(CREATED_WORLDS_DIR):
        os.makedirs(CREATED_WORLDS_DIR)
    if not os.path.exists(CREATED_CONFIG_DIR):
        os.makedirs(CREATED_CONFIG_DIR)
    if not os.path.exists(CREATED_IRIS_BRIDGE_DIR):
        os.makedirs(CREATED_IRIS_BRIDGE_DIR)
    if not os.path.exists(CREATED_LAUNCH_DIR):
        os.makedirs(CREATED_LAUNCH_DIR)
    if not os.path.exists(CREATERD_LAUNCH_ROBOTS_DIR):
        os.makedirs(CREATERD_LAUNCH_ROBOTS_DIR)
    print('-------------------------------------------')
    print(f"Templates directory: {TEMPLATES_DIR}")
    print(f"Models template directory: {MODELS_TEMPLATE_DIR}")
    print(f"Worlds template directory: {WORLDS_TEMPLATE_DIR}")

    print('-------------------------------------------')
    print(f"Created resources directory: {CREATED_RESOURCES_DIR}")
    print(f"Created models directory: {CREATED_MODELS_DIR}")
    print(f"Created worlds directory: {CREATED_WORLDS_DIR}")



# 3- create n (DRONES_NUMBER) models in the (DRONE_MODELS_DIR)
def remove_directory(directory:str):
    """Remove a directory if it exists"""
    import shutil # used to remove a directory
    if os.path.exists(directory):
        shutil.rmtree(directory, ignore_errors=True)

def create_drone_models(drones_number:int):
    """Create n (DRONES_NUMBER) models in the (CREAATED_MODELS_DIR)"""

    model_template_dir = os.path.join(MODELS_TEMPLATE_DIR, "iris_with_gimbal")
    for i in range(1, drones_number + 1):
        # create new directory for each drone
        drone_model_dir = os.path.join(CREATED_MODELS_DIR, f"drone{i}")
        # remove the old directory if it exists and create a new one
        if os.path.exists(drone_model_dir):
            remove_directory(drone_model_dir)
        os.mkdir(drone_model_dir)
        # copy the template contents to the new directory
        os.system(f"cp -r {model_template_dir}/* {drone_model_dir}")
        # change the fdm_port_in in the sdf file
        with open(os.path.join(drone_model_dir, "model.sdf"), "r") as f:
            sdf_content = f.read()
            # fdm_port in e.x: <fdm_port_in>9012</fdm_port_in> (9012 is the default base value)
            # change the fdm_port_in to a unique value (starting from 9012 and incrementing by 10 for each drone)
            fdm_port_in = 9002 + ( (i-1) * 10)
            sdf_content = sdf_content.replace("<fdm_port_in>9012</fdm_port_in>", f"<fdm_port_in>{fdm_port_in}</fdm_port_in>")
            # save the new sdf file
            with open(os.path.join(drone_model_dir, "model.sdf"), "w") as f:
                f.write(sdf_content)
                f.close()
            f.close()
        
        print(f"Created drone{i} model at {drone_model_dir}")
        

            
def update_gazebo_world(drones_number:int, world_name = "iris_runway.sdf"):
    """Create a new world from the template world file, and add the drones to it"""
    # open the world file
    with open(os.path.join(WORLDS_TEMPLATE_DIR, world_name), "r") as world_file:
        world_content = world_file.read()

        # add the new drones to the world before the </world> tag
        # e.x: 
        # <include>
        #   <uri>model://iris_with_gimbal</uri>
        #   <name>iris</name>
        #   <pose degrees="true">0 0 0.195 0 0 90</pose>
        # </include>
        
        for i in range(1, drones_number + 1):
            #  TODO: return the value of i
            # adding the include tag before the </world> tag
            pose_degrees = f"{16 + (i * 3)} 0 0.195 0 0 90"
            world_content = world_content.replace("</world>", f"\n<include>\n   <uri>model://drone{i}</uri>\n   <name>drone{i}</name>\n   <pose degrees=\"true\">{pose_degrees}</pose>\n</include>\n\n</world>")
        # save the new world file
        with open(os.path.join(CREATED_WORLDS_DIR, world_name), "w") as new_world_file:
            new_world_file.write(world_content)
            # close the file
            new_world_file.close()
        # close the file
        world_file.close()
        print(f"World {world_name} created at {CREATED_WORLDS_DIR}")


def create_iriss_bridge(drones_number:int, iris_bridge_file_name = "iris_bridge.yaml"):
    """Create the iris bridge for each drone"""
    
    for i in range(1, drones_number + 1):
        # edit the iris_bridge.yaml template file
        # the file have two variables:
        # <namespace> and <iris_number>
        # <namespace> is the namespace of the drone
        # <iris_number> is the number of the drone
        # the file will be saved in the (CREATED_IRIS_BRIDGE_DIR)
        # the file will be named iris_bridge_<iris_number>.yaml

        with open(os.path.join(IRIS_BRIDGE_TEMPLATE_DIR, iris_bridge_file_name), "r") as iris_bridge_file:
            iris_bridge_content = iris_bridge_file.read()
            iris_bridge_content = iris_bridge_content.replace("<namespace>", f"drone{i}")
            iris_bridge_content = iris_bridge_content.replace("<drone_number>", f"{i}")
            # save the new iris_bridge.yaml file
            with open(os.path.join(CREATED_IRIS_BRIDGE_DIR, f"iris_bridge_{i}.yaml"), "w") as new_iris_bridge_file:
                new_iris_bridge_file.write(iris_bridge_content)
                # close the file
                new_iris_bridge_file.close()
            # close the file
            iris_bridge_file.close()
            print(f"Iris bridge {i} created at {CREATED_IRIS_BRIDGE_DIR}")

def create_iris_rviz_config(drones_number:int, iris_rviz_config_file_name = "iris.rviz"):
    """Create the iris rviz config for each drone"""
    for i in range(1, drones_number + 1):
        # edit the iris.rviz template file
        # the file have one variable:
        # {drone_instance}
        # <drone_instance> is the number of the drone
        # the file will be saved in the (CREATED_IRIS_RViz_CONFIG_DIR)
        # the file will be named iris_<drone_instance>.rviz
        with open(os.path.join(IRIS_RViz_CONFIG_TEMPLATE_DIR, iris_rviz_config_file_name), "r") as iris_rviz_config_file:
            iris_rviz_config_content = iris_rviz_config_file.read()
            iris_rviz_config_content = iris_rviz_config_content.replace("{drone_instance}", f"{i}")
            # save the new iris.rviz file
            with open(os.path.join(CREATED_IRIS_RViz_CONFIG_DIR, f"iris_{i}.rviz"), "w") as new_iris_rviz_config_file:
                new_iris_rviz_config_file.write(iris_rviz_config_content)
                # close the file
                new_iris_rviz_config_file.close()
            # close the file
            iris_rviz_config_file.close()
            print(f"Iris rviz config {i} created at {CREATED_IRIS_RViz_CONFIG_DIR}")


def create_rviz_cameras_config(drones_number: int, output_path = f"{CREATED_RVIZ_CONFIG_DIR}/rviz_cameras_config.rviz"):
    """
    Create RViz configuration file with camera views for multiple drones.
    
    Args:
        drones_number (int): Number of drones to configure
        output_path (str): Path where to save the configuration file
    """
    # Calculate window layout
    WINDOW_WIDTH = 1620
    WINDOW_HEIGHT = 965
    IMAGE_HEIGHT = int(WINDOW_HEIGHT * 0.4)  # 40% of window height for each image
    
    # Generate the configuration content
    config = f"""Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
      Splitter Ratio: 0.5
    Tree Height: 318
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5

Visualization Manager:
  Class: ""
  Displays:
"""

    # Add Image displays for each drone
    for i in range(1, drones_number + 1):
        config += f"""    - Class: rviz_default_plugins/Image
      Enabled: true
      Max Value: 1
      Median window: 5
      Min Value: 0
      Name: Drone{i}_Camera
      Normalize Range: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /drone{i}/camera/image
      Value: true
"""

    # Add global options and views
    config += """  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: drone1/base_link
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 10
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.8953976035118103
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0.4953981637954712
    Saved: ~"""

    # Calculate window geometry
    window_geometry = """
Window Geometry:
  Displays:
    collapsed: false
  Height: """ + str(WINDOW_HEIGHT) + """
  Hide Left Dock: false
  Hide Right Dock: false"""

    # Add geometry for each drone camera
    for i in range(1, drones_number + 1):
        window_geometry += f"""
  Drone{i}_Camera:
    collapsed: false"""

    # Add QMainWindow state and final geometry
    window_geometry += f"""
  QMainWindow State: 000000ff00000000fd00000004000000000000051600000352fc020000000b
  Width: {WINDOW_WIDTH}
  X: 249
  Y: 50"""

    config += window_geometry

    # Write the configuration to file
    try:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        with open(output_path, 'w') as f:
            f.write(config)
        print(f"RViz configuration file created successfully at: {output_path}")
    except Exception as e:
        print(f"Error creating RViz configuration file: {e}")
        raise

def calculate_image_layout(num_images: int, window_width: int, window_height: int):
    """
    Calculate the layout for multiple images in the window.
    Returns a list of (x, y, width, height) tuples for each image.
    """
    if num_images <= 0:
        return []

    # Calculate number of rows and columns
    num_cols = min(3, num_images)  # Max 3 columns
    num_rows = (num_images + num_cols - 1) // num_cols

    # Calculate image dimensions
    image_width = window_width // num_cols
    image_height = window_height // num_rows

    layouts = []
    for i in range(num_images):
        row = i // num_cols
        col = i % num_cols
        x = col * image_width
        y = row * image_height
        layouts.append((x, y, image_width, image_height))

    return layouts   



def main():
    print("Starting drone swarm auto setup")
    
    init_created_resources_dir()
    create_drone_models(DRONES_NUMBER)
    update_gazebo_world(DRONES_NUMBER)
    create_iriss_bridge(DRONES_NUMBER)
    create_iris_rviz_config(DRONES_NUMBER)
    create_rviz_cameras_config(DRONES_NUMBER)

    print("Drone swarm auto setup finished")

if __name__ == "__main__":
    main()
