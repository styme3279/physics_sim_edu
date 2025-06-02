#####################################################################################
# Copyright (c) 2023-2025 Galbot. All Rights Reserved.
#
# This software contains confidential and proprietary information of Galbot, Inc.
# ("Confidential Information"). You shall not disclose such Confidential Information
# and shall use it only in accordance with the terms of the license agreement you
# entered into with Galbot, Inc.
#
# UNAUTHORIZED COPYING, USE, OR DISTRIBUTION OF THIS SOFTWARE, OR ANY PORTION OR
# DERIVATIVE THEREOF, IS STRICTLY PROHIBITED. IF YOU HAVE RECEIVED THIS SOFTWARE IN
# ERROR, PLEASE NOTIFY GALBOT, INC. IMMEDIATELY AND DELETE IT FROM YOUR SYSTEM.
#####################################################################################
#          _____             _   _       _   _
#         / ____|           | | | |     | \ | |
#        | (___  _   _ _ __ | |_| |__   |  \| | _____   ____ _
#         \___ \| | | | '_ \| __| '_ \  | . ` |/ _ \ \ / / _` |
#         ____) | |_| | | | | |_| | | | | |\  | (_) \ V / (_| |
#        |_____/ \__, |_| |_|\__|_| |_| |_| \_|\___/ \_/ \__,_|
#                 __/ |
#                |___/
#
#####################################################################################
#
# Description: Example of modifying SynthNova configuration
# Author: Herman Ye@Galbot
# Date: 2025-03-03
#
#####################################################################################


def modify_simulator_config():
    """Modify and customize simulator configuration settings.

    This function demonstrates how to create and customize simulator configurations by:
    1. Creating a default RenderSimulatorConfig configuration
    2. Creating and customizing an Isaac Sim configuration
    3. Updating the simulator settings within RenderSimulatorConfig

    Modified parameters include:
    - Display resolution (1920x1080)
    - Physics update rate (120 Hz)
    - Rendering frame rate (60 Hz)

    Returns:
        None: The function prints the modified configuration but does not return any value.

    Example:
        >>> modify_simulator_config()
        Modified render simulator config: {...}
    """
    from synthnova_config import RenderSimulatorConfig
    from synthnova_config import IsaacSimConfig

    # Initialize Isaac Sim configuration with default values
    isaac_sim_config = IsaacSimConfig()

    # Customize simulator display and performance settings
    isaac_sim_config.width = 1920  # Set display width
    isaac_sim_config.height = 1080  # Set display height
    isaac_sim_config.physics_hz = 120  # Set physics update frequency
    isaac_sim_config.rendering_hz = 60  # Set visual frame rate

    # Update the simulator configuration in the main config
    render_simulator_config = RenderSimulatorConfig(isaac_sim_config=isaac_sim_config)
    print(f"Modified render simulator config: {render_simulator_config.model_dump()}\n")


def modify_camera_config():
    """Modify and customize camera configuration settings.

    This function demonstrates how to configure camera settings by:
    1. Configuring a RealSense D405 RGB sensor with custom intrinsics
    2. Creating a camera configuration with specific mounting parameters

    The camera is configured with:
    - Custom intrinsic parameters:
        * fx, fy: 616.282 (focal length in x and y directions)
        * cx: 320.0 (principal point x-coordinate)
        * cy: 240.0 (principal point y-coordinate)
    - Resolution: 640x360 pixels
    - Specific mounting position and orientation relative to the robot

    Returns:
        None: The function prints the modified configuration but does not return any value.

    Example:
        >>> modify_camera_config()
        Modified camera config: {...}
    """
    from synthnova_config import RgbCameraConfig
    from synthnova_config import RealsenseD405RgbSensorConfig

    # Configure RealSense D405 RGB sensor with custom intrinsic parameters
    my_d405_rgb_sensor_config = RealsenseD405RgbSensorConfig()
    my_d405_rgb_sensor_config.fx = 616.282  # Focal length in x-direction
    my_d405_rgb_sensor_config.fy = 616.282  # Focal length in y-direction
    my_d405_rgb_sensor_config.cx = 320.0  # Principal point x-coordinate
    my_d405_rgb_sensor_config.cy = 240.0  # Principal point y-coordinate
    my_d405_rgb_sensor_config.width = 640  # Image width in pixels
    my_d405_rgb_sensor_config.height = 360  # Image height in pixels

    # Create camera configuration with mounting specifications
    camera_config = RgbCameraConfig(
        name="left_arm_rgb_camera",
        # Full path to camera in the scene graph
        prim_path="/World/Robot/left_arm_link7/left_arm_end_effector_mount_link/left_arm_rgb_camera",
        # Camera position relative to mount point (in meters)
        translation=[-0.02255, 0.0, 0.05178],
        # Camera orientation as quaternion [x, y, z, w]
        rotation=[0.0, -0.04361938736547762, 0.0, 0.9990482215818516],
        sensor_config=my_d405_rgb_sensor_config,
    )

    print(f"Modified camera config: {camera_config.model_dump()}\n")


def main():
    """Execute the configuration modification examples.

    This function serves as the entry point and demonstrates the usage of:
    - Simulator configuration modification
    - Camera configuration modification
    """
    modify_simulator_config()
    modify_camera_config()


if __name__ == "__main__":
    main()
