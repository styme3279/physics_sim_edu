.. _Examples:

Examples
========

This section provides examples demonstrating how to use the Physics Simulator. All examples are based on actual working code from the examples directory.

Basic Usage
-----------

Setting Up a Basic Simulation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The most fundamental usage pattern for the Physics Simulator.

**File:** ``examples/basic_usage.py``

.. code-block:: python

   from physics_simulator import PhysicsSimulator
   from synthnova_config import PhysicsSimulatorConfig, RobotConfig
   from pathlib import Path

   def main():
       # Create simulator configuration
       config = PhysicsSimulatorConfig()
       physics_simulator = PhysicsSimulator(config)
       
       # Add default scene with ground plane
       physics_simulator.add_default_scene()

       # Add robot to the simulation
       robot_config = RobotConfig(
           prim_path="/World/Galbot",
           name="galbot_one_charlie",
           mjcf_path=Path()
               .joinpath(physics_simulator.synthnova_assets_directory)
               .joinpath("synthnova_assets")
               .joinpath("robot")
               .joinpath("galbot_one_charlie_description")
               .joinpath("galbot_one_charlie.xml"),
           position=[0, 0, 0],
           orientation=[0, 0, 0, 1]
       )
       robot_path = physics_simulator.add_robot(robot_config)

       # Initialize simulator - required before stepping
       physics_simulator.initialize()
       
       # Get simulation info
       init_state = physics_simulator.get_current_state()
       sim_time = physics_simulator.get_simulation_time()
       print(f"Simulation time: {sim_time}")

       # Run simulation loop
       physics_simulator.loop()
       
       # Clean up resources
       physics_simulator.close()

   if __name__ == "__main__":
       main()

Adding Entities to the Scene
----------------------------

Primitive Objects
^^^^^^^^^^^^^^^^^

Adding basic geometric shapes to create simple environments.

**File:** ``examples/add_entity/add_basic_geom.py``

.. code-block:: python

   from synthnova_config import PhysicsSimulatorConfig, CuboidConfig
   from physics_simulator import PhysicsSimulator
   from pathlib import Path

   def main():
       # Initialize simulator
       config = PhysicsSimulatorConfig()
       physics_simulator = PhysicsSimulator(config)
       physics_simulator.add_default_scene()

       # Add colored cubes
       cube_configs = [
           CuboidConfig(
               prim_path=Path(physics_simulator.root_prim_path).joinpath("cube_1"),
               position=[2, 2, 2],
               orientation=[0, 0, 0, 1],
               scale=[1, 1, 1],
               color=[1.0, 0.0, 0.0]  # Red cube
           ),
           CuboidConfig(
               prim_path=Path(physics_simulator.root_prim_path).joinpath("cube_2"),
               position=[0, 0, 2],
               orientation=[0, 0, 0, 1],
               scale=[1, 1, 1],
               color=[0.0, 1.0, 0.0]  # Green cube
           )
       ]

       # Add all cubes to the simulation
       for cube_config in cube_configs:
           physics_simulator.add_object(cube_config)

       physics_simulator.initialize()
       physics_simulator.loop()
       physics_simulator.close()

Complex Mesh Objects
^^^^^^^^^^^^^^^^^^^^^

Loading complex 3D models from MJCF files.

**File:** ``examples/add_entity/add_mesh.py``

.. code-block:: python

   from synthnova_config import PhysicsSimulatorConfig, MeshConfig
   from physics_simulator import PhysicsSimulator
   from pathlib import Path

   def main():
       config = PhysicsSimulatorConfig()
       physics_simulator = PhysicsSimulator(config)
       physics_simulator.add_default_scene()

       # Add shelf mesh object
       shelf_config = MeshConfig(
           prim_path="/World/Shelf",
           name="shelf",
           mjcf_path=Path()
               .joinpath(physics_simulator.synthnova_assets_directory)
               .joinpath("synthnova_assets")
               .joinpath("default")
               .joinpath("shelves")
               .joinpath("1")
               .joinpath("model")
               .joinpath("mjcf")
               .joinpath("convex_decomposition.xml"),
           position=[0.55, 0, 0],
           orientation=[0, 0, 0, 1]
       )
       physics_simulator.add_object(shelf_config)

       physics_simulator.initialize()
       physics_simulator.loop()
       physics_simulator.close()

GalbotInterface Robot Control
-----------------------------

The GalbotInterface provides high-level, modular control for Galbot robots.

Arm Control
^^^^^^^^^^^

Controlling robot arms with position control and trajectory following.

**File:** ``examples/galbot_interface_examples/basic/left_arm.py``

.. code-block:: python

   from physics_simulator import PhysicsSimulator
   from physics_simulator.galbot_interface import GalbotInterface, GalbotInterfaceConfig
   from physics_simulator.utils.data_types import JointTrajectory
   from synthnova_config import PhysicsSimulatorConfig, RobotConfig
   import numpy as np
   from pathlib import Path

   def interpolate_joint_positions(start_positions, end_positions, steps):
       """Create smooth interpolation between joint positions."""
       return np.linspace(start_positions, end_positions, steps)

   def main():
       # Setup simulator and robot
       config = PhysicsSimulatorConfig()
       physics_simulator = PhysicsSimulator(config)
       physics_simulator.add_default_scene()
       
       robot_config = RobotConfig(
           prim_path="/World/Galbot",
           name="galbot_one_charlie",
           mjcf_path=Path()
               .joinpath(physics_simulator.synthnova_assets_directory)
               .joinpath("synthnova_assets")
               .joinpath("robot")
               .joinpath("galbot_one_charlie_description")
               .joinpath("galbot_one_charlie.xml"),
           position=[0, 0, 0],
           orientation=[0, 0, 0, 1]
       )
       robot_path = physics_simulator.add_robot(robot_config)
       physics_simulator.initialize()

       # Configure GalbotInterface for left arm
       galbot_config = GalbotInterfaceConfig()
       galbot_config.modules_manager.enabled_modules.append("left_arm")
       galbot_config.left_arm.joint_names = [
           f"{robot_config.name}/left_arm_joint1",
           f"{robot_config.name}/left_arm_joint2",
           f"{robot_config.name}/left_arm_joint3",
           f"{robot_config.name}/left_arm_joint4",
           f"{robot_config.name}/left_arm_joint5",
           f"{robot_config.name}/left_arm_joint6",
           f"{robot_config.name}/left_arm_joint7",
       ]
       galbot_config.robot.prim_path = robot_path
       
       galbot_interface = GalbotInterface(galbot_config, physics_simulator)
       galbot_interface.initialize()

       # Control arm movement
       current_positions = galbot_interface.left_arm.get_joint_positions()
       target_positions = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
       
       # Create trajectory
       positions = interpolate_joint_positions(current_positions, target_positions, 500)
       trajectory = JointTrajectory(positions=positions)
       galbot_interface.left_arm.follow_trajectory(trajectory)

       physics_simulator.loop()
       physics_simulator.close()

**Similar examples available for other arms:**

- ``examples/galbot_interface_examples/basic/right_arm.py`` - Right arm control
- ``examples/galbot_interface_examples/basic/head.py`` - Head joint control
- ``examples/galbot_interface_examples/basic/leg.py`` - Leg joint control
- ``examples/galbot_interface_examples/basic/chassis.py`` - Chassis control

Gripper Control
^^^^^^^^^^^^^^^

Controlling robot grippers with open/close commands.

**File:** ``examples/galbot_interface_examples/basic/left_gripper.py``

.. code-block:: python

   from physics_simulator import PhysicsSimulator
   from physics_simulator.galbot_interface import GalbotInterface, GalbotInterfaceConfig
   from synthnova_config import PhysicsSimulatorConfig, RobotConfig
   from pathlib import Path

   def main():
       # Setup simulator and robot
       config = PhysicsSimulatorConfig()
       physics_simulator = PhysicsSimulator(config)
       physics_simulator.add_default_scene()

       robot_config = RobotConfig(
           prim_path="/World/Galbot",
           name="galbot_one_charlie",
           mjcf_path=Path()
               .joinpath(physics_simulator.synthnova_assets_directory)
               .joinpath("synthnova_assets")
               .joinpath("robot")
               .joinpath("galbot_one_charlie_description")
               .joinpath("galbot_one_charlie.xml"),
           position=[0, 0, 0],
           orientation=[0, 0, 0, 1]
       )
       robot_path = physics_simulator.add_robot(robot_config)
       physics_simulator.initialize()

       # Configure gripper interface
       galbot_config = GalbotInterfaceConfig()
       galbot_config.modules_manager.enabled_modules.append("left_gripper")
       galbot_config.left_gripper.joint_names = [
           f"{robot_config.name}/left_gripper_robotiq_85_right_knuckle_joint"
       ]
       galbot_config.robot.prim_path = robot_path
       
       galbot_interface = GalbotInterface(galbot_config, physics_simulator)
       galbot_interface.initialize()

       # Control gripper
       galbot_interface.left_gripper.set_gripper_close()

       physics_simulator.loop()
       physics_simulator.close()

**Available gripper examples:**

- ``examples/galbot_interface_examples/basic/right_gripper.py`` - Right gripper control

Camera and Sensor Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Accessing RGB, depth, and point cloud data from robot-mounted cameras.

**File:** ``examples/galbot_interface_examples/sensors/front_head_camera.py``

.. code-block:: python

   from physics_simulator import PhysicsSimulator
   from synthnova_config import (
       PhysicsSimulatorConfig, RobotConfig, 
       RgbCameraConfig, RealsenseD435RgbSensorConfig,
       DepthCameraConfig, RealsenseD435DepthSensorConfig
   )
   from physics_simulator.galbot_interface import GalbotInterface, GalbotInterfaceConfig
   from physics_simulator.utils import preprocess_depth
   import os
   import numpy as np
   import cv2
   from pathlib import Path

   def main():
       # Setup simulator
       config = PhysicsSimulatorConfig()
       physics_simulator = PhysicsSimulator(config)
       physics_simulator.add_default_scene()

       # Add robot
       robot_config = RobotConfig(
           prim_path="/World/Galbot",
           name="galbot_one_charlie",
           mjcf_path=Path()
               .joinpath(physics_simulator.synthnova_assets_directory)
               .joinpath("synthnova_assets")
               .joinpath("robot")
               .joinpath("galbot_one_charlie_description")
               .joinpath("galbot_one_charlie.xml"),
           position=[0, 0, 0],
           orientation=[0, 0, 0, 1]
       )
       robot_path = physics_simulator.add_robot(robot_config)

       # Add RGB camera
       rgb_camera_config = RgbCameraConfig(
           name="front_head_rgb_camera",
           prim_path=os.path.join(
               robot_path, "head_link2", "head_end_effector_mount_link", 
               "front_head_rgb_camera"
           ),
           translation=[0.09321, -0.06166, 0.033],
           rotation=[0.683012701855461, 0.1830127020294028, 
                    0.18301270202940284, 0.6830127018554611],
           sensor_config=RealsenseD435RgbSensorConfig(),
           parent_entity_name="galbot_one_charlie/head_end_effector_mount_link"
       )
       rgb_camera_path = physics_simulator.add_sensor(rgb_camera_config)

       # Add depth camera
       depth_camera_config = DepthCameraConfig(
           name="front_head_depth_camera",
           prim_path=os.path.join(
               robot_path, "head_link2", "head_end_effector_mount_link",
               "front_head_depth_camera"
           ),
           translation=[0.09321, -0.06166, 0.033],
           rotation=[0.683012701855461, 0.1830127020294028,
                    0.18301270202940284, 0.6830127018554611],
           sensor_config=RealsenseD435DepthSensorConfig(),
           parent_entity_name="galbot_one_charlie/head_end_effector_mount_link"
       )
       depth_camera_path = physics_simulator.add_sensor(depth_camera_config)

       # Configure camera interface
       galbot_config = GalbotInterfaceConfig()
       galbot_config.modules_manager.enabled_modules.append("front_head_camera")
       galbot_config.robot.prim_path = robot_path
       galbot_config.front_head_camera.prim_path_rgb = rgb_camera_path
       galbot_config.front_head_camera.prim_path_depth = depth_camera_path
       
       galbot_interface = GalbotInterface(galbot_config, physics_simulator)
       galbot_interface.initialize()

       # Start simulation
       physics_simulator.play()
       physics_simulator.step(10)

       while True:
           physics_simulator.step(7)
           
           # Get sensor data
           rgb_data = galbot_interface.front_head_camera.get_rgb()
           depth_data = galbot_interface.front_head_camera.get_depth()
           
           # Process depth data for visualization
           depth_data = preprocess_depth(
               depth_data,
               scale=1000,      # Convert m to mm
               min_value=0.0,
               max_value=3 * 1000,  # 3m to mm
               data_type=np.uint16,
           )
           
           # Display images
           cv2.imshow("RGB Camera", cv2.cvtColor(rgb_data, cv2.COLOR_RGB2BGR))
           cv2.imshow("Depth Camera", depth_data)

           if cv2.waitKey(1) & 0xFF == ord("q"):
               cv2.destroyAllWindows()
               break

       # Get camera parameters
       params = galbot_interface.front_head_camera.get_parameters()
       intrinsic_matrix = params["rgb"]["intrinsic_matrix"]
       print("Camera parameters:", params)
       print("Intrinsic matrix:", intrinsic_matrix)

       physics_simulator.close()

**Available camera examples:**

- ``examples/galbot_interface_examples/sensors/left_wrist_camera.py`` - Left wrist camera
- ``examples/galbot_interface_examples/sensors/right_wrist_camera.py`` - Right wrist camera

IOAI Demo Examples
-------------------

The examples directory also contains other specialized examples:

- ``examples/ioai/`` - IOAI demo examples
    - ``examples/ioai/ioai_grasp_demo.py`` - Basic pick and place demo using a simple state machine
    - ``examples/ioai/ioai_nav_demo.py`` - Navigation demo using A* pathfinding algorithm

IOAI Grasp Demo
^^^^^^^^^^^^^^^

This example demonstrates a basic pick and place task using a simple state machine. The robot performs the following sequence:

1. Move to a pre-grasp position
2. Open the gripper
3. Move to the object
4. Close the gripper to grasp the object
5. Lift the object
6. Move to the target position
7. Open the gripper to release the object

.. raw:: html

    <video width="640" height="360" controls>
        <source src="../_static/videos/ioai_grasp_demo.mp4" type="video/mp4">
        Your browser does not support the video tag.
    </video>

IOAI Navigation Demo
^^^^^^^^^^^^^^^^^^^^

This example showcases autonomous navigation using the A* pathfinding algorithm. The robot:

1. Receives a target position
2. Plans a path using A* algorithm
3. Follows the planned path while avoiding obstacles
4. Reaches the target position

.. raw:: html

    <video width="640" height="360" controls>
        <source src="../_static/videos/ioai_nav_demo.mp4" type="video/mp4">
        Your browser does not support the video tag.
    </video>

