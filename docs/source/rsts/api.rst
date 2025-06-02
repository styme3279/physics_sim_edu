.. _API:

API Reference
=============

This section provides comprehensive documentation for all public classes and methods in the Physics Simulator.

PhysicsSimulator (MujocoSimulator)
----------------------------------

The main simulation class that provides the primary interface to the MuJoCo physics engine.

**Import:**

.. code-block:: python

   from physics_simulator import PhysicsSimulator
   from synthnova_config import PhysicsSimulatorConfig

**Initialization:**

.. code-block:: python

   config = PhysicsSimulatorConfig()
   sim = PhysicsSimulator(config)

Core Simulation Methods
^^^^^^^^^^^^^^^^^^^^^^^

**initialize()**
   Initialize the MuJoCo model, data structures, and viewer. Must be called before stepping the simulation.

   .. code-block:: python

      sim.initialize()

**step(num_steps: int = 1, render: bool = True)**
   Advance the simulation by the specified number of timesteps.

   :param num_steps: Number of timesteps to advance (default: 1)
   :param render: Rendering flag (ignored in MuJoCo implementation)

   .. code-block:: python

      sim.step(10)  # Advance 10 timesteps

**forward()**
   Run forward dynamics to synchronize derived quantities without advancing time.

   .. code-block:: python

      sim.forward()

**step1()** / **step2()**
   Execute first/second part of simulation step for advanced control.

**loop()**
   Run continuous simulation loop until interrupted.

   .. code-block:: python

      sim.loop()  # Run until Ctrl+C

**play()**
   Start the simulation if not already running (calls initialize() if needed).

**reset()**
   Reset simulation to initial state.

**close()**
   Clean up and release all resources.

   .. code-block:: python

      sim.close()

**is_running() -> bool**
   Check if simulator is currently running.

**render(width=None, height=None, *, camera_name=None, depth=False, mode="offscreen", device_id=-1, segmentation=False) -> np.ndarray**
   Render the current scene from a specific camera view.
   
   :param width: Image width in pixels
   :param height: Image height in pixels
   :param camera_name: Name of the camera to render from
   :param depth: Whether to render depth information
   :param mode: Rendering mode ('offscreen' or 'window')
   :param device_id: GPU device ID to use for rendering
   :param segmentation: Whether to render segmentation mask
   :returns: The rendered image as numpy array

   .. code-block:: python

      # Render from specific camera
      image = sim.render(width=640, height=480, camera_name="front_camera")

State and Information Methods
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**get_current_state() -> dict**
   Get complete simulation state including all robots and objects.

   :returns: Dictionary with keys: 'robots', 'objects', 'physics_params', 'time'

   .. code-block:: python

      state = sim.get_current_state()
      print(f"Simulation time: {state['time']}")

**get_simulation_time() -> float**
   Get current simulation time in seconds.

**get_step_count() -> int**
   Get total number of simulation steps executed.

**get_physics_dt() -> float**
   Get physics simulation timestep.

Scene Management
^^^^^^^^^^^^^^^^

**add_default_scene(ground_plane_config: GroundPlaneConfig = None)**
   Add default ground plane to the scene.

   .. code-block:: python

      sim.add_default_scene()

Entity Management - Robots
^^^^^^^^^^^^^^^^^^^^^^^^^^^

**add_robot(robot_config: RobotConfig) -> str**
   Add a robot to the simulation.

   :param robot_config: Robot configuration object
   :returns: Primary path of the added robot

   .. code-block:: python

      from synthnova_config import RobotConfig
      
      robot_config = RobotConfig(
          prim_path="/World/Robot",
          name="galbot_one_charlie",
          mjcf_path="path/to/robot.xml",
          position=[0, 0, 0],
          orientation=[0, 0, 0, 1]
      )
      robot_path = sim.add_robot(robot_config)

**get_robot(prim_path: str) -> MujocoRobot**
   Retrieve robot instance by path.

   .. code-block:: python

      robot = sim.get_robot("/World/Robot")

**get_robot_state(prim_path: str) -> dict**
   Get complete state of a robot.

   :returns: Dictionary with keys: 'position', 'orientation', 'joint_positions', 'joint_velocities'

**remove_robot(prim_path: str)**
   Remove robot from simulation.

Entity Management - Objects
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**add_object(object_config: ObjectConfig) -> str**
   Add an object to the simulation.

   .. code-block:: python

      from synthnova_config import CuboidConfig
      
      cube_config = CuboidConfig(
          prim_path="/World/Cube",
          position=[1, 1, 1],
          scale=[0.5, 0.5, 0.5],
          color=[1.0, 0.0, 0.0]
      )
      cube_path = sim.add_object(cube_config)

**get_object(prim_path: str)**
   Get object instance by path.

**get_object_state(prim_path: str) -> dict**
   Get object state including position and orientation.

**remove_object(prim_path: str)**
   Remove object from simulation.

Entity Management - Sensors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**add_sensor(sensor_config: SensorConfig) -> str**
   Add a sensor (camera) to the simulation.

   .. code-block:: python

      from synthnova_config import RgbCameraConfig, RealsenseD435RgbSensorConfig
      
      camera_config = RgbCameraConfig(
          prim_path="/World/Camera",
          name="rgb_camera",
          translation=[0.1, 0, 0.1],
          rotation=[0, 0, 0, 1],
          sensor_config=RealsenseD435RgbSensorConfig(),
          parent_entity_name="galbot_one_charlie/head_link"
      )
      camera_path = sim.add_sensor(camera_config)

**get_sensor(prim_path: str)**
   Get sensor instance by path.

**get_sensor_state(prim_path: str) -> dict**
   Get sensor data and state.

**remove_sensor(prim_path: str)**
   Remove sensor from simulation.

Joint Control Methods
^^^^^^^^^^^^^^^^^^^^^

**set_joint_positions(robot, joint_positions: List[float], joint_names: List[str], immediate: bool = False)**
   Set joint positions for a robot.

   :param robot: Robot instance
   :param joint_positions: List of target positions
   :param joint_names: List of joint names
   :param immediate: Apply immediately without interpolation

**get_joint_positions(robot, joint_names: List[str]) -> List[float]**
   Get current joint positions.

**get_joint_velocities(joint_names: List[str] = None) -> List[float]**
   Get current joint velocities.

Physics Callbacks
^^^^^^^^^^^^^^^^^^

**add_physics_callback(name: str, callback_fn: callable)**
   Register a callback function to be executed during each simulation step.

   .. code-block:: python

      def my_callback():
          # Custom physics behavior
          pass
      
      sim.add_physics_callback("my_callback", my_callback)

**remove_physics_callback(name: str)**
   Remove a physics callback.

**physics_callback_exists(name: str) -> bool**
   Check if a callback exists.

Scenario Management
^^^^^^^^^^^^^^^^^^^

**import_scenario(scenario_config: ScenarioConfig)**
   Import a complete scenario configuration.

**export_scenario(file_path: str, scenario_name: str = None, scenario_description: str = None)**
   Export current simulation state to a scenario file.

Properties
^^^^^^^^^^

**root_directory: str**
   Root directory path for assets.

**synthnova_assets_directory: str**
   SynthNova assets directory path.

**root_prim_path: str**
   Root primitive path ("/World").

MujocoRobot
-----------

Robot control interface for individual robots in the simulation.

State Query Methods
^^^^^^^^^^^^^^^^^^^

**get_joint_names() -> List[str]**
   Get all joint names defined in the robot model.

**get_joint_positions(joint_names: List[str] = None) -> np.ndarray**
   Get current joint positions.

   :param joint_names: Specific joints to query (default: all joints)
   :returns: Array of joint positions

**get_joint_velocities(joint_names: List[str] = None) -> np.ndarray**
   Get current joint velocities.

**get_world_pose() -> Tuple[np.ndarray, np.ndarray]**
   Get robot's position and orientation in world coordinates.

   :returns: (position [x,y,z], orientation [qx,qy,qz,qw])

**get_state() -> dict**
   Get complete robot state.

   :returns: Dictionary with position, orientation, joint_positions, joint_velocities

**get_position() -> np.ndarray**
   Get robot position in world frame.

**get_orientation() -> np.ndarray**
   Get robot orientation as quaternion.

**get_all_joint_positions() -> np.ndarray**
   Get all joint positions from the robot model.

**get_all_joint_velocities() -> np.ndarray**
   Get all joint velocities from the robot model.

Control Methods
^^^^^^^^^^^^^^^

**set_joint_positions(positions: List[float], joint_names: List[str], immediate: bool = False)**
   Set target joint positions.

   .. code-block:: python

      robot.set_joint_positions([0.1, 0.2, 0.3], ["joint1", "joint2", "joint3"])

**set_joint_velocities(velocities: List[float], joint_names: List[str], immediate: bool = False)**
   Set joint velocities.

**set_world_pose(pose: np.ndarray)**
   Set robot pose in world coordinates.

   :param pose: Array [x, y, z, qx, qy, qz, qw]

Forward Kinematics
^^^^^^^^^^^^^^^^^^

**fk_link(q: np.ndarray, link: str) -> Tuple[np.ndarray, np.ndarray]**
   Compute forward kinematics for a specific link.

   :param q: Joint configuration
   :param link: Link name
   :returns: (position, rotation_matrix)

**fk_all_link(q: np.ndarray) -> Tuple[Dict[str, np.ndarray], Dict[str, np.ndarray]]**
   Compute forward kinematics for all links.

   :returns: (positions_dict, rotations_dict)

**fk_jacobian(q: np.ndarray, link: str) -> np.ndarray**
   Compute Jacobian matrix for a link.

   :returns: 6xN Jacobian matrix (3 translation + 3 rotation)

Utility Methods
^^^^^^^^^^^^^^^

**generate_random_qpos() -> np.ndarray**
   Generate random valid joint configuration within limits.

**clip_qpos(q: np.ndarray) -> np.ndarray**
   Clip joint positions to valid limits.

**get_articulation_controller()**
   Get low-level articulation controller.

**apply_action(action)**
   Apply articulation action with joint positions/velocities.

GalbotInterface
---------------

High-level modular interface for controlling Galbot robots.

Initialization
^^^^^^^^^^^^^^

.. code-block:: python

   from physics_simulator.galbot_interface import GalbotInterface, GalbotInterfaceConfig
   
   # Configure interface
   config = GalbotInterfaceConfig()
   config.modules_manager.enabled_modules = ["left_arm", "left_gripper"]
   config.robot.prim_path = robot_path
   
   # Initialize interface
   galbot = GalbotInterface(config, simulator)
   galbot.initialize()

Available Modules
^^^^^^^^^^^^^^^^^

**BasicLimb Modules:**
   - ``chassis`` - Mobile base control
   - ``left_arm`` - Left arm joint control
   - ``right_arm`` - Right arm joint control  
   - ``head`` - Head joint control
   - ``leg`` - Leg joint control

**BasicGripper Modules:**
   - ``left_gripper`` - Left gripper control
   - ``right_gripper`` - Right gripper control

**BasicCamera Modules:**
   - ``left_wrist_camera`` - Left wrist camera interface
   - ``right_wrist_camera`` - Right wrist camera interface
   - ``front_head_camera`` - Front head camera interface

BasicLimb Methods
^^^^^^^^^^^^^^^^^

All arm, chassis, head, and leg modules inherit from BasicLimb.

**get_joint_positions() -> np.ndarray**
   Get current joint positions for this module.

**set_joint_positions(joint_positions: List[float], immediate: bool = False)**
   Set joint positions for this module.

**follow_trajectory(joint_trajectory: JointTrajectory)**
   Execute a joint trajectory.

   .. code-block:: python

      from physics_simulator.utils.data_types import JointTrajectory
      import numpy as np
      
      # Create trajectory
      start_pos = galbot.left_arm.get_joint_positions()
      end_pos = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
      positions = np.linspace(start_pos, end_pos, 100)
      trajectory = JointTrajectory(positions=positions)
      
      # Execute trajectory
      galbot.left_arm.follow_trajectory(trajectory)

**get_joint_names() -> List[str]**
   Get joint names for this module.

BasicGripper Methods
^^^^^^^^^^^^^^^^^^^^

Gripper modules inherit from BasicLimb and add gripper-specific functionality.

**set_gripper_open()**
   Open the gripper to maximum width.

**set_gripper_close()**
   Close the gripper completely.

**set_gripper_width(normalized_width: float)**
   Set gripper width using normalized value.

   :param normalized_width: Value between 0.0 (closed) and 1.0 (open)

   .. code-block:: python

      galbot.left_gripper.set_gripper_width(0.5)  # Half open

**is_open -> bool**
   Property indicating if gripper is open.

**is_close -> bool**
   Property indicating if gripper is closed.

BasicCamera Methods
^^^^^^^^^^^^^^^^^^^

Camera modules provide access to RGB, depth, and segmentation data.

**get_rgb() -> np.ndarray**
   Get RGB image data.

   :returns: RGB image array (H, W, 3)

**get_depth() -> np.ndarray**
   Get depth image data.

   :returns: Depth image array (H, W)

**get_segmentation() -> np.ndarray**
   Get segmentation mask.

   :returns: Segmentation image array

**get_point_cloud_wrt_robot() -> np.ndarray**
   Generate point cloud in robot coordinate frame.

   :returns: Point cloud array (N, 3)

**get_pose_wrt_robot() -> Tuple[np.ndarray, np.ndarray]**
   Get camera pose relative to robot.

   :returns: (position, orientation)

**get_parameters() -> dict**
   Get camera intrinsic parameters.

   :returns: Dictionary with 'rgb' and 'depth' parameter sets

   .. code-block:: python

      params = galbot.front_head_camera.get_parameters()
      intrinsic_matrix = params["rgb"]["intrinsic_matrix"]

Configuration Classes
---------------------

Configuration objects define simulation parameters and entity properties.

PhysicsSimulatorConfig
^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   from synthnova_config import PhysicsSimulatorConfig, MujocoConfig
   
   config = PhysicsSimulatorConfig(
       mujoco_config=MujocoConfig(
           headless=False,
           timestep=0.01,
           gravity=[0, 0, -9.81],
           solver="Newton",
           iterations=20,
           tolerance=1e-6
       )
   )

RobotConfig
^^^^^^^^^^^

.. code-block:: python

   RobotConfig(
       prim_path="/World/Robot",
       name="robot_name",
       mjcf_path="path/to/robot.xml",
       position=[0, 0, 0],
       orientation=[0, 0, 0, 1]
   )

Object Configuration
^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   # Primitive objects
   CuboidConfig(
       prim_path="/World/Box",
       position=[0, 0, 1],
       orientation=[0, 0, 0, 1],
       scale=[1, 1, 1],
       color=[1.0, 0.0, 0.0]
   )
   
   # Mesh objects
   MeshConfig(
       prim_path="/World/Object",
       name="object_name",
       mjcf_path="path/to/object.xml",
       position=[0, 0, 0],
       orientation=[0, 0, 0, 1]
   )

Sensor Configuration
^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   RgbCameraConfig(
       prim_path="/World/Camera",
       name="camera_name",
       translation=[0.1, 0, 0.1],
       rotation=[0, 0, 0, 1],
       sensor_config=RealsenseD435RgbSensorConfig(width=640, height=480),
       parent_entity_name="robot_name/link_name"
   )

GalbotInterfaceConfig
^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   config = GalbotInterfaceConfig()
   config.modules_manager.enabled_modules = ["left_arm", "left_gripper"]
   config.robot.prim_path = "/World/Robot"
   config.left_arm.joint_names = [
       "robot_name/left_arm_joint1",
       "robot_name/left_arm_joint2",
       # ... additional joints
   ]

Data Types
----------

JointTrajectory
^^^^^^^^^^^^^^^

.. code-block:: python

   from physics_simulator.utils.data_types import JointTrajectory
   import numpy as np
   
   # Create trajectory with waypoints
   positions = np.array([
       [0.0, 0.0, 0.0],  # First waypoint
       [0.5, 0.5, 0.5],  # Second waypoint
       [1.0, 1.0, 1.0]   # Final waypoint
   ])
   trajectory = JointTrajectory(positions=positions)

Error Handling
--------------

**Common Exceptions:**

- ``RuntimeError``: Module not initialized, simulation not running
- ``ValueError``: Invalid configuration parameters, joint names not found
- ``KeyError``: Entity not found at specified path

**Best Practices:**

.. code-block:: python

   try:
       sim.initialize()
       galbot.initialize()
       # Simulation code
   except RuntimeError as e:
       print(f"Initialization error: {e}")
   finally:
       sim.close()