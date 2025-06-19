"""
Integration tests for the physics simulator.
Tests the full workflow from initialization to simulation.
"""
import pytest
import numpy as np
from unittest.mock import Mock, patch, MagicMock
from physics_simulator import PhysicsSimulator
from synthnova_config import PhysicsSimulatorConfig, RobotConfig


@pytest.mark.integration
class TestPhysicsSimulatorIntegration:
    """Integration test suite for the complete physics simulator workflow."""

    @patch('physics_simulator.simulator.mujoco.PathManager')
    @patch('physics_simulator.simulator.mujoco.signal')
    @patch('physics_simulator.simulator.mujoco.MujocoSimulator._load_logger')
    @patch('physics_simulator.simulator.mujoco.MujocoSimulator._load_simulator')
    def test_simulator_full_lifecycle(self, mock_load_sim, mock_load_logger, mock_signal, mock_path_manager, basic_config):
        """Test complete simulator lifecycle from init to close."""
        # Setup mocks
        mock_path_manager.get_root_path.return_value = "/mock/path"
        mock_load_logger.return_value = Mock()
        mock_load_sim.return_value = Mock()
        
        # Initialize simulator
        simulator = PhysicsSimulator(basic_config)
        
        # Test initial state
        assert not simulator.is_running()
        assert simulator._robots == {}
        assert simulator._objects == {}
        
        # Test cleanup
        simulator.close()
        assert not simulator._running

    @patch('physics_simulator.simulator.mujoco.PathManager')
    @patch('physics_simulator.simulator.mujoco.signal')
    @patch('physics_simulator.simulator.mujoco.MujocoSimulator._load_logger')
    @patch('physics_simulator.simulator.mujoco.MujocoSimulator._load_simulator')
    def test_add_robot_and_simulation_step(self, mock_load_sim, mock_load_logger, mock_signal, mock_path_manager, basic_config, robot_config):
        """Test adding robot and running simulation steps."""
        # Setup mocks
        mock_path_manager.get_root_path.return_value = "/mock/path"
        mock_load_logger.return_value = Mock()
        mock_load_sim.return_value = Mock()
        
        simulator = PhysicsSimulator(basic_config)
        simulator.world = Mock()
        
        # Mock robot creation
        with patch('physics_simulator.robot.MujocoRobot') as mock_robot_class:
            mock_robot = Mock()
            mock_robot_class.return_value = mock_robot
            
            # Add robot
            simulator.add_robot(robot_config)
            
            # Verify robot was added
            assert robot_config.prim_path in simulator._robots
            
            # Test simulation stepping
            simulator._running = True
            simulator.model = Mock()
            simulator.model._model = Mock()
            simulator.data = Mock()
            simulator.data._data = Mock()
            
            with patch('mujoco.mj_step') as mock_step:
                simulator.step(num_steps=5)
                assert mock_step.call_count == 5

    @patch('physics_simulator.simulator.mujoco.PathManager')
    @patch('physics_simulator.simulator.mujoco.signal')
    @patch('physics_simulator.simulator.mujoco.MujocoSimulator._load_logger')
    @patch('physics_simulator.simulator.mujoco.MujocoSimulator._load_simulator')
    def test_robot_state_retrieval(self, mock_load_sim, mock_load_logger, mock_signal, mock_path_manager, basic_config, robot_config):
        """Test retrieving robot state after adding robot."""
        # Setup mocks
        mock_path_manager.get_root_path.return_value = "/mock/path"
        mock_load_logger.return_value = Mock()
        mock_load_sim.return_value = Mock()
        
        simulator = PhysicsSimulator(basic_config)
        simulator.world = Mock()
        
        # Mock robot and its state
        with patch('physics_simulator.robot.MujocoRobot') as mock_robot_class:
            mock_robot = Mock()
            mock_robot.get_state.return_value = {
                'position': [0, 0, 0],
                'orientation': [0, 0, 0, 1],
                'joint_positions': [0.1, -0.2, 0.3]
            }
            mock_robot_class.return_value = mock_robot
            
            # Add robot and get state
            simulator.add_robot(robot_config)
            state = simulator.get_robot_state(robot_config.prim_path)
            
            # Verify state structure
            assert 'position' in state
            assert 'orientation' in state
            assert 'joint_positions' in state

    @patch('physics_simulator.simulator.mujoco.PathManager')
    @patch('physics_simulator.simulator.mujoco.signal')
    @patch('physics_simulator.simulator.mujoco.MujocoSimulator._load_logger')
    @patch('physics_simulator.simulator.mujoco.MujocoSimulator._load_simulator')
    def test_simulation_time_tracking(self, mock_load_sim, mock_load_logger, mock_signal, mock_path_manager, basic_config):
        """Test simulation time tracking functionality."""
        # Setup mocks
        mock_path_manager.get_root_path.return_value = "/mock/path"
        mock_load_logger.return_value = Mock()
        mock_load_sim.return_value = Mock()
        
        simulator = PhysicsSimulator(basic_config)
        
        # Mock time tracking
        simulator.data = Mock()
        simulator.data._data = Mock()
        simulator.data._data.time = 0.0
        simulator._mujoco_timestep = 0.01
        
        # Test initial time
        assert simulator.get_simulation_time() == 0.0
        assert simulator.get_physics_dt() == 0.01
        
        # Simulate time progression
        simulator.data._data.time = 1.5
        assert simulator.get_simulation_time() == 1.5

    @patch('physics_simulator.simulator.mujoco.PathManager')
    @patch('physics_simulator.simulator.mujoco.signal')
    @patch('physics_simulator.simulator.mujoco.MujocoSimulator._load_logger')
    @patch('physics_simulator.simulator.mujoco.MujocoSimulator._load_simulator')
    def test_reset_and_state_consistency(self, mock_load_sim, mock_load_logger, mock_signal, mock_path_manager, basic_config):
        """Test that reset maintains state consistency."""
        # Setup mocks
        mock_path_manager.get_root_path.return_value = "/mock/path"
        mock_load_logger.return_value = Mock()
        mock_load_sim.return_value = Mock()
        
        simulator = PhysicsSimulator(basic_config)
        simulator.model = Mock()
        simulator.data = Mock()
        
        # Test reset functionality
        with patch('mujoco.mj_resetData') as mock_reset:
            simulator.reset()
            mock_reset.assert_called_once()

    @patch('physics_simulator.simulator.mujoco.PathManager')
    @patch('physics_simulator.simulator.mujoco.signal')
    @patch('physics_simulator.simulator.mujoco.MujocoSimulator._load_logger')
    @patch('physics_simulator.simulator.mujoco.MujocoSimulator._load_simulator')
    def test_physics_callback_workflow(self, mock_load_sim, mock_load_logger, mock_signal, mock_path_manager, basic_config):
        """Test physics callback integration in simulation workflow."""
        # Setup mocks
        mock_path_manager.get_root_path.return_value = "/mock/path"
        mock_load_logger.return_value = Mock()
        mock_load_sim.return_value = Mock()
        
        simulator = PhysicsSimulator(basic_config)
        
        # Track callback calls
        callback_calls = []
        
        def test_callback():
            callback_calls.append("called")
        
        # Add callback and verify it exists
        simulator.add_physics_callback("test_callback", test_callback)
        assert simulator.physics_callback_exists("test_callback")
        
        # Remove callback and verify it's gone
        simulator.remove_physics_callback("test_callback")
        assert not simulator.physics_callback_exists("test_callback")

    @patch('physics_simulator.simulator.mujoco.PathManager')
    @patch('physics_simulator.simulator.mujoco.signal')
    @patch('physics_simulator.simulator.mujoco.MujocoSimulator._load_logger')
    @patch('physics_simulator.simulator.mujoco.MujocoSimulator._load_simulator')
    def test_multiple_robots_management(self, mock_load_sim, mock_load_logger, mock_signal, mock_path_manager, basic_config):
        """Test managing multiple robots in the same simulation."""
        # Setup mocks
        mock_path_manager.get_root_path.return_value = "/mock/path"
        mock_load_logger.return_value = Mock()
        mock_load_sim.return_value = Mock()
        
        simulator = PhysicsSimulator(basic_config)
        simulator.world = Mock()
        
        # Create multiple robot configs
        robot_configs = [
            RobotConfig(
                prim_path=f"/World/Robot{i}",
                name=f"robot_{i}",
                mjcf_path="test_robot.xml",
                position=[i, 0, 0],
                orientation=[0, 0, 0, 1]
            ) for i in range(3)
        ]
        
        with patch('physics_simulator.robot.MujocoRobot') as mock_robot_class:
            mock_robot_class.return_value = Mock()
            
            # Add multiple robots
            for config in robot_configs:
                simulator.add_robot(config)
            
            # Verify all robots were added
            assert len(simulator._robots) == 3
            for config in robot_configs:
                assert config.prim_path in simulator._robots
                
            # Test retrieving each robot
            for config in robot_configs:
                robot = simulator.get_robot(config.prim_path)
                assert robot is not None 