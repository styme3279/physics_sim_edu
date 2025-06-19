"""
Tests for MujocoSimulator class.
"""
import pytest
import numpy as np
from unittest.mock import Mock, patch, MagicMock
from physics_simulator.simulator.mujoco import MujocoSimulator


@pytest.mark.unit
class TestMujocoSimulator:
    """Test suite for MujocoSimulator class."""

    @patch('physics_simulator.simulator.mujoco.PathManager')
    @patch('physics_simulator.simulator.mujoco.signal')
    def test_simulator_initialization(self, mock_signal, mock_path_manager, basic_config):
        """Test simulator initialization with basic configuration."""
        mock_path_manager.get_root_path.return_value = "/mock/path"
        
        with patch.object(MujocoSimulator, '_load_logger') as mock_logger, \
             patch.object(MujocoSimulator, '_load_simulator') as mock_sim:
            mock_logger.return_value = Mock()
            mock_sim.return_value = Mock()
            
            simulator = MujocoSimulator(basic_config)
            
            assert simulator.config == basic_config
            assert simulator._running is False
            assert simulator._robots == {}
            assert simulator._sensors == {}
            assert simulator._objects == {}

    def test_is_running_status(self, basic_config):
        """Test simulator running status tracking."""
        with patch.object(MujocoSimulator, '_load_logger'), \
             patch.object(MujocoSimulator, '_load_simulator'), \
             patch('physics_simulator.simulator.mujoco.PathManager'), \
             patch('physics_simulator.simulator.mujoco.signal'):
            
            simulator = MujocoSimulator(basic_config)
            assert not simulator.is_running()
            
            simulator._running = True
            assert simulator.is_running()

    @patch('mujoco.mj_step')
    def test_step_simulation(self, mock_step, basic_config):
        """Test simulation stepping functionality."""
        with patch.object(MujocoSimulator, '_load_logger'), \
             patch.object(MujocoSimulator, '_load_simulator'), \
             patch('physics_simulator.simulator.mujoco.PathManager'), \
             patch('physics_simulator.simulator.mujoco.signal'):
            
            simulator = MujocoSimulator(basic_config)
            simulator._running = True
            simulator.model = Mock()
            simulator.model._model = Mock()
            simulator.data = Mock()
            simulator.data._data = Mock()
            simulator.viewer = None
            
            simulator.step(num_steps=3)
            assert mock_step.call_count == 3

    def test_get_simulation_time(self, basic_config):
        """Test getting simulation time."""
        with patch.object(MujocoSimulator, '_load_logger'), \
             patch.object(MujocoSimulator, '_load_simulator'), \
             patch('physics_simulator.simulator.mujoco.PathManager'), \
             patch('physics_simulator.simulator.mujoco.signal'):
            
            simulator = MujocoSimulator(basic_config)
            simulator.data = Mock()
            simulator.data._data = Mock()
            simulator.data._data.time = 1.5
            
            assert simulator.get_simulation_time() == 1.5

    def test_get_physics_dt(self, basic_config):
        """Test getting physics timestep."""
        with patch.object(MujocoSimulator, '_load_logger'), \
             patch.object(MujocoSimulator, '_load_simulator'), \
             patch('physics_simulator.simulator.mujoco.PathManager'), \
             patch('physics_simulator.simulator.mujoco.signal'):
            
            simulator = MujocoSimulator(basic_config)
            simulator._mujoco_timestep = 0.01
            
            assert simulator.get_physics_dt() == 0.01

    def test_robot_management(self, basic_config, robot_config):
        """Test adding and retrieving robots."""
        with patch.object(MujocoSimulator, '_load_logger'), \
             patch.object(MujocoSimulator, '_load_simulator'), \
             patch('physics_simulator.simulator.mujoco.PathManager'), \
             patch('physics_simulator.simulator.mujoco.signal'):
            
            simulator = MujocoSimulator(basic_config)
            simulator.world = Mock()
            
            # Mock robot creation
            with patch('physics_simulator.robot.MujocoRobot') as mock_robot_class:
                mock_robot = Mock()
                mock_robot_class.return_value = mock_robot
                
                simulator.add_robot(robot_config)
                
                assert robot_config.prim_path in simulator._robots
                retrieved_robot = simulator.get_robot(robot_config.prim_path)
                assert retrieved_robot == mock_robot

    def test_object_management(self, basic_config, object_config):
        """Test adding and retrieving objects."""
        with patch.object(MujocoSimulator, '_load_logger'), \
             patch.object(MujocoSimulator, '_load_simulator'), \
             patch('physics_simulator.simulator.mujoco.PathManager'), \
             patch('physics_simulator.simulator.mujoco.signal'):
            
            simulator = MujocoSimulator(basic_config)
            simulator.world = Mock()
            
            # Mock object creation
            with patch('physics_simulator.object.MujocoObject') as mock_object_class:
                mock_object = Mock()
                mock_object_class.return_value = mock_object
                
                simulator.add_object(object_config)
                
                assert object_config.prim_path in simulator._objects
                retrieved_object = simulator.get_object(object_config.prim_path)
                assert retrieved_object == mock_object

    def test_joint_position_setting(self, basic_config, sample_joint_positions, sample_joint_names):
        """Test setting joint positions for robots."""
        with patch.object(MujocoSimulator, '_load_logger'), \
             patch.object(MujocoSimulator, '_load_simulator'), \
             patch('physics_simulator.simulator.mujoco.PathManager'), \
             patch('physics_simulator.simulator.mujoco.signal'):
            
            simulator = MujocoSimulator(basic_config)
            mock_robot = Mock()
            
            with patch('mujoco.mj_name2id', return_value=0), \
                 patch('mujoco.mj_forward'):
                simulator.model = Mock()
                simulator.model._model = Mock()
                simulator.data = Mock()
                simulator.data._data = Mock()
                simulator.data._data.ctrl = np.zeros(10)
                
                # Should not raise any exceptions
                simulator.set_joint_positions(
                    mock_robot, 
                    sample_joint_positions.tolist(), 
                    sample_joint_names
                )

    def test_reset_functionality(self, basic_config):
        """Test simulation reset functionality."""
        with patch.object(MujocoSimulator, '_load_logger'), \
             patch.object(MujocoSimulator, '_load_simulator'), \
             patch('physics_simulator.simulator.mujoco.PathManager'), \
             patch('physics_simulator.simulator.mujoco.signal'):
            
            simulator = MujocoSimulator(basic_config)
            simulator.model = Mock()
            simulator.data = Mock()
            
            with patch('mujoco.mj_resetData') as mock_reset:
                simulator.reset()
                mock_reset.assert_called_once()

    def test_close_functionality(self, basic_config):
        """Test simulator close and cleanup."""
        with patch.object(MujocoSimulator, '_load_logger'), \
             patch.object(MujocoSimulator, '_load_simulator'), \
             patch('physics_simulator.simulator.mujoco.PathManager'), \
             patch('physics_simulator.simulator.mujoco.signal'):
            
            simulator = MujocoSimulator(basic_config)
            simulator._running = True
            simulator.viewer = Mock()
            
            simulator.close()
            
            assert not simulator._running
            simulator.viewer.close.assert_called_once()

    def test_physics_callback_management(self, basic_config):
        """Test physics callback addition and removal."""
        with patch.object(MujocoSimulator, '_load_logger'), \
             patch.object(MujocoSimulator, '_load_simulator'), \
             patch('physics_simulator.simulator.mujoco.PathManager'), \
             patch('physics_simulator.simulator.mujoco.signal'):
            
            simulator = MujocoSimulator(basic_config)
            
            def dummy_callback():
                pass
            
            # Test adding callback
            simulator.add_physics_callback("test_callback", dummy_callback)
            assert simulator.physics_callback_exists("test_callback")
            
            # Test removing callback
            simulator.remove_physics_callback("test_callback")
            assert not simulator.physics_callback_exists("test_callback") 