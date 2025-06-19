"""
Tests for MujocoRobotModel class.
"""
import pytest
import numpy as np
from unittest.mock import Mock, patch, MagicMock
from physics_simulator.robot.robot_model import MujocoRobotModel


@pytest.mark.unit
class TestMujocoRobotModel:
    """Test suite for MujocoRobotModel class."""

    @patch('physics_simulator.object.MujocoXMLModel.__init__')
    def test_robot_model_initialization(self, mock_super_init):
        """Test robot model initialization."""
        mock_super_init.return_value = None
        
        robot_model = MujocoRobotModel("test_robot.xml", "test_namespace")
        
        assert robot_model.namespace == "test_namespace"
        assert robot_model.joint_names == []
        assert robot_model.links_name == []
        assert robot_model.actuators == []
        assert robot_model.root_body is None

    def test_actuators_property(self):
        """Test actuators property getter and setter."""
        with patch('physics_simulator.object.MujocoXMLModel.__init__'):
            robot_model = MujocoRobotModel("test_robot.xml", "test_namespace")
            
            # Test initial empty state
            assert robot_model.actuators == []
            
            # Test internal setting (through initialization)
            robot_model._actuators = ["actuator1", "actuator2"]
            assert robot_model.actuators == ["actuator1", "actuator2"]

    def test_root_body_property(self):
        """Test root body property getter and setter."""
        with patch('physics_simulator.object.MujocoXMLModel.__init__'):
            robot_model = MujocoRobotModel("test_robot.xml", "test_namespace")
            
            # Test initial state
            assert robot_model.root_body is None
            
            # Test setter
            robot_model.root_body = "base_link"
            assert robot_model.root_body == "base_link"

    @patch('mujoco.mj_id2name')
    @patch('mujoco.mj_name2id')
    def test_initialization_with_mujoco_model(self, mock_name2id, mock_id2name):
        """Test robot model initialization with MuJoCo model and data."""
        with patch('physics_simulator.object.MujocoXMLModel.__init__'):
            robot_model = MujocoRobotModel("test_robot.xml", "test_")
            
            # Mock MuJoCo model and data
            mock_model = Mock()
            mock_model.njnt = 3
            mock_model.nu = 2
            mock_model.nbody = 4
            mock_model.jnt_limited = np.array([True, False, True])
            mock_model.jnt_range = np.array([[-1, 1], [0, 0], [-2, 2]])
            
            mock_data = Mock()
            
            # Mock mj_id2name returns for joints, actuators, and bodies
            def mock_id2name_side_effect(model, obj_type, id):
                if obj_type == 1:  # mjOBJ_JOINT
                    joint_names = ["test_joint1", "test_joint2", "other_joint"]
                    return joint_names[id] if id < len(joint_names) else None
                elif obj_type == 3:  # mjOBJ_ACTUATOR
                    actuator_names = ["test_actuator1", "test_actuator2"]
                    return actuator_names[id] if id < len(actuator_names) else None
                elif obj_type == 1:  # mjOBJ_BODY (same enum value as JOINT, need different handling)
                    if id == 0:
                        return "world"
                    body_names = ["world", "test_base", "test_link1", "test_link2"]
                    return body_names[id] if id < len(body_names) else None
                return None
            
            mock_id2name.side_effect = mock_id2name_side_effect
            mock_name2id.return_value = 0
            
            robot_model.initialize(mock_model, mock_data)
            
            # Check that joints with namespace prefix are included
            assert len(robot_model.joint_names) == 2  # test_joint1, test_joint2
            assert "test_joint1" in robot_model.joint_names
            assert "test_joint2" in robot_model.joint_names
            
            # Check actuators
            assert len(robot_model.actuators) == 2
            assert "test_actuator1" in robot_model.actuators

    def test_generate_random_qpos(self):
        """Test generation of random joint positions."""
        with patch('physics_simulator.object.MujocoXMLModel.__init__'):
            robot_model = MujocoRobotModel("test_robot.xml", "test_namespace")
            
            # Set up joint limits manually
            robot_model.joint_names = ["joint1", "joint2", "joint3"]
            robot_model.joints_lower_limit = np.array([-1.0, -2.0, -0.5])
            robot_model.joints_upper_limit = np.array([1.0, 2.0, 0.5])
            
            random_qpos = robot_model.generate_random_qpos()
            
            assert len(random_qpos) == 3
            assert np.all(random_qpos >= robot_model.joints_lower_limit)
            assert np.all(random_qpos <= robot_model.joints_upper_limit)

    def test_clip_qpos(self):
        """Test joint position clipping to limits."""
        with patch('physics_simulator.object.MujocoXMLModel.__init__'):
            robot_model = MujocoRobotModel("test_robot.xml", "test_namespace")
            
            # Set up joint limits
            robot_model.joint_names = ["joint1", "joint2"]
            robot_model.joints_lower_limit = np.array([-1.0, -2.0])
            robot_model.joints_upper_limit = np.array([1.0, 2.0])
            
            # Test clipping values outside limits
            input_qpos = np.array([-2.0, 3.0])  # Both outside limits
            clipped_qpos = robot_model.clip_qpos(input_qpos)
            
            expected_qpos = np.array([-1.0, 2.0])  # Clipped to limits
            np.testing.assert_array_equal(clipped_qpos, expected_qpos)

    @patch('mujoco.mj_forward')
    @patch('mujoco.mj_name2id')
    def test_forward_kinematics_link(self, mock_name2id, mock_forward):
        """Test forward kinematics computation for a specific link."""
        with patch('physics_simulator.object.MujocoXMLModel.__init__'):
            robot_model = MujocoRobotModel("test_robot.xml", "test_namespace")
            
            # Set up mock model and data
            mock_model = Mock()
            mock_model.jnt_qposadr = np.array([0, 1, 2])
            mock_data = Mock()
            mock_data.qpos = np.zeros(10)
            mock_data.xpos = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]])
            mock_data.xmat = np.eye(3).reshape(1, 9)
            
            robot_model._mujoco_model = mock_model
            robot_model._mujoco_data = mock_data
            robot_model.joint_names = ["joint1", "joint2"]
            
            mock_name2id.return_value = 1  # Mock link ID
            
            q = np.array([0.1, -0.2])
            translation, rotation = robot_model.fk_link(q, "test_link")
            
            # Should return translation and rotation
            assert translation.shape == (3,)
            assert rotation.shape == (3, 3)
            mock_forward.assert_called_once()

    def test_fk_link_without_initialization(self):
        """Test that forward kinematics raises error when not initialized."""
        with patch('physics_simulator.object.MujocoXMLModel.__init__'):
            robot_model = MujocoRobotModel("test_robot.xml", "test_namespace")
            
            q = np.array([0.1, -0.2])
            
            with pytest.raises(ValueError, match="MuJoCo model and data not initialized"):
                robot_model.fk_link(q, "test_link")

    def test_contact_geom_rgba_property(self):
        """Test contact geometry RGBA property."""
        with patch('physics_simulator.object.MujocoXMLModel.__init__'):
            robot_model = MujocoRobotModel("test_robot.xml", "test_namespace")
            
            rgba = robot_model.contact_geom_rgba
            assert isinstance(rgba, np.ndarray)
            assert len(rgba) == 4  # RGBA should have 4 components

    def test_naming_prefix_property(self):
        """Test naming prefix property."""
        with patch('physics_simulator.object.MujocoXMLModel.__init__'):
            robot_model = MujocoRobotModel("test_robot.xml", "test_namespace")
            
            assert robot_model.naming_prefix == "test_namespace"

    @patch('physics_simulator.utils.mjcf_utils.array_to_string')
    def test_set_base_position(self, mock_array_to_string):
        """Test setting robot base position."""
        with patch('physics_simulator.object.MujocoXMLModel.__init__'):
            robot_model = MujocoRobotModel("test_robot.xml", "test_namespace")
            robot_model._elements = {"root_body": Mock()}
            robot_model.bottom_offset = np.array([0, 0, 0.1])
            
            mock_array_to_string.return_value = "1.0 2.0 2.9"
            
            position = np.array([1.0, 2.0, 3.0])
            robot_model.set_base_position(position)
            
            robot_model._elements["root_body"].set.assert_called_with("pos", "1.0 2.0 2.9")

    @patch('physics_simulator.utils.mjcf_utils.array_to_string')
    @patch('auro_utils.math.transform.xyzw_to_wxyz')
    def test_set_base_orientation(self, mock_xyzw_to_wxyz, mock_array_to_string):
        """Test setting robot base orientation."""
        with patch('physics_simulator.object.MujocoXMLModel.__init__'):
            robot_model = MujocoRobotModel("test_robot.xml", "test_namespace")
            robot_model._elements = {"root_body": Mock()}
            
            mock_xyzw_to_wxyz.return_value = np.array([1, 0, 0, 0])
            mock_array_to_string.return_value = "1.0 0.0 0.0 0.0"
            
            orientation = np.array([0, 0, 0, 1])  # xyzw format
            robot_model.set_base_orientation(orientation)
            
            robot_model._elements["root_body"].set.assert_called_with("quat", "1.0 0.0 0.0 0.0") 