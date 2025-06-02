import pytest
import numpy as np
from pathlib import PosixPath
import tempfile
import os
from synthnova_config.robot.robot import RobotConfig
from synthnova_config.object import InteractionType, CollisionType, SemanticType


def test_robot_config_basic_initialization():
    """Test basic initialization of RobotConfig with minimal required parameters."""
    # Create a temporary file for testing
    with tempfile.NamedTemporaryFile(suffix='.usd', delete=False) as tmp:
        tmp_path = tmp.name
    
    try:
        config = RobotConfig(
            prim_path="/World/robot",
            usd_path=tmp_path
        )
        assert config.prim_path == "/World/robot"
        assert config.name is not None
        assert config.uuid is not None
        assert np.array_equal(config.scale, np.array([1.0, 1.0, 1.0]))
        assert config.interaction_type == InteractionType.DYNAMIC
        assert config.collision_type == CollisionType.CONVEX_DECOMPOSITION
        assert config.semantic_type == SemanticType.CLASS
    finally:
        # Clean up the temporary file
        os.unlink(tmp_path)


def test_robot_config_full_initialization():
    """Test initialization of RobotConfig with all parameters."""
    # Create a temporary file for testing
    with tempfile.NamedTemporaryFile(suffix='.usd', delete=False) as tmp:
        tmp_path = tmp.name
    
    try:
        position = np.array([1.0, 2.0, 3.0])
        orientation = np.array([0.0, 0.0, 0.0, 1.0])
        scale = np.array([2.0, 2.0, 2.0])
        
        config = RobotConfig(
            prim_path="/World/robot",
            name="test_robot",
            uuid="123e4567-e89b-12d3-a456-426614174000",
            usd_path=tmp_path,
            position=position,
            orientation=orientation,
            scale=scale,
            interaction_type=InteractionType.STATIC,
            collision_type=CollisionType.CONVEX_DECOMPOSITION,
            semantic_label="robot",
            semantic_type=SemanticType.INSTANCE,
            attributes={"custom_field": "value"}
        )
        
        assert config.prim_path == "/World/robot"
        assert config.name == "test_robot"
        assert config.uuid == "123e4567_e89b_12d3_a456_426614174000"
        assert np.array_equal(config.position, position)
        assert np.array_equal(config.orientation, orientation)
        assert np.array_equal(config.scale, scale)
        assert config.interaction_type == InteractionType.STATIC
        assert config.semantic_label == "robot"
        assert config.semantic_type == SemanticType.INSTANCE
        assert config.attributes is not None
        assert config.attributes.get("custom_field") == "value"
    finally:
        # Clean up the temporary file
        os.unlink(tmp_path)


def test_robot_config_path_conversion():
    """Test path conversion for different input types."""
    # Create temporary files for testing
    with tempfile.NamedTemporaryFile(suffix='.usd', delete=False) as tmp_usd, \
         tempfile.NamedTemporaryFile(suffix='.xml', delete=False) as tmp_mjcf, \
         tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as tmp_urdf, \
         tempfile.NamedTemporaryFile(suffix='.srdf', delete=False) as tmp_srdf:
        tmp_usd_path = tmp_usd.name
        tmp_mjcf_path = tmp_mjcf.name
        tmp_urdf_path = tmp_urdf.name
        tmp_srdf_path = tmp_srdf.name
    
    try:
        config = RobotConfig(
            prim_path=PosixPath("/World/robot"),
            usd_path=PosixPath(tmp_usd_path),
            mjcf_path=tmp_mjcf_path,
            urdf_path=tmp_urdf_path,
            srdf_path=tmp_srdf_path
        )
        
        assert isinstance(config.prim_path, str)
        assert isinstance(config.usd_path, str)
        assert isinstance(config.mjcf_path, str)
        assert isinstance(config.urdf_path, str)
        assert isinstance(config.srdf_path, str)
    finally:
        # Clean up the temporary files
        for path in [tmp_usd_path, tmp_mjcf_path, tmp_urdf_path, tmp_srdf_path]:
            os.unlink(path)


def test_robot_config_pose_validation():
    """Test validation of position/orientation and translation/rotation combinations."""
    # Create a temporary file for testing
    with tempfile.NamedTemporaryFile(suffix='.usd', delete=False) as tmp:
        tmp_path = tmp.name
    
    try:
        # Test valid global coordinates
        config1 = RobotConfig(
            prim_path="/World/robot",
            usd_path=tmp_path,
            position=np.array([1.0, 2.0, 3.0]),
            orientation=np.array([0.0, 0.0, 0.0, 1.0])
        )
        assert config1.position is not None
        assert config1.orientation is not None
        
        # Test valid local coordinates
        config2 = RobotConfig(
            prim_path="/World/robot",
            usd_path=tmp_path,
            translation=np.array([1.0, 2.0, 3.0]),
            rotation=np.array([0.0, 0.0, 0.0, 1.0])
        )
        assert config2.translation is not None
        assert config2.rotation is not None
        
        # Test invalid combination
        with pytest.raises(ValueError):
            RobotConfig(
                prim_path="/World/robot",
                usd_path=tmp_path,
                position=np.array([1.0, 2.0, 3.0]),
                translation=np.array([1.0, 2.0, 3.0])
            )
    finally:
        # Clean up the temporary file
        os.unlink(tmp_path)


def test_robot_config_uuid_validation():
    """Test UUID validation and formatting."""
    # Create a temporary file for testing
    with tempfile.NamedTemporaryFile(suffix='.usd', delete=False) as tmp:
        tmp_path = tmp.name
    
    try:
        # Test valid UUID
        config = RobotConfig(
            prim_path="/World/robot",
            usd_path=tmp_path,
            uuid="123e4567-e89b-12d3-a456-426614174000"
        )
        assert config.uuid == "123e4567_e89b_12d3_a456_426614174000"
        
        # Test invalid UUID
        with pytest.raises(ValueError):
            RobotConfig(
                prim_path="/World/robot",
                usd_path=tmp_path,
                uuid="invalid-uuid"
            )
    finally:
        # Clean up the temporary file
        os.unlink(tmp_path)


def test_robot_config_serialization():
    """Test conversion to and from dictionary."""
    # Create a temporary file for testing
    with tempfile.NamedTemporaryFile(suffix='.usd', delete=False) as tmp:
        tmp_path = tmp.name
    
    try:
        original_config = RobotConfig(
            prim_path="/World/robot",
            usd_path=tmp_path,
            name="test_robot",
            position=np.array([1.0, 2.0, 3.0]),
            orientation=np.array([0.0, 0.0, 0.0, 1.0])
        )
        
        # Convert to dictionary
        config_dict = original_config.to_dict()
        
        # Create new config from dictionary
        new_config = RobotConfig.from_dict(config_dict)
        
        assert new_config.prim_path == original_config.prim_path
        assert new_config.name == original_config.name
        assert np.array_equal(new_config.position, original_config.position)
        assert np.array_equal(new_config.orientation, original_config.orientation)
    finally:
        # Clean up the temporary file
        os.unlink(tmp_path)


def test_robot_config_numeric_conversion():
    """Test conversion of integer values to float in arrays."""
    # Create a temporary file for testing
    with tempfile.NamedTemporaryFile(suffix='.usd', delete=False) as tmp:
        tmp_path = tmp.name
    
    try:
        config = RobotConfig(
            prim_path="/World/robot",
            usd_path=tmp_path,
            position=[1, 2, 3],  # Integer values
            orientation=[0, 0, 0, 1],  # Integer values
            scale=[2, 2, 2]  # Integer values
        )
        
        # Convert numpy arrays to lists for type checking
        position_list = config.position.tolist() if hasattr(config.position, 'tolist') else list(config.position)
        orientation_list = config.orientation.tolist() if hasattr(config.orientation, 'tolist') else list(config.orientation)
        scale_list = config.scale.tolist() if hasattr(config.scale, 'tolist') else list(config.scale)
        
        assert all(isinstance(x, float) for x in position_list)
        assert all(isinstance(x, float) for x in orientation_list)
        assert all(isinstance(x, float) for x in scale_list)
    finally:
        # Clean up the temporary file
        os.unlink(tmp_path) 