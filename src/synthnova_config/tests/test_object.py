import pytest
import numpy as np
from pathlib import PosixPath
import tempfile
import os
from synthnova_config.object.object import (
    ObjectConfig,
    MeshConfig,
    CapsuleConfig,
    ConeConfig,
    CuboidConfig,
    CylinderConfig,
    SphereConfig,
    ObjType,
    CollisionType,
    SemanticType,
    InteractionType,
    convert_to_abs_str_path,
    convert_int_to_float,
)


def test_object_config_basic():
    """Test basic ObjectConfig creation and validation."""
    config = ObjectConfig(
        prim_path="/test/path",
        type=ObjType.SPHERE,
        name="test_sphere",
        position=np.array([1.0, 2.0, 3.0]),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
    )
    assert config.prim_path == "/test/path"
    assert config.type == ObjType.SPHERE
    assert config.name == "test_sphere"
    assert np.array_equal(config.position, np.array([1.0, 2.0, 3.0]))
    assert np.array_equal(config.orientation, np.array([0.0, 0.0, 0.0, 1.0]))


def test_object_config_pose_validation():
    """Test pose validation in ObjectConfig."""
    # Test that we can't use both global and local coordinates
    with pytest.raises(ValueError):
        ObjectConfig(
            prim_path="/test/path",
            position=np.array([1.0, 2.0, 3.0]),
            translation=np.array([1.0, 2.0, 3.0]),
        )

    # Test that we can use either global or local coordinates
    config1 = ObjectConfig(
        prim_path="/test/path",
        position=np.array([1.0, 2.0, 3.0]),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
    )
    assert config1.position is not None
    assert config1.translation is None

    config2 = ObjectConfig(
        prim_path="/test/path",
        translation=np.array([1.0, 2.0, 3.0]),
        rotation=np.array([0.0, 0.0, 0.0, 1.0]),
    )
    assert config2.translation is not None
    assert config2.position is None


def test_mesh_config():
    """Test MeshConfig creation and validation."""
    # Create a temporary file for testing
    with tempfile.NamedTemporaryFile(suffix='.usd', delete=False) as temp_file:
        temp_path = temp_file.name
        
        # Test with USD path
        config = MeshConfig(
            prim_path="/test/path",
            usd_path=temp_path,
            collision_type=CollisionType.CONVEX_HULL,
        )
        assert config.type == ObjType.MESH
        assert config.collision_type == CollisionType.CONVEX_HULL

        # Test that at least one path is required
        with pytest.raises(ValueError):
            MeshConfig(prim_path="/test/path")
    
    # Clean up the temporary file
    os.unlink(temp_path)


def test_capsule_config():
    """Test CapsuleConfig creation and validation."""
    config = CapsuleConfig(
        prim_path="/test/path",
        radius=2.0,
        height=3.0,
        color=np.array([0.5, 0.5, 0.5]),
    )
    assert config.type == ObjType.CAPSULE
    assert config.radius == 2.0
    assert config.height == 3.0
    assert np.array_equal(config.color, np.array([0.5, 0.5, 0.5]))

    # Test color validation
    with pytest.raises(ValueError):
        CapsuleConfig(
            prim_path="/test/path",
            color=np.array([1.5, 0.5, 0.5]),  # Invalid color value
        )


def test_cone_config():
    """Test ConeConfig creation and validation."""
    config = ConeConfig(
        prim_path="/test/path",
        radius=2.0,
        height=3.0,
        color=np.array([0.5, 0.5, 0.5]),
    )
    assert config.type == ObjType.CONE
    assert config.radius == 2.0
    assert config.height == 3.0
    assert np.array_equal(config.color, np.array([0.5, 0.5, 0.5]))


def test_cuboid_config():
    """Test CuboidConfig creation and validation."""
    config = CuboidConfig(
        prim_path="/test/path",
        size=2.0,
        color=np.array([0.5, 0.5, 0.5]),
    )
    assert config.type == ObjType.CUBOID
    assert config.size == 2.0
    assert np.array_equal(config.color, np.array([0.5, 0.5, 0.5]))


def test_cylinder_config():
    """Test CylinderConfig creation and validation."""
    config = CylinderConfig(
        prim_path="/test/path",
        radius=2.0,
        height=3.0,
        color=np.array([0.5, 0.5, 0.5]),
    )
    assert config.type == ObjType.CYLINDER
    assert config.radius == 2.0
    assert config.height == 3.0
    assert np.array_equal(config.color, np.array([0.5, 0.5, 0.5]))


def test_sphere_config():
    """Test SphereConfig creation and validation."""
    config = SphereConfig(
        prim_path="/test/path",
        radius=2.0,
        color=np.array([0.5, 0.5, 0.5]),
    )
    assert config.type == ObjType.SPHERE
    assert config.radius == 2.0
    assert np.array_equal(config.color, np.array([0.5, 0.5, 0.5]))


def test_convert_to_abs_str_path():
    """Test path conversion utility."""
    # Test with string path
    assert convert_to_abs_str_path("/test/path") == "/test/path"
    
    # Test with PosixPath
    path = PosixPath("/test/path")
    assert convert_to_abs_str_path(path) == str(path.absolute())
    
    # Test with None
    assert convert_to_abs_str_path(None) is None
    
    # Test with empty string
    with pytest.raises(ValueError):
        convert_to_abs_str_path("")


def test_convert_int_to_float():
    """Test integer to float conversion utility."""
    # Test with list
    assert np.array_equal(
        convert_int_to_float([1, 2, 3]),
        np.array([1.0, 2.0, 3.0], dtype=np.float64)
    )
    
    # Test with tuple
    assert np.array_equal(
        convert_int_to_float((1, 2, 3)),
        np.array([1.0, 2.0, 3.0], dtype=np.float64)
    )
    
    # Test with numpy array
    assert np.array_equal(
        convert_int_to_float(np.array([1, 2, 3])),
        np.array([1.0, 2.0, 3.0], dtype=np.float64)
    )
    
    # Test with None
    assert convert_int_to_float(None) is None


def test_object_config_serialization():
    """Test ObjectConfig serialization and deserialization."""
    config = ObjectConfig(
        prim_path="/test/path",
        type=ObjType.SPHERE,
        name="test_sphere",
        position=np.array([1.0, 2.0, 3.0]),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
    )
    
    # Test to_dict
    config_dict = config.to_dict()
    assert config_dict["prim_path"] == "/test/path"
    assert config_dict["type"] == "sphere"
    assert config_dict["name"] == "test_sphere"
    assert np.array_equal(config_dict["position"], np.array([1.0, 2.0, 3.0]))
    
    # Test from_dict
    new_config = ObjectConfig.from_dict(config_dict)
    assert new_config.prim_path == config.prim_path
    assert new_config.type == config.type
    assert new_config.name == config.name
    assert np.array_equal(new_config.position, config.position)
    assert np.array_equal(new_config.orientation, config.orientation) 