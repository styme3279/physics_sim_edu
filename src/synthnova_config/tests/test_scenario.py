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
# Description: Test cases for ScenarioConfig
# Author: Herman Ye@Galbot
# Date: 2025-05-11
#
#####################################################################################

import pytest
import toml
from pathlib import Path
import tempfile
import uuid
import numpy as np

from synthnova_config.scenario import ScenarioConfig
from synthnova_config.robot import RobotConfig
from synthnova_config.object import (
    ObjectConfig,
    MeshConfig,
    CuboidConfig,
    GroundPlaneConfig,
)
from synthnova_config.sensor import (
    SensorConfig,
    RgbCameraConfig,
    DepthCameraConfig,
)


@pytest.fixture
def temp_dir():
    """Create a temporary directory for test files."""
    with tempfile.TemporaryDirectory() as tmp_dir:
        yield Path(tmp_dir)


@pytest.fixture
def robot_usd_file(temp_dir):
    """Create a temporary USD file for robot testing."""
    usd_file = temp_dir / "robot.usd"
    usd_file.touch()  # Create an empty file
    return usd_file


@pytest.fixture
def mesh_obj_file(temp_dir):
    """Create a temporary OBJ file for mesh testing."""
    obj_file = temp_dir / "mesh.obj"
    obj_file.touch()  # Create an empty file
    return obj_file


@pytest.fixture
def basic_scenario():
    """Create a basic scenario configuration for testing."""
    return ScenarioConfig(
        name="test_scenario",
        description="Test scenario for unit testing",
    )


@pytest.fixture
def complex_scenario(temp_dir, robot_usd_file, mesh_obj_file):
    """Create a complex scenario with various components."""
    # Create robot config
    robot = RobotConfig(
        name="test_robot",
        prim_path="/World/robots/test_robot",
        usd_path=str(robot_usd_file),  # Use actual file path
    )

    # Create object configs
    mesh_obj = MeshConfig(
        name="test_mesh",
        prim_path="/World/objects/test_mesh",
        obj_path=str(mesh_obj_file),  # Use actual file path
    )

    cuboid = CuboidConfig(
        name="test_cuboid",
        prim_path="/World/objects/test_cuboid",
        scale=np.array([1.0, 1.0, 1.0]),
    )

    # Create sensor configs
    rgb_camera = RgbCameraConfig(
        name="test_rgb_camera",
        prim_path="/World/sensors/test_rgb_camera",
    )

    depth_camera = DepthCameraConfig(
        name="test_depth_camera",
        prim_path="/World/sensors/test_depth_camera",
    )

    # Create ground plane config
    ground = GroundPlaneConfig(
        static_friction=0.5,
        dynamic_friction=0.5,
        restitution=0.8,
        z_position=0.0,
    )

    return ScenarioConfig(
        name="complex_scenario",
        description="Complex test scenario with multiple components",
        robots=[robot],
        objects=[mesh_obj, cuboid],
        sensors=[rgb_camera, depth_camera],
        ground_planes=[ground],
    )


def test_scenario_creation():
    """Test basic scenario creation."""
    scenario = ScenarioConfig()
    # Check the name pattern using string operations
    name_str = str(scenario.name)
    assert "scenario_" in name_str
    assert scenario.description is None
    assert isinstance(scenario.uuid, str)
    assert len(scenario.uuid) > 0
    assert scenario.robots == []
    assert scenario.objects == []
    assert scenario.sensors == []
    assert scenario.ground_planes == []


def test_scenario_with_custom_name():
    """Test scenario creation with custom name."""
    name = "custom_scenario"
    scenario = ScenarioConfig(name=name)
    assert scenario.name == name


def test_scenario_name_validation():
    """Test scenario name validation."""
    with pytest.raises(ValueError):
        ScenarioConfig(name="")  # Empty name should raise error

    with pytest.raises(ValueError):
        ScenarioConfig(name="   ")  # Whitespace-only name should raise error


def test_scenario_uuid_format():
    """Test UUID format validation and conversion."""
    # Test with standard UUID
    uuid_str = str(uuid.uuid4())
    scenario = ScenarioConfig(uuid=uuid_str)
    assert "_" in scenario.uuid
    assert "-" not in scenario.uuid

    # Test with invalid UUID
    with pytest.raises(ValueError):
        ScenarioConfig(uuid="invalid-uuid")


def test_scenario_with_components(complex_scenario):
    """Test scenario with various components."""
    assert len(complex_scenario.robots) == 1
    assert len(complex_scenario.objects) == 2
    assert len(complex_scenario.sensors) == 2
    assert len(complex_scenario.ground_planes) == 1

    # Verify robot
    robot = complex_scenario.robots[0]
    assert robot.name == "test_robot"
    assert robot.prim_path == "/World/robots/test_robot"
    # Check that usd_path is an absolute path
    assert Path(robot.usd_path).is_absolute()

    # Verify objects
    mesh_obj = complex_scenario.objects[0]
    assert isinstance(mesh_obj, MeshConfig)
    assert mesh_obj.name == "test_mesh"

    cuboid = complex_scenario.objects[1]
    assert isinstance(cuboid, CuboidConfig)
    assert cuboid.name == "test_cuboid"

    # Verify sensors
    rgb_camera = complex_scenario.sensors[0]
    assert isinstance(rgb_camera, RgbCameraConfig)
    assert rgb_camera.name == "test_rgb_camera"

    depth_camera = complex_scenario.sensors[1]
    assert isinstance(depth_camera, DepthCameraConfig)
    assert depth_camera.name == "test_depth_camera"

    # Verify ground plane
    ground = complex_scenario.ground_planes[0]
    assert isinstance(ground, GroundPlaneConfig)
    assert ground.static_friction == 0.5
    assert ground.dynamic_friction == 0.5
    assert ground.restitution == 0.8
    assert ground.z_position == 0.0


def test_scenario_to_dict(basic_scenario):
    """Test conversion to dictionary."""
    scenario_dict = basic_scenario.to_dict()
    assert isinstance(scenario_dict, dict)
    assert scenario_dict["name"] == "test_scenario"
    assert scenario_dict["description"] == "Test scenario for unit testing"
    assert "uuid" in scenario_dict


def test_scenario_from_dict(basic_scenario):
    """Test creation from dictionary."""
    scenario_dict = basic_scenario.to_dict()
    new_scenario = ScenarioConfig.from_dict(scenario_dict)
    assert new_scenario.name == basic_scenario.name
    assert new_scenario.description == basic_scenario.description
    assert new_scenario.uuid == basic_scenario.uuid


def test_scenario_file_operations(temp_dir, complex_scenario):
    """Test loading and saving scenario to file."""
    # Save scenario to file
    file_path = temp_dir / "test_scenario.toml"
    complex_scenario.export_to_file(file_path)
    assert file_path.exists()

    # Load scenario from file
    loaded_scenario = ScenarioConfig.load_from_file(file_path)
    assert loaded_scenario.name == complex_scenario.name
    assert loaded_scenario.description == complex_scenario.description
    assert len(loaded_scenario.robots) == len(complex_scenario.robots)
    assert len(loaded_scenario.objects) == len(complex_scenario.objects)
    assert len(loaded_scenario.sensors) == len(complex_scenario.sensors)
    assert len(loaded_scenario.ground_planes) == len(complex_scenario.ground_planes)


def test_scenario_file_not_found():
    """Test handling of non-existent file."""
    with pytest.raises(FileNotFoundError, match="Configuration file not found: nonexistent_file.toml"):
        ScenarioConfig.load_from_file("nonexistent_file.toml")


def test_scenario_invalid_file_format(temp_dir):
    """Test handling of invalid file format."""
    file_path = temp_dir / "invalid.toml"
    with open(file_path, "w") as f:
        f.write("invalid toml content")

    with pytest.raises(ValueError):
        ScenarioConfig.load_from_file(file_path)


def test_scenario_extra_fields():
    """Test handling of extra fields in configuration."""
    with pytest.raises(ValueError):
        ScenarioConfig(extra_field="should raise error")


def test_scenario_none_values():
    """Test handling of None values in configuration."""
    scenario = ScenarioConfig(
        name="test_scenario",
        description=None,  # Should remain None
    )
    assert scenario.description is None  # Changed from empty string to None


def test_scenario_whitespace_handling():
    """Test handling of whitespace in string fields."""
    scenario = ScenarioConfig(
        name="  test_scenario  ",
        description="  test description  ",
    )
    assert scenario.name == "test_scenario"
    assert scenario.description == "test description" 