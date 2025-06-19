"""
Pytest configuration and fixtures for physics simulator tests.
"""
import pytest
import numpy as np
from pathlib import Path
from synthnova_config import (
    PhysicsSimulatorConfig,
    MujocoConfig,
    LoggerConfig,
    RobotConfig,
    ObjectConfig,
    GroundPlaneConfig
)


@pytest.fixture
def basic_config():
    """Create basic physics simulator configuration for testing."""
    config = PhysicsSimulatorConfig()
    config.mujoco_config = MujocoConfig(headless=True)
    config.logger_config = LoggerConfig()  # Use default logger config
    return config


@pytest.fixture
def robot_config():
    """Create basic robot configuration for testing."""
    return RobotConfig(
        prim_path="/World/TestRobot",
        name="test_robot",
        mjcf_path="test_robot.xml",  # Mock path for testing
        position=[0, 0, 0],
        orientation=[0, 0, 0, 1]
    )


@pytest.fixture
def object_config():
    """Create basic object configuration for testing."""
    return ObjectConfig(
        prim_path="/World/TestObject",
        name="test_object",
        mjcf_path="test_object.xml",  # Mock path for testing
        position=[1, 0, 0],
        orientation=[0, 0, 0, 1]
    )


@pytest.fixture
def ground_plane_config():
    """Create ground plane configuration for testing."""
    return GroundPlaneConfig(
        prim_path="/World/GroundPlane",
        name="test_ground",
        size=[10, 10, 0.1]
    )


@pytest.fixture
def sample_joint_positions():
    """Sample joint positions for robot testing."""
    return np.array([0.1, -0.2, 0.3, -0.4, 0.5, -0.6])


@pytest.fixture
def sample_joint_names():
    """Sample joint names for robot testing."""
    return ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"] 