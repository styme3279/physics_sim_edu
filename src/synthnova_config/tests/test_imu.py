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
# Description: Test cases for IMU configuration
# Author: Herman Ye@Galbot
# Date: 2025-03-06
#
#####################################################################################

import pytest
import numpy as np
from pathlib import Path
from synthnova_config.sensor.basic.imu import ImuConfig, ImuSensorConfig


def test_imu_sensor_config_creation():
    """Test basic creation of ImuSensorConfig with default values."""
    config = ImuSensorConfig()
    assert config.frequency is None


def test_imu_sensor_config_with_values():
    """Test ImuSensorConfig creation with specific values."""
    config = ImuSensorConfig(frequency=200.0)
    assert config.frequency == 200.0


def test_imu_config_creation():
    """Test basic creation of ImuConfig with required fields."""
    config = ImuConfig(
        name="test_imu",
        prim_path="/World/IMU",
    )

    assert config.type == "imu"
    assert config.name == "test_imu"
    assert config.prim_path == "/World/IMU"
    assert config.uuid is not None
    assert config.position is None
    assert config.orientation is None
    assert config.translation is None
    assert config.rotation is None
    assert isinstance(config.sensor_config, ImuSensorConfig)


def test_imu_config_with_global_pose():
    """Test ImuConfig creation with global pose."""
    config = ImuConfig(
        name="test_imu",
        prim_path="/World/IMU",
        position=np.array([1.0, 2.0, 3.0]),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
    )

    assert np.array_equal(config.position, np.array([1.0, 2.0, 3.0]))
    assert np.array_equal(config.orientation, np.array([0.0, 0.0, 0.0, 1.0]))
    assert config.translation is None
    assert config.rotation is None


def test_imu_config_with_local_pose():
    """Test ImuConfig creation with local pose."""
    config = ImuConfig(
        name="test_imu",
        prim_path="/World/IMU",
        translation=np.array([1.0, 2.0, 3.0]),
        rotation=np.array([0.0, 0.0, 0.0, 1.0]),
    )

    assert config.position is None
    assert config.orientation is None
    assert np.array_equal(config.translation, np.array([1.0, 2.0, 3.0]))
    assert np.array_equal(config.rotation, np.array([0.0, 0.0, 0.0, 1.0]))


def test_imu_config_pose_validation():
    """Test that ImuConfig raises error when both global and local poses are provided."""
    with pytest.raises(ValueError, match="Cannot use both global \\(position/orientation\\) and local \\(translation/rotation\\) coordinates simultaneously"):
        ImuConfig(
            name="test_imu",
            prim_path="/World/IMU",
            position=np.array([1.0, 2.0, 3.0]),
            translation=np.array([1.0, 2.0, 3.0]),
        )


def test_imu_config_uuid_format():
    """Test UUID format validation in ImuConfig."""
    config = ImuConfig(
        name="test_imu",
        prim_path="/World/IMU",
        uuid="12345678-1234-5678-1234-567812345678",
    )

    assert config.uuid == "12345678_1234_5678_1234_567812345678"


def test_imu_config_invalid_uuid():
    """Test that ImuConfig raises error for invalid UUID format."""
    with pytest.raises(ValueError, match="Invalid UUID format"):
        ImuConfig(
            name="test_imu",
            prim_path="/World/IMU",
            uuid="invalid-uuid",
        )


def test_imu_config_to_dict():
    """Test conversion of ImuConfig to dictionary."""
    config = ImuConfig(
        name="test_imu",
        prim_path="/World/IMU",
        position=np.array([1.0, 2.0, 3.0]),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
    )

    config_dict = config.to_dict()
    assert isinstance(config_dict, dict)
    assert config_dict["name"] == "test_imu"
    assert config_dict["prim_path"] == "/World/IMU"
    assert np.array_equal(config_dict["position"], np.array([1.0, 2.0, 3.0]))
    assert np.array_equal(config_dict["orientation"], np.array([0.0, 0.0, 0.0, 1.0]))


def test_imu_config_from_dict():
    """Test creation of ImuConfig from dictionary."""
    config_dict = {
        "name": "test_imu",
        "prim_path": "/World/IMU",
        "position": [1.0, 2.0, 3.0],
        "orientation": [0.0, 0.0, 0.0, 1.0],
    }

    config = ImuConfig.from_dict(config_dict)
    assert config.name == "test_imu"
    assert config.prim_path == "/World/IMU"
    assert np.array_equal(config.position, np.array([1.0, 2.0, 3.0]))
    assert np.array_equal(config.orientation, np.array([0.0, 0.0, 0.0, 1.0]))


def test_imu_sensor_config_load_from_file(tmp_path):
    """Test loading ImuSensorConfig from a JSON file."""
    # Create a temporary JSON file
    json_data = {
        "frequency": 200.0,
    }
    json_file = tmp_path / "sensor_config.json"
    import json
    with open(json_file, "w") as f:
        json.dump(json_data, f)

    # Load the configuration
    config = ImuSensorConfig.load_from_file(str(json_file))
    assert config.frequency == 200.0


def test_imu_sensor_config_export_to_file(tmp_path):
    """Test exporting ImuSensorConfig to a JSON file."""
    config = ImuSensorConfig(frequency=200.0)

    # Export to a temporary file
    json_file = tmp_path / "exported_config.json"
    config.export_to_file(json_file)

    # Verify the file exists and contains the correct data
    assert json_file.exists()
    import json
    with open(json_file, "r") as f:
        loaded_data = json.load(f)
        assert loaded_data["frequency"] == 200.0 