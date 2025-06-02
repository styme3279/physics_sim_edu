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
# Description: Test cases for 3D LiDAR configuration
# Author: Herman Ye@Galbot
# Date: 2025-03-06
#
#####################################################################################

import pytest
import numpy as np
from pathlib import Path
from synthnova_config.sensor.basic.lidar_3d import Lidar3DConfig, Lidar3DSensorConfig


def test_lidar_3d_sensor_config_creation():
    """Test basic creation of Lidar3DSensorConfig with default values."""
    config = Lidar3DSensorConfig()
    assert config.scan_type is None
    assert config.rotation_frequency is None
    assert config.valid_range is None
    assert config.min_reflectance is None
    assert config.min_reflectance_range is None
    assert config.avg_power is None
    assert config.wave_length is None
    assert config.pulse_time is None
    assert config.firing_frequency is None
    assert config.num_of_point_cloud_per_data_frame is None
    assert config.num_of_emitters is None
    assert config.start_azimuth_deg is None
    assert config.end_azimuth_deg is None
    assert config.up_elevation_deg is None
    assert config.down_elevation_deg is None
    assert config.default_fire_time_ns_dt is None
    assert config.azimuth_error_mean is None
    assert config.azimuth_error_std is None
    assert config.elevation_error_mean is None
    assert config.elevation_error_std is None


def test_lidar_3d_sensor_config_with_values():
    """Test Lidar3DSensorConfig creation with specific values."""
    config = Lidar3DSensorConfig(
        scan_type="rotary",
        rotation_frequency=10.0,
        valid_range=np.array([0.1, 40.0]),
        min_reflectance=0.1,
        min_reflectance_range=40.0,
        avg_power=6.5,
        wave_length=905.0,
        pulse_time=6e-9,
        firing_frequency=24000.0,
        num_of_point_cloud_per_data_frame=19968,
        num_of_emitters=128,
        start_azimuth_deg=0.0,
        end_azimuth_deg=360.0,
        up_elevation_deg=52.0,
        down_elevation_deg=-7.0,
        default_fire_time_ns_dt=762.0,
        azimuth_error_mean=0.0,
        azimuth_error_std=0.015,
        elevation_error_mean=0.0,
        elevation_error_std=0.015,
    )

    assert config.scan_type == "rotary"
    assert config.rotation_frequency == 10.0
    assert np.array_equal(config.valid_range, np.array([0.1, 40.0]))
    assert config.min_reflectance == 0.1
    assert config.min_reflectance_range == 40.0
    assert config.avg_power == 6.5
    assert config.wave_length == 905.0
    assert config.pulse_time == 6e-9
    assert config.firing_frequency == 24000.0
    assert config.num_of_point_cloud_per_data_frame == 19968
    assert config.num_of_emitters == 128
    assert config.start_azimuth_deg == 0.0
    assert config.end_azimuth_deg == 360.0
    assert config.up_elevation_deg == 52.0
    assert config.down_elevation_deg == -7.0
    assert config.default_fire_time_ns_dt == 762.0
    assert config.azimuth_error_mean == 0.0
    assert config.azimuth_error_std == 0.015
    assert config.elevation_error_mean == 0.0
    assert config.elevation_error_std == 0.015


def test_lidar_3d_sensor_config_validation():
    """Test validation of Lidar3DSensorConfig parameters."""
    # Test valid range validation
    with pytest.raises(ValueError, match="Valid range must have exactly 2 components"):
        Lidar3DSensorConfig(valid_range=np.array([0.1]))

    with pytest.raises(ValueError, match="min_range must be less than max_range"):
        Lidar3DSensorConfig(valid_range=np.array([40.0, 0.1]))

    with pytest.raises(ValueError, match="min_range must be non-negative"):
        Lidar3DSensorConfig(valid_range=np.array([-0.1, 40.0]))

    # Test min_reflectance validation
    with pytest.raises(ValueError, match="Minimum reflectance must be between 0 and 1"):
        Lidar3DSensorConfig(min_reflectance=1.5)

    # Test azimuth range validation
    with pytest.raises(ValueError, match="Azimuth angle must be between 0 and 360 degrees"):
        Lidar3DSensorConfig(start_azimuth_deg=-1.0)
    with pytest.raises(ValueError, match="Azimuth angle must be between 0 and 360 degrees"):
        Lidar3DSensorConfig(end_azimuth_deg=361.0)

    # Test elevation range validation
    with pytest.raises(ValueError, match="Elevation angle must be between -90 and 90 degrees"):
        Lidar3DSensorConfig(up_elevation_deg=91.0)
    with pytest.raises(ValueError, match="Elevation angle must be between -90 and 90 degrees"):
        Lidar3DSensorConfig(down_elevation_deg=-91.0)

    # Test frequency validation
    with pytest.raises(ValueError, match="Frequency must be positive"):
        Lidar3DSensorConfig(rotation_frequency=-1.0)
    with pytest.raises(ValueError, match="Frequency must be positive"):
        Lidar3DSensorConfig(firing_frequency=-1.0)

    # Test number of emitters validation
    with pytest.raises(ValueError, match="Number of emitters must be positive"):
        Lidar3DSensorConfig(num_of_emitters=-1)
    with pytest.raises(ValueError, match="Number of emitters should typically be a multiple of 8"):
        Lidar3DSensorConfig(num_of_emitters=10)

    # Test number of point clouds validation
    with pytest.raises(ValueError, match="Number of point clouds per data frame must be positive"):
        Lidar3DSensorConfig(num_of_point_cloud_per_data_frame=-1)


def test_lidar_3d_config_creation():
    """Test basic creation of Lidar3DConfig with required fields."""
    config = Lidar3DConfig(
        name="test_lidar",
        prim_path="/World/Lidar",
    )

    assert config.type == "lidar_3d"
    assert config.name == "test_lidar"
    assert config.prim_path == "/World/Lidar"
    assert config.uuid is not None
    assert config.position is None
    assert config.orientation is None
    assert config.translation is None
    assert config.rotation is None
    assert isinstance(config.sensor_config, Lidar3DSensorConfig)


def test_lidar_3d_config_with_global_pose():
    """Test Lidar3DConfig creation with global pose."""
    config = Lidar3DConfig(
        name="test_lidar",
        prim_path="/World/Lidar",
        position=np.array([1.0, 2.0, 3.0]),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
    )

    assert np.array_equal(config.position, np.array([1.0, 2.0, 3.0]))
    assert np.array_equal(config.orientation, np.array([0.0, 0.0, 0.0, 1.0]))
    assert config.translation is None
    assert config.rotation is None


def test_lidar_3d_config_with_local_pose():
    """Test Lidar3DConfig creation with local pose."""
    config = Lidar3DConfig(
        name="test_lidar",
        prim_path="/World/Lidar",
        translation=np.array([1.0, 2.0, 3.0]),
        rotation=np.array([0.0, 0.0, 0.0, 1.0]),
    )

    assert config.position is None
    assert config.orientation is None
    assert np.array_equal(config.translation, np.array([1.0, 2.0, 3.0]))
    assert np.array_equal(config.rotation, np.array([0.0, 0.0, 0.0, 1.0]))


def test_lidar_3d_config_pose_validation():
    """Test that Lidar3DConfig raises error when both global and local poses are provided."""
    with pytest.raises(ValueError, match="Cannot use both global \\(position/orientation\\) and local \\(translation/rotation\\) coordinates simultaneously"):
        Lidar3DConfig(
            name="test_lidar",
            prim_path="/World/Lidar",
            position=np.array([1.0, 2.0, 3.0]),
            translation=np.array([1.0, 2.0, 3.0]),
        )


def test_lidar_3d_config_uuid_format():
    """Test UUID format validation in Lidar3DConfig."""
    config = Lidar3DConfig(
        name="test_lidar",
        prim_path="/World/Lidar",
        uuid="12345678-1234-5678-1234-567812345678",
    )

    assert config.uuid == "12345678_1234_5678_1234_567812345678"


def test_lidar_3d_config_invalid_uuid():
    """Test that Lidar3DConfig raises error for invalid UUID format."""
    with pytest.raises(ValueError, match="Invalid UUID format"):
        Lidar3DConfig(
            name="test_lidar",
            prim_path="/World/Lidar",
            uuid="invalid-uuid",
        )


def test_lidar_3d_config_to_dict():
    """Test conversion of Lidar3DConfig to dictionary."""
    config = Lidar3DConfig(
        name="test_lidar",
        prim_path="/World/Lidar",
        position=np.array([1.0, 2.0, 3.0]),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
    )

    config_dict = config.to_dict()
    assert isinstance(config_dict, dict)
    assert config_dict["name"] == "test_lidar"
    assert config_dict["prim_path"] == "/World/Lidar"
    assert np.array_equal(config_dict["position"], np.array([1.0, 2.0, 3.0]))
    assert np.array_equal(config_dict["orientation"], np.array([0.0, 0.0, 0.0, 1.0]))


def test_lidar_3d_config_from_dict():
    """Test creation of Lidar3DConfig from dictionary."""
    config_dict = {
        "name": "test_lidar",
        "prim_path": "/World/Lidar",
        "position": [1.0, 2.0, 3.0],
        "orientation": [0.0, 0.0, 0.0, 1.0],
    }

    config = Lidar3DConfig.from_dict(config_dict)
    assert config.name == "test_lidar"
    assert config.prim_path == "/World/Lidar"
    assert np.array_equal(config.position, np.array([1.0, 2.0, 3.0]))
    assert np.array_equal(config.orientation, np.array([0.0, 0.0, 0.0, 1.0]))


def test_lidar_3d_sensor_config_load_from_file(tmp_path):
    """Test loading Lidar3DSensorConfig from a JSON file."""
    # Create a temporary JSON file
    json_data = {
        "scan_type": "rotary",
        "rotation_frequency": 10.0,
        "valid_range": [0.1, 40.0],
        "min_reflectance": 0.1,
        "min_reflectance_range": 40.0,
    }
    json_file = tmp_path / "sensor_config.json"
    import json
    with open(json_file, "w") as f:
        json.dump(json_data, f)

    # Load the configuration
    config = Lidar3DSensorConfig.load_from_file(str(json_file))
    assert config.scan_type == "rotary"
    assert config.rotation_frequency == 10.0
    assert np.array_equal(config.valid_range, np.array([0.1, 40.0]))
    assert config.min_reflectance == 0.1
    assert config.min_reflectance_range == 40.0


def test_lidar_3d_sensor_config_export_to_file(tmp_path):
    """Test exporting Lidar3DSensorConfig to a JSON file."""
    config = Lidar3DSensorConfig(
        scan_type="rotary",
        rotation_frequency=10.0,
        valid_range=np.array([0.1, 40.0]),
        min_reflectance=0.1,
        min_reflectance_range=40.0,
    )

    # Export to a temporary file
    json_file = tmp_path / "exported_config.json"
    config.export_to_file(json_file)

    # Verify the file exists and contains the correct data
    assert json_file.exists()
    import json
    with open(json_file, "r") as f:
        loaded_data = json.load(f)
        assert loaded_data["scan_type"] == "rotary"
        assert loaded_data["rotation_frequency"] == 10.0
        assert loaded_data["valid_range"] == [0.1, 40.0]
        assert loaded_data["min_reflectance"] == 0.1
        assert loaded_data["min_reflectance_range"] == 40.0 