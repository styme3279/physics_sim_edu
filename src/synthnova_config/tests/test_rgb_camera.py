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
# Description: Test cases for RGB camera configuration
# Author: Herman Ye@Galbot
# Date: 2025-03-06
#
#####################################################################################

import pytest
import numpy as np
from pathlib import Path
from synthnova_config.sensor.basic.rgb_camera import RgbCameraConfig, RgbSensorConfig


def test_rgb_sensor_config_creation():
    """Test basic creation of RgbSensorConfig with default values."""
    config = RgbSensorConfig()
    assert config.frequency is None
    assert config.width is None
    assert config.height is None
    assert config.fx is None
    assert config.fy is None
    assert config.cx is None
    assert config.cy is None
    assert config.distortion_model is None
    assert config.distortion_coefficients is None
    assert config.pixel_size is None
    assert config.f_stop is None
    assert config.focus_distance is None
    assert config.projection_type is None
    assert np.array_equal(config.clipping_range, np.array([0.05, 30.0]))
    assert config.horizontal_fov is None
    assert config.vertical_fov is None
    assert config.diagonal_fov is None


def test_rgb_sensor_config_with_values():
    """Test RgbSensorConfig creation with specific values."""
    config = RgbSensorConfig(
        frequency=30.0,
        width=1920,
        height=1080,
        fx=1000.0,
        fy=1000.0,
        cx=960.0,
        cy=540.0,
        distortion_model="plumb_bob",
        distortion_coefficients=[0.1, 0.1, 0.0, 0.0, 0.0],
        pixel_size=0.003,
        f_stop=2.8,
        focus_distance=1.0,
        projection_type="pinhole",
        clipping_range=np.array([0.1, 20.0]),
        horizontal_fov=90.0,
        vertical_fov=60.0,
        diagonal_fov=110.0,
    )

    assert config.frequency == 30.0
    assert config.width == 1920
    assert config.height == 1080
    assert config.fx == 1000.0
    assert config.fy == 1000.0
    assert config.cx == 960.0
    assert config.cy == 540.0
    assert config.distortion_model == "plumb_bob"
    assert config.distortion_coefficients == [0.1, 0.1, 0.0, 0.0, 0.0]
    assert config.pixel_size == 0.003
    assert config.f_stop == 2.8
    assert config.focus_distance == 1.0
    assert config.projection_type == "pinhole"
    assert np.array_equal(config.clipping_range, np.array([0.1, 20.0]))
    assert config.horizontal_fov == 90.0
    assert config.vertical_fov == 60.0
    assert config.diagonal_fov == 110.0


def test_rgb_camera_config_creation():
    """Test basic creation of RgbCameraConfig with required fields."""
    config = RgbCameraConfig(
        name="test_camera",
        prim_path="/World/Camera",
    )

    assert config.type == "rgb_camera"
    assert config.name == "test_camera"
    assert config.prim_path == "/World/Camera"
    assert config.uuid is not None
    assert config.position is None
    assert config.orientation is None
    assert config.translation is None
    assert config.rotation is None
    assert config.viewport_name is None
    assert config.camera_axes == "world"
    assert isinstance(config.sensor_config, RgbSensorConfig)


def test_rgb_camera_config_with_global_pose():
    """Test RgbCameraConfig creation with global pose."""
    config = RgbCameraConfig(
        name="test_camera",
        prim_path="/World/Camera",
        position=np.array([1.0, 2.0, 3.0]),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
    )

    assert np.array_equal(config.position, np.array([1.0, 2.0, 3.0]))
    assert np.array_equal(config.orientation, np.array([0.0, 0.0, 0.0, 1.0]))
    assert config.translation is None
    assert config.rotation is None


def test_rgb_camera_config_with_local_pose():
    """Test RgbCameraConfig creation with local pose."""
    config = RgbCameraConfig(
        name="test_camera",
        prim_path="/World/Camera",
        translation=np.array([1.0, 2.0, 3.0]),
        rotation=np.array([0.0, 0.0, 0.0, 1.0]),
    )

    assert config.position is None
    assert config.orientation is None
    assert np.array_equal(config.translation, np.array([1.0, 2.0, 3.0]))
    assert np.array_equal(config.rotation, np.array([0.0, 0.0, 0.0, 1.0]))


def test_rgb_camera_config_pose_validation():
    """Test that RgbCameraConfig raises error when both global and local poses are provided."""
    with pytest.raises(ValueError, match="Cannot use both global \\(position/orientation\\) and local \\(translation/rotation\\) coordinates simultaneously"):
        RgbCameraConfig(
            name="test_camera",
            prim_path="/World/Camera",
            position=np.array([1.0, 2.0, 3.0]),
            translation=np.array([1.0, 2.0, 3.0]),
        )


def test_rgb_camera_config_uuid_format():
    """Test UUID format validation in RgbCameraConfig."""
    config = RgbCameraConfig(
        name="test_camera",
        prim_path="/World/Camera",
        uuid="12345678-1234-5678-1234-567812345678",
    )

    assert config.uuid == "12345678_1234_5678_1234_567812345678"


def test_rgb_camera_config_invalid_uuid():
    """Test that RgbCameraConfig raises error for invalid UUID format."""
    with pytest.raises(ValueError, match="Invalid UUID format"):
        RgbCameraConfig(
            name="test_camera",
            prim_path="/World/Camera",
            uuid="invalid-uuid",
        )


def test_rgb_camera_config_to_dict():
    """Test conversion of RgbCameraConfig to dictionary."""
    config = RgbCameraConfig(
        name="test_camera",
        prim_path="/World/Camera",
        position=np.array([1.0, 2.0, 3.0]),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
    )

    config_dict = config.to_dict()
    assert isinstance(config_dict, dict)
    assert config_dict["name"] == "test_camera"
    assert config_dict["prim_path"] == "/World/Camera"
    assert np.array_equal(config_dict["position"], np.array([1.0, 2.0, 3.0]))
    assert np.array_equal(config_dict["orientation"], np.array([0.0, 0.0, 0.0, 1.0]))


def test_rgb_camera_config_from_dict():
    """Test creation of RgbCameraConfig from dictionary."""
    config_dict = {
        "name": "test_camera",
        "prim_path": "/World/Camera",
        "position": [1.0, 2.0, 3.0],
        "orientation": [0.0, 0.0, 0.0, 1.0],
    }

    config = RgbCameraConfig.from_dict(config_dict)
    assert config.name == "test_camera"
    assert config.prim_path == "/World/Camera"
    assert np.array_equal(config.position, np.array([1.0, 2.0, 3.0]))
    assert np.array_equal(config.orientation, np.array([0.0, 0.0, 0.0, 1.0]))


def test_rgb_sensor_config_load_from_file(tmp_path):
    """Test loading RgbSensorConfig from a JSON file."""
    # Create a temporary JSON file
    json_data = {
        "frequency": 30.0,
        "width": 1920,
        "height": 1080,
        "fx": 1000.0,
        "fy": 1000.0,
        "cx": 960.0,
        "cy": 540.0,
    }
    json_file = tmp_path / "sensor_config.json"
    import json
    with open(json_file, "w") as f:
        json.dump(json_data, f)

    # Load the configuration
    config = RgbSensorConfig.load_from_file(str(json_file))
    assert config.frequency == 30.0
    assert config.width == 1920
    assert config.height == 1080
    assert config.fx == 1000.0
    assert config.fy == 1000.0
    assert config.cx == 960.0
    assert config.cy == 540.0


def test_rgb_sensor_config_export_to_file(tmp_path):
    """Test exporting RgbSensorConfig to a JSON file."""
    config = RgbSensorConfig(
        frequency=30.0,
        width=1920,
        height=1080,
        fx=1000.0,
        fy=1000.0,
        cx=960.0,
        cy=540.0,
    )

    # Export to a temporary file
    json_file = tmp_path / "exported_config.json"
    config.export_to_file(json_file)

    # Verify the file exists and contains the correct data
    assert json_file.exists()
    import json
    with open(json_file, "r") as f:
        loaded_data = json.load(f)
        assert loaded_data["frequency"] == 30.0
        assert loaded_data["width"] == 1920
        assert loaded_data["height"] == 1080
        assert loaded_data["fx"] == 1000.0
        assert loaded_data["fy"] == 1000.0
        assert loaded_data["cx"] == 960.0
        assert loaded_data["cy"] == 540.0 