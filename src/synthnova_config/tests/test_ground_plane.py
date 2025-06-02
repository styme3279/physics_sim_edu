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
# Description: SynthNova Ground Plane Config Test
# Author: Herman Ye@Galbot
# Date: 2025-05-10
#
#####################################################################################

import pytest
from synthnova_config.object.ground_plane import GroundPlaneConfig


def test_ground_plane_default_values():
    """Test default values of GroundPlaneConfig."""
    config = GroundPlaneConfig()
    assert config.static_friction == 0.5
    assert config.dynamic_friction == 0.5
    assert config.restitution == 0.8
    assert config.z_position == 0.0


def test_ground_plane_custom_values():
    """Test GroundPlaneConfig with custom values."""
    config = GroundPlaneConfig(
        static_friction=0.3, dynamic_friction=0.2, restitution=0.5, z_position=1.0
    )
    assert config.static_friction == 0.3
    assert config.dynamic_friction == 0.2
    assert config.restitution == 0.5
    assert config.z_position == 1.0


def test_ground_plane_validation():
    """Test validation of GroundPlaneConfig values."""
    # Test negative friction
    with pytest.raises(ValueError):
        GroundPlaneConfig(static_friction=-0.1)

    with pytest.raises(ValueError):
        GroundPlaneConfig(dynamic_friction=-0.1)

    # Test invalid restitution
    with pytest.raises(ValueError):
        GroundPlaneConfig(restitution=1.5)  # Should be between 0 and 1


def test_ground_plane_dict_conversion():
    """Test dictionary conversion methods."""
    original_config = GroundPlaneConfig(
        static_friction=0.3, dynamic_friction=0.2, restitution=0.5, z_position=1.0
    )

    # Test to_dict
    config_dict = original_config.to_dict()
    assert isinstance(config_dict, dict)
    assert config_dict["static_friction"] == 0.3
    assert config_dict["dynamic_friction"] == 0.2
    assert config_dict["restitution"] == 0.5
    assert config_dict["z_position"] == 1.0

    # Test from_dict
    new_config = GroundPlaneConfig.from_dict(config_dict)
    assert isinstance(new_config, GroundPlaneConfig)
    assert new_config.static_friction == 0.3
    assert new_config.dynamic_friction == 0.2
    assert new_config.restitution == 0.5
    assert new_config.z_position == 1.0


def test_ground_plane_extra_fields():
    """Test that extra fields are forbidden."""
    with pytest.raises(ValueError):
        GroundPlaneConfig(extra_field="value")


def test_ground_plane_frozen():
    """Test that the config is not frozen and can be modified."""
    config = GroundPlaneConfig()
    config.static_friction = 0.4
    assert config.static_friction == 0.4
