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
# Description: LiDAR config for Livox Mid-360
# Author: Herman Ye@Galbot
# Date: 2025-04-16
#
#####################################################################################

import numpy as np
from .basic.lidar_3d import Lidar3DSensorConfig
from typing import List


class LivoxMid360LidarSensorConfig(Lidar3DSensorConfig):
    """
    Configuration class for the Livox Mid-360 LiDAR sensor.

    The Livox Mid-360 is a non-repetitive scanning LiDAR that can be approximated as a 128-line
    mechanical spinning LiDAR in the Galbot Sim environment. This simplification helps represent
    its scanning behavior as a mechanical lidar with rotational scanning.

    Note:
        The configuration is based on the official Livox Mid-360 specifications and documentation.
        The sensor uses a non-repetitive scanning pattern, but is approximated as a mechanical
        spinning LiDAR for simulation purposes.
        - Official Livox Mid-360 Specs: https://www.livoxtech.com/cn/mid-360/specs
        - Livox Mid-360 User Manual (Chinese): https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/Livox/Livox_Mid-360_User_Manual_CHS.pdf
        - Livox Scanning Pattern Overview: https://livox-wiki-cn.readthedocs.io/zh-cn/latest/introduction/livox_scanning_pattern.html#id3
        - Related research paper on Livox lidar: https://arxiv.org/abs/2006.11034
    """

    scan_type: str = "rotary"
    rotation_frequency: int = 10  # Standard mode rotation frequency
    valid_range: List[float] = [0.1, 40.0]  # [min_range, max_range] in meters
    min_reflectance: float = 0.1  # 10% minimum reflectance
    min_reflectance_range: float = 40.0  # Maximum range for 10% reflectivity
    avg_power: float = 6.5  # Power consumption in watts
    wave_length: float = 905.0  # Wavelength in nanometers
    pulse_time: float = 6e-9  # Pulse time in seconds (6 nanoseconds)
    firing_frequency: float = 24000.0  # Firing frequency in Hz
    num_of_point_cloud_per_data_frame: int = 19968  # 96*208 points per frame
    num_of_emitters: int = 128  # Number of laser emitters
    start_azimuth_deg: float = 0.0  # Start azimuth angle
    end_azimuth_deg: float = 360.0  # End azimuth angle
    up_elevation_deg: float = 52.0  # Maximum elevation angle
    down_elevation_deg: float = -7.0  # Minimum elevation angle
    default_fire_time_ns_dt: float = 762.0  # Default time offset for pulse firing
    azimuth_error_mean: float = 0.0  # Mean azimuth error
    azimuth_error_std: float = 0.015  # Standard deviation of azimuth error
    elevation_error_mean: float = 0.0  # Mean elevation error
    elevation_error_std: float = 0.015  # Standard deviation of elevation error
