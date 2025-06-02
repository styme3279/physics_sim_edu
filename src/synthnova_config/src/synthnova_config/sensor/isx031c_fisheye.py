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
# Description: Sensor config for ISX031C Fisheye Camera
# Author: Herman Ye@Galbot
# Date: 2025-03-06
#
#####################################################################################

from .basic import RgbSensorConfig
from typing import Literal, List


class Isx031cFisheyeSensorConfig(RgbSensorConfig):
    """
    Configuration class for the ISX031C Fisheye camera.

    Intrinsics:
      - Width:         1920
      - Height:        1536
      - PPX:           956.5711241703
      - PPY:           762.371411956
      - Fx:            505.8954167817
      - Fy:            506.7242139608
      - Distortion:    Fisheye Kannala-Brandt
      - Coeffs:        [0.1360692895, -0.0357720711, 0.0005297636, 0.0002798558]
      - FOV (deg):     196 x 154 (diagonal: 210)
    """

    frequency: int = 30
    width: int = 1920
    height: int = 1536
    fx: float = 505.8954167817
    fy: float = 506.7242139608
    cx: float = 956.5711241703
    cy: float = 762.371411956
    pixel_size: float = 0.003
    f_stop: float = 0.0
    focus_distance: float = 0.0
    projection_type: Literal["fisheyePolynomial"] = "fisheyePolynomial"
    clipping_range: List[float] = [0.05, 10000000.0]
    distortion_model: Literal["fisheye_kannala_brandt"] = "fisheye_kannala_brandt"
    distortion_coefficients: List[float] = [
        0.1360692895,
        -0.0357720711,
        0.0005297636,
        0.0002798558,
    ]
    horizontal_fov: float = 196.0
    vertical_fov: float = 154.0
    diagonal_fov: float = 210.0
