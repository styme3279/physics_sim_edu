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
# Description: Sensor config for Taobao Tele Camera
# Author: Herman Ye@Galbot
# Date: 2025-04-16
#
#####################################################################################

from .basic import RgbSensorConfig
from typing import Literal, List, Optional


class TaobaoTeleCameraConfig(RgbSensorConfig):
    """
    Configuration class for the Taobao Tele Camera(640x480).

    Intrinsics of "Color" / 640x480 / {YUYV/RGB8/BGR8/RGBA8/BGRA8/Y16}:
      - frequency:               30
      - Width:                   640
      - Height:                  480
      - Fx:                      328.1441021902765
      - Fy:                      328.35625575547493
      - Cx:                      334.20459638730296
      - Cy:                      256.2106227587468
      - Pixel Size:              0.003
      - F-Stop:                  0.0
      - Focus Distance:          0.0
      - Projection Type:         fisheyePolynomial
      - Clipping Range:          [0.05, 15.0]
      - Distortion Model:        fisheye_rational_polynomial
      - Horizontal FOV:          88.56020250291559
      - Vertical FOV:            72.32714246825783
      - Diagonal FOV:            101.25365271162845
      - Distortion Coefficients: [12.029037323742996, 
                                  14.99792566030082, 
                                  -0.000433609946653566,
                                  -0.00015883038622668312, 
                                  -0.846870061589365, 
                                  12.080256018880311, 
                                  14.930336211420737,
                                  0.07965253308388849, 
                                  0.0, 
                                  0.0, 
                                  0.0, 
                                  0.0, 
                                  0.0, 
                                  0.0
                                  ]
    """

    frequency: float | None = 30.0
    width: int | None = 640
    height: int | None = 480
    fx: float | None = 328.1441021902765
    fy: float | None = 328.35625575547493
    cx: float | None = 334.20459638730296
    cy: float | None = 256.2106227587468
    pixel_size: float | None = 0.003
    f_stop: float | None = 0.0
    focus_distance: float | None = 0.0
    projection_type: str | None = "fisheyePolynomial"
    clipping_range: list[float] | None = [0.05, 15.0]
    distortion_model: str | None = "fisheye_rational_polynomial"
    horizontal_fov: float | None = 88.56020250291559
    vertical_fov: float | None = 72.32714246825783
    diagonal_fov: float | None = 101.25365271162845
    distortion_coefficients: list[float] | None = [
        12.029037323742996,
        14.99792566030082,
        -0.000433609946653566,
        -0.00015883038622668312,
        -0.846870061589365,
        12.080256018880311,
        14.930336211420737,
        0.07965253308388849,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0
    ]

