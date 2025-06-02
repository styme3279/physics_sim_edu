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
# Description: Main Galbot interface class for robot control
# Author: Chenyu Cao@Galbot
# Date: 2025-05-01
#
#####################################################################################

from physics_simulator.galbot_interface.config import GalbotInterfaceConfig
from physics_simulator import PhysicsSimulator

class GalbotInterface:
    def __init__(self,
                 galbot_interface_config: GalbotInterfaceConfig,
                 simulator: PhysicsSimulator):
        """
        Initialize GalbotInterface with enabled modules from config.

        Args:
            galbot_interface_config (GalbotInterfaceConfig): Configuration for the interface
            simulator (PhysicsSimulator): Simulator instance
        """
        from physics_simulator.galbot_interface.left_arm import LeftArm
        from physics_simulator.galbot_interface.right_arm import RightArm
        from physics_simulator.galbot_interface.head import Head
        from physics_simulator.galbot_interface.leg import Leg
        from physics_simulator.galbot_interface.left_gripper import LeftGripper
        from physics_simulator.galbot_interface.right_gripper import RightGripper
        from physics_simulator.galbot_interface.left_wrist_camera import LeftWristCamera
        from physics_simulator.galbot_interface.right_wrist_camera import RightWristCamera
        from physics_simulator.galbot_interface.front_head_camera import FrontHeadCamera
        from physics_simulator.galbot_interface.chassis import Chassis

        self.initialized = False
        self.simulator = simulator
        self.logger = simulator.logger
        self.galbot_interface_config = galbot_interface_config
        
        # Map module names to their corresponding classes
        self.module_map = {
            "chassis": Chassis,
            "left_arm": LeftArm,
            "right_arm": RightArm,
            "leg": Leg,
            "head": Head,
            "left_gripper": LeftGripper,
            "right_gripper": RightGripper,
            "left_wrist_camera": LeftWristCamera,
            "right_wrist_camera": RightWristCamera,
            "front_head_camera": FrontHeadCamera,
        }

        # Initialize enabled modules
        enabled_modules = galbot_interface_config.modules_manager.enabled_modules

        for module_name in enabled_modules:
            if module_name in self.module_map:
                self.logger.log_debug(f"Initializing module [{module_name}]")
                # Create instance of the module
                module_instance = self.module_map[module_name](
                    galbot_interface_config=galbot_interface_config, simulator=simulator
                )
                # Set the module as an attribute of the interface
                setattr(self, module_name, module_instance)
            else:
                simulator.logger.log_warning(
                    f"Unknown module {module_name} specified in enabled_modules"
                )

    def initialize(self):
        """Initialize all enabled modules"""
        enabled_modules = self.galbot_interface_config.modules_manager.enabled_modules
        for module_name in enabled_modules:
            if module_name in self.module_map:
                module = getattr(self, module_name)
                module.initialize()
                self.logger.log_success(f"Module [{module_name}] initialized")
        self.initialized = True
        self.logger.log_success("Galbot Interface initialized") 