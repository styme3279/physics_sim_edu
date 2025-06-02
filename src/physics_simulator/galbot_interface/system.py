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
# Description: System interface for robot control
# Author: Chenyu Cao@Galbot
# Date: 2025-05-01
#
#####################################################################################

from physics_simulator.galbot_interface.config import GalbotInterfaceConfig
from physics_simulator import PhysicsSimulator
from typing import Dict, Any


class System:
    """
    Interface for system-level operations and information retrieval.
    """
    def __init__(
        self, galbot_interface_config: GalbotInterfaceConfig, simulator: PhysicsSimulator
    ):
        self.galbot_interface_config = galbot_interface_config
        self.simulator = simulator
        self.logger = simulator.logger
        self.initialized = False
        
    def initialize(self):
        """Initialize the system interface"""
        self.initialized = True
        self.logger.log_debug(f"{self.__class__.__name__} initialized")
        
    def get_simulation_time(self) -> float:
        """
        Get the current simulation time in seconds.
        
        Returns:
            float: Current simulation time in seconds
        """
        if not self.initialized:
            raise RuntimeError(f"{self.__class__.__name__} is not initialized")
        
        return self.simulator.get_simulation_time()
    
    def get_step_count(self) -> int:
        """
        Get the current simulation step count.
        
        Returns:
            int: Current simulation step count
        """
        if not self.initialized:
            raise RuntimeError(f"{self.__class__.__name__} is not initialized")
        
        return self.simulator.get_step_count()
    
    def get_system_info(self) -> Dict[str, Any]:
        """
        Get general system information.
        
        Returns:
            Dict[str, Any]: Dictionary containing system information
        """
        if not self.initialized:
            raise RuntimeError(f"{self.__class__.__name__} is not initialized")
        
        return {
            "simulation_time": self.get_simulation_time(),
            "step_count": self.get_step_count(),
            "is_running": self.simulator.is_running()
        } 