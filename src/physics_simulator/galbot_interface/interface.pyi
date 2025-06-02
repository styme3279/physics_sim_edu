from typing import Dict, Type, Optional

from physics_simulator import PhysicsSimulator
from physics_simulator.galbot_interface.config import GalbotInterfaceConfig
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
from physics_simulator.galbot_interface.basic_part import BasicLimb

class GalbotInterface:
    initialized: bool
    simulator: PhysicsSimulator
    logger: any
    galbot_interface_config: GalbotInterfaceConfig
    module_map: Dict[str, Type[BasicLimb]]
    
    # Module attributes with type annotations
    chassis: Optional[Chassis]
    left_arm: Optional[LeftArm]
    right_arm: Optional[RightArm]
    leg: Optional[Leg]
    head: Optional[Head]
    left_gripper: Optional[LeftGripper]
    right_gripper: Optional[RightGripper]
    left_wrist_camera: Optional[LeftWristCamera]
    right_wrist_camera: Optional[RightWristCamera]
    front_head_camera: Optional[FrontHeadCamera]
    
    def __init__(self, galbot_interface_config: GalbotInterfaceConfig, simulator: PhysicsSimulator) -> None: ...
    def initialize(self) -> None: ... 