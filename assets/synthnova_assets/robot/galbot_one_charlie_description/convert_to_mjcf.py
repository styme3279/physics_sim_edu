import os
import json
from urdf2mjcf import run
from urdf2mjcf.model import ConversionMetadata, JointParam
from urdf2mjcf.postprocess.add_sensors import add_sensors
from urdf2mjcf.postprocess.base_joint import fix_base_joint
from urdf2mjcf.postprocess.explicit_floor_contacts import add_explicit_floor_contacts
from urdf2mjcf.postprocess.flat_feet import make_feet_flat
from urdf2mjcf.postprocess.remove_redundancies import remove_redundancies
from urdf2mjcf.utils import save_xml


from urdf2mjcf.model import (
    ConversionMetadata,
    CollisionParams,
    JointParam,
    ImuSensor,
    CameraSensor,
    ForceSensor,
)


if __name__ == "__main__":
    # Get the directory of this script
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Define paths
    urdf_path = os.path.join(script_dir, "galbot_one_charlie.urdf")
    mjcf_path = os.path.join(script_dir, "galbot_one_charlie.xml")
    metadata_path = os.path.join(script_dir, "metadata.json")

    # Create collision parameters
    collision_params = CollisionParams(
        friction=[0.8, 0.02, 0.01],  # Default friction values
        condim=6,  # 6D contact model
    )

    # Define joint parameters for the arms
    joint_params = [
        # Leg joints
        JointParam(
            name="robot_joint",
            suffixes=[
                "leg_joint1",
                "leg_joint2",
                "leg_joint3",
                "leg_joint4",
                "right_arm_joint1",
                "right_arm_joint2",
                "right_arm_joint3",
                "right_arm_joint4",
                "right_arm_joint5",
                "right_arm_joint6",
                "right_arm_joint7",
                "left_arm_joint1",
                "left_arm_joint2",
                "left_arm_joint3",
                "left_arm_joint4",
                "left_arm_joint5",
                "left_arm_joint6",
                "left_arm_joint7",
                "head_joint1",
                "head_joint2",
                "left_gripper_robotiq_85_right_knuckle_joint",
                "left_gripper_robotiq_85_left_knuckle_joint",
                "left_gripper_robotiq_85_left_inner_knuckle_joint",
                "left_gripper_robotiq_85_right_inner_knuckle_joint",
                "left_gripper_robotiq_85_left_finger_tip_joint",
                "left_gripper_robotiq_85_right_finger_tip_joint",
                "right_gripper_robotiq_85_right_knuckle_joint",
                "right_gripper_robotiq_85_left_knuckle_joint",
                "right_gripper_robotiq_85_left_inner_knuckle_joint",
                "right_gripper_robotiq_85_right_inner_knuckle_joint",
                "right_gripper_robotiq_85_left_finger_tip_joint",
                "right_gripper_robotiq_85_right_finger_tip_joint",
            ],
            armature=0.1,
            frictionloss=0.1,
        ),
        # # Right arm joints
        # JointParam(
        #     name="right_arm_joint",
        #     suffixes=["1", "2", "3", "4", "5", "6", "7"],
        #     armature=0.1,
        #     frictionloss=0.1,
        # ),
        # # Left arm joints
        # JointParam(
        #     name="left_arm_joint",
        #     suffixes=["1", "2", "3", "4", "5", "6", "7"],
        #     armature=0.1,
        #     frictionloss=0.1,
        # ),
        # # Head joints
        # JointParam(
        #     name="head_joint", suffixes=["1", "2"], armature=0.1, frictionloss=0.1
        # ),
        # # Gripper joints
        # JointParam(
        #     name="right_gripper",
        #     suffixes=[
        #         "_robotiq_85_left_knuckle_joint",
        #         "_robotiq_85_right_knuckle_joint",
        #         "_robotiq_85_left_inner_knuckle_joint",
        #         "_robotiq_85_right_inner_knuckle_joint",
        #         "_robotiq_85_left_finger_tip_joint",
        #         "_robotiq_85_right_finger_tip_joint",
        #     ],
        #     armature=0.01,
        #     frictionloss=0.01,
        # ),
        # JointParam(
        #     name="left_gripper",
        #     suffixes=[
        #         "_robotiq_85_left_knuckle_joint",
        #         "_robotiq_85_right_knuckle_joint",
        #         "_robotiq_85_left_inner_knuckle_joint",
        #         "_robotiq_85_right_inner_knuckle_joint",
        #         "_robotiq_85_left_finger_tip_joint",
        #         "_robotiq_85_right_finger_tip_joint",
        #     ],
        #     armature=0.01,
        #     frictionloss=0.01,
        # ),
    ]

    # Create the ConversionMetadata instance
    galbot_one_charlie_metadata = ConversionMetadata(
        freejoint=True,  # Add a free joint for the base
        collision_params=collision_params,
        joint_params=joint_params,
        remove_redundancies=False,
        floating_base=False,
        maxhullvert=None,  # No limit on convex hull vertices
        angle="radian",  # Use radians for angles
    )

    # Run the conversion
    run(
        urdf_path=urdf_path,
        mjcf_path=mjcf_path,
        copy_meshes=False,
        metadata=galbot_one_charlie_metadata,
    )
