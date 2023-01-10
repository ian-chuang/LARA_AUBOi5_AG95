#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler, translation_matrix, quaternion_matrix, translation_from_matrix, quaternion_from_matrix
import numpy as np
import tf
from math import pi

planning_frame = "manipulation_frame"

def create_pose(x, y, z, roll, pitch, yaw, header=False):
    pose = geometry_msgs.msg.Pose()
    quat_tf = quaternion_from_euler(roll, pitch, yaw)
    orientation = geometry_msgs.msg.Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3])
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation = orientation

    if not header:
        return pose
    else:
        poseStamped = geometry_msgs.msg.PoseStamped()
        poseStamped.header.frame_id = planning_frame
        poseStamped.pose = pose
        return poseStamped

class Manipulation:
    def __init__(self):
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.gripper = moveit_commander.MoveGroupCommander("gripper")

        self.arm.set_max_velocity_scaling_factor(1)
        self.arm.set_max_acceleration_scaling_factor(1)

        self.arm.set_pose_reference_frame(planning_frame)
        self.arm.set_support_surface_name('table')
        
    def move_to_pose(self, pose):
        self.arm.set_pose_target(pose)

        error_code_val, plan, planning_time, error_code = self.arm.plan()

        error_code_val = self.arm.execute(plan, wait=True)
        success = (error_code_val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS)
        if not success:
            return False

        return True

    def move_gripper(self, val):
        joint_goal = self.gripper.get_current_joint_values()
        joint_goal = [val] * len(joint_goal)

        self.gripper.set_joint_value_target(joint_goal)

        error_code_val, plan, planning_time, error_code = self.gripper.plan()

        error_code_val = self.gripper.execute(plan, wait=True)
        success = (error_code_val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS)
        if not success:
            return False

        return True

    def move_cartesian_path(self, pose):
        waypoints = []
        waypoints.append(pose)

        (plan, fraction) = self.arm.compute_cartesian_path(
            waypoints,
            eef_step=0.01,
            jump_threshold=0, # 0?
            avoid_collisions=True
        )

        if fraction < 1:
            return False

        error_code_val = self.arm.execute(plan, wait=True)
        success = (error_code_val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS)
        if not success:
            return False

        return True  

    

    

        
if __name__ == '__main__':
    rospy.init_node('pick_and_place')

    manip = Manipulation()
    eef_link = manip.arm.get_end_effector_link()

    ys = [-.7, -.55, -.395, -.25, -.1, .05] 
    heights = [0.04, 0.02, 0.045, 0.008, 0.07, 0.02] 
    gripper_max = 0.93

    for y, height in zip(ys, heights):
        y = y + 0.4
        pose = create_pose(0.4, y, 0.15, 0.0, pi/2, 0.0)
        manip.move_to_pose(pose)
        manip.move_gripper(0)
        pose = create_pose(0.4, y, height, 0.0, pi/2, 0.0)
        manip.move_cartesian_path(pose)
        manip.move_gripper(gripper_max)
        pose = create_pose(0.4, y, 0.2, 0.0, pi/2, 0.0)
        manip.move_cartesian_path(pose)
        rospy.sleep(0.1)
        pose = create_pose(0.4, y, height, 0.0, pi/2, 0.0)
        manip.move_cartesian_path(pose)
        manip.move_gripper(0)
        pose = create_pose(0.4, y, 0.2, 0.0, pi/2, 0.0)
        manip.move_cartesian_path(pose)

    # y = 0.19
    # height = 0.1
    # y = y + 0.4
    # pose = create_pose(0.4, y, 0.2, 0.0, pi/2, 0.0)
    # manip.move_to_pose(pose)
    # manip.move_gripper(0)
    # pose = create_pose(0.4, y, height, 0.0, pi/2, 0.0)
    # manip.move_cartesian_path(pose)
    # manip.move_gripper(gripper_max)
    # pose = create_pose(0.4, y, 0.2, 0.0, pi/2, 0.0)
    # manip.move_cartesian_path(pose)
    # rospy.sleep(1)
    # pose = create_pose(0.4, y, height+0.01, 0.0, pi/2, 0.0)
    # manip.move_cartesian_path(pose)
    # pose = create_pose(0.4, -.3+0.4, height+0.01, 0.0, pi/2, 0.0)
    # manip.move_cartesian_path(pose)



    # # rospy.spin()

    




        
