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

    y = -.55
    height = 0.02
    gripper_max = 0.93

    y = y + 0.4

    
    manip = Manipulation()
    eef_link = manip.arm.get_end_effector_link()

    rate = rospy.Rate(2)

    rate.sleep()
    pose = create_pose(0.4, y, 0.012, 0, 0, 0, header=True)
    manip.scene.add_box('cube', pose, size=(0.02, 0.02, 0.02))
    rate.sleep()

    pose = create_pose(0.4, y, 0.15, 0.0, pi/2, 0.0)
    manip.move_to_pose(pose)

    manip.move_gripper(0)
    
    pose = create_pose(0.4, y, height, 0.0, pi/2, 0.0)
    manip.move_cartesian_path(pose)

    rate.sleep()
    grasping_group = 'gripper'
    touch_links = manip.robot.get_link_names(group=grasping_group)
    manip.scene.attach_box(eef_link, 'cube', touch_links=touch_links)
    rate.sleep()

    manip.move_gripper(gripper_max)

    pose = create_pose(0.4, y, 0.2, 0.0, pi/2, 0.0)
    manip.move_cartesian_path(pose)

    rate.sleep()
    manip.scene.remove_attached_object(eef_link, name='cube')
    rate.sleep()
    manip.scene.remove_world_object('cube')
    rate.sleep()

    rospy.sleep(3)

    manip.move_gripper(0)


    # # rospy.spin()

    




        
