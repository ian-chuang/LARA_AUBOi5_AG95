#! /usr/bin/env python

import rospy
import geometry_msgs.msg   
import sensor_msgs.msg 
from aubo_msgs.msg import TeachCommand
from dh_gripper_msgs.msg import GripperCtrl
from threading import Lock
from copy import deepcopy

class AuboJoyTeleop():
    def __init__(self):
        self.teach_pub = rospy.Publisher('/teach', TeachCommand, queue_size=1)
        self.gripper_pub = rospy.Publisher('/gripper/ctrl', GripperCtrl, queue_size=1)
        self.joy_sub = rospy.Subscriber("/joy", sensor_msgs.msg.Joy, self.callback)
        
        self.last_teach_command= None

        self.joy_msg = None
        self.joy_msg_mutex = Lock()

    def publish(self):
        self.joy_msg_mutex.acquire()
        msg = deepcopy(self.joy_msg)
        self.joy_msg_mutex.release()

        if msg != None:

            a = msg.buttons[0]
            b = msg.buttons[1]
            x = msg.buttons[2]
            y = msg.buttons[3]
            lb = msg.buttons[4]
            rb = msg.buttons[5]
            # ljoy_x = msg.axes[0]
            # ljoy_y = msg.axes[1]
            # rjoy_x = msg.axes[3]
            # rjoy_y = msg.axes[4]
            dpad_x = msg.axes[6]
            dpad_y = msg.axes[7]

            mov_x_plus = dpad_y == 1
            mov_x_minus = dpad_y == -1
            mov_y_plus = dpad_x == 1
            mov_y_minus = dpad_x == -1
            mov_z_plus = rb == 1
            mov_z_minus = lb == 1
            rot_z_plus = x == 1
            rot_z_minus = y == 1
            close_gripper = a == 1
            open_gripper = b == 1

            num_commands = sum([mov_x_plus, mov_x_minus, mov_y_plus, mov_y_minus, mov_z_plus, mov_z_minus, rot_z_plus, rot_z_minus])
            
            msg = TeachCommand()
            if num_commands != 1:
                msg.command = TeachCommand.STOP
            elif mov_x_plus:
                msg.command = TeachCommand.MOV_X
                msg.direction = TeachCommand.POSITIVE
            elif mov_x_minus:
                msg.command = TeachCommand.MOV_X
                msg.direction = TeachCommand.NEGATIVE
            elif mov_y_plus:
                msg.command = TeachCommand.MOV_Y
                msg.direction = TeachCommand.POSITIVE
            elif mov_y_minus:
                msg.command = TeachCommand.MOV_Y
                msg.direction = TeachCommand.NEGATIVE
            elif mov_z_plus:
                msg.command = TeachCommand.MOV_Z
                msg.direction = TeachCommand.POSITIVE
            elif mov_z_minus:
                msg.command = TeachCommand.MOV_Z
                msg.direction = TeachCommand.NEGATIVE
            elif rot_z_plus:
                msg.command = TeachCommand.ROT_Z
                msg.direction = TeachCommand.POSITIVE
            elif rot_z_minus:
                msg.command = TeachCommand.ROT_Z
                msg.direction = TeachCommand.NEGATIVE

            self.teach_pub.publish(msg)
            

            num_commands = sum([close_gripper, open_gripper])

            msg = GripperCtrl()
            msg.initialize = False
            msg.speed = 100
            msg.force = 100
            if num_commands != 1:
                pass
            elif close_gripper:
                msg.position = 0
                self.gripper_pub.publish(msg)
            elif open_gripper:
                msg.position = 1000
                self.gripper_pub.publish(msg)
                

    def callback(self, msg):
        self.joy_msg_mutex.acquire()
        self.joy_msg = msg
        self.joy_msg_mutex.release()
        


if __name__ == '__main__':
    rospy.init_node('aubo_joy_teleop')

    aubo_joy_teleop = AuboJoyTeleop()

    rate = rospy.Rate(60) # 10hz
    while not rospy.is_shutdown():
        aubo_joy_teleop.publish()
        rate.sleep()
    
    rospy.spin()