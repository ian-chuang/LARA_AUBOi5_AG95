#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped, Twist


    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('convert_twist_to_twist_stamped', anonymous=True)

    pub = rospy.Publisher('/servo_server/cmd_vel', TwistStamped, queue_size=10)

    def callback(msg):
        stamped = TwistStamped()
        stamped.twist = msg
        stamped.header.stamp = rospy.Time.now()
        pub.publish(stamped)

    rospy.Subscriber("/cmd_vel", Twist, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()