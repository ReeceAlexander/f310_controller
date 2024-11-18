#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
cmd = Twist()
turning = False

def callback(msg):
    if msg.data == "forward" and turning == False:
        cmd.linear.x = 0.5
        pub.publish(cmd)
        rospy.sleep(0.05)

    elif msg.data == "reverse" and turning == False:
        cmd.linear.x = -0.5
        pub.publish(cmd)
        rospy.sleep(0.05)

def callback_two(msg):
    global turning
    
    if msg.angular.z != 0.0:
        turning = True
        
    else:
        turning = False

def main():
    rospy.init_node('cruise_cntl_node')

    rospy.Subscriber('/cruise_cntl', String, callback, queue_size=1)

    rospy.Subscriber('/cmd_vel', Twist, callback_two, queue_size=1)

    rospy.spin()

if __name__ == "__main__":

    main()