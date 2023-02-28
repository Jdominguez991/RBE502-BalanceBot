#!/usr/bin/env python3

from __future__ import print_function


import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

from std_msgs.msg import Float64

TwistMsg = Twist

linearSpeed=0.0
twistSpeed=0.0
def callback(data):
    global linearSpeed, twistSpeed
    linearSpeed=data.linear.x*3.8
    #rospy.loginfo(rospy.get_caller_id() + " linear Speed data %f", linearSpeed)
    twistSpeed=data.angular.z*1.2
     
def run():
    global linearSpeed, twistSpeed
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('convertor', anonymous=True)

    leftPub = rospy.Publisher('/teeterbot/left_speed_cmd', Float64, queue_size=10)
    rightPub = rospy.Publisher('/teeterbot/right_speed_cmd', Float64, queue_size=10)

    rospy.Subscriber("cmd_vel", TwistMsg, callback)

    rate = rospy.Rate(10) # 10hz

    #delete when publishing for wheels
    #rospy.spin()

    leftSpeed=0.0
    rightSpeed=0.0
    while not rospy.is_shutdown():
        if(twistSpeed==0.0):
            leftSpeed=linearSpeed
            rightSpeed=linearSpeed
        elif(twistSpeed>0):
            leftSpeed=linearSpeed-abs(twistSpeed)
            rightSpeed=linearSpeed+abs(twistSpeed)
        elif(twistSpeed<0):
            leftSpeed=linearSpeed+abs(twistSpeed)
            rightSpeed=linearSpeed-abs(twistSpeed)
        #rospy.loginfo(rospy.get_caller_id() + "left wheel Speed %f", leftSpeed)
        leftPub.publish(leftSpeed)
        rightPub.publish(rightSpeed)
        rate.sleep()

if __name__ == '__main__':
    run()