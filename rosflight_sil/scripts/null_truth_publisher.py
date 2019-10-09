#!/usr/bin/python

import rospy
from nav_msgs.msg import Odometry

if __name__ == '__main__':

    # initialize
    rospy.init_node('truth_pub')
    pub = rospy.Publisher('uav_truth_NED', Odometry, queue_size=1)

    # calibrate IMU

    # main loop
    update_freq = 1000 # Hz
    rate = rospy.Rate(update_freq)

    msg = Odometry()

    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()
