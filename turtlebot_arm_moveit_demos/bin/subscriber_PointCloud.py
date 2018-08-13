#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2

def callbackPointCloud(data):
    rospy.loginfo(rospy.get_caller_id() + " Received Point Cloud: %s", data.is_dense)
    # pc = PointCloud2()
    # pc.is
    
def listener():
    rospy.init_node('subscriber2PointCloud', anonymous=True)
    rospy.Subscriber("/camera/depth/points", PointCloud2, callbackPointCloud)

    print "============ starting listening to PointCloud topic"
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSException as e:
        print(e)