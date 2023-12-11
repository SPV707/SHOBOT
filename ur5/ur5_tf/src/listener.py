#!/usr/bin/env python  
import rospy

import math
import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('tf2_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('base_link', 'tag_6', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        print('translation vector: (',trans.transform.translation.x,',',trans.transform.translation.y,',',trans.transform.translation.z,')')


        rate.sleep()