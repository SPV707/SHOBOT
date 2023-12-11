#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16
import subprocess
import threading

def publish_completion():
    rospy.init_node('completion_publisher', anonymous=True)
    completion_pub = rospy.Publisher('completion_u', Int16, queue_size=10)

    rate = rospy.Rate(1)  # Publish at a rate of 1 Hz (1 message per second)

    while not rospy.is_shutdown():
        completion_value = 100  # Set the desired completion value
        completion_pub.publish(completion_value)
        rate.sleep()
        def run_node():
            # Run the desired ROS node using rosrun
            node_name = 'picknplace'
            package_name = 'ur5_tf'
            command = ['rosrun', package_name, node_name]
            subprocess.Popen(command)
        thread = threading.Thread(target=run_node)
        thread.start()

if __name__ == '__main__':
    try:
        publish_completion()
    except rospy.ROSInterruptException:
        pass
