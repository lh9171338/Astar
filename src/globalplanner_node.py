#! /usr/bin/env python
import rospy
from globalplanner import GlobalPlanner


if __name__ == '__main__':
    # Initial node
    rospy.init_node('globalplanner', anonymous=True)
    rospy.loginfo('Start the globalplanner node')

    globalplanner = GlobalPlanner()
    if globalplanner.setup():
        globalplanner.spin()
