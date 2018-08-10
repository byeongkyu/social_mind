#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy

class TurnDetectorNode:
    def __init__(self):
        rospy.loginfo('\033[92m[%s]\033[0m initialized...'%rospy.get_name())
        pass


if __name__ == '__main__':
    rospy.init_node('turn_detector', anonymous=False)
    m = TurnDetectorNode()
    rospy.spin()