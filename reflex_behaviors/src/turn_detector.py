#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy
from std_msgs.msg import Bool, String, Empty
from mind_msgs.msg import RaisingEvents

class TurnDetectorNode:
    def __init__(self):
        rospy.Subscriber('raising_events', RaisingEvents, self.handle_raising_events)
        rospy.Subscriber('robot_is_saying', Bool, self.handle_robot_is_saying)

        self.pub_start_speech_recognition = rospy.Publisher('sp_speech_recognizer/start', Empty, queue_size=1)
        self.pub_stop_speech_recognition = rospy.Publisher('sp_speech_recognizer/stop', Empty, queue_size=1)

        self.pub_set_idle_motion = rospy.Publisher('set_enable_idle_motion', Bool, queue_size=10)
        self.pub_set_idle_motion.publish(True)


        rospy.loginfo('\033[92m[%s]\033[0m initialized...'%rospy.get_name())
        pass

    def handle_raising_events(self, msg):
        pass

    def handle_robot_is_saying(self, msg):
        if msg.data:    # Robot started saying
            self.pub_set_idle_motion.publish(False)
            self.pub_stop_speech_recognition.publish()
        else:           # Robot completed saying
            self.pub_set_idle_motion.publish(True)
            self.pub_start_speech_recognition.publish()


if __name__ == '__main__':
    rospy.init_node('turn_detector', anonymous=False)
    m = TurnDetectorNode()
    rospy.spin()