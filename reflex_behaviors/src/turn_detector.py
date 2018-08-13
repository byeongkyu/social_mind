#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy
from std_msgs.msg import Bool, String, Empty
from mind_msgs.msg import RaisingEvents, SetIdleMotion

class TurnDetectorNode:
    def __init__(self):
        rospy.Subscriber('raising_events', RaisingEvents, self.handle_raising_events)
        rospy.Subscriber('robot_is_saying', Bool, self.handle_robot_is_saying)

        self.pub_start_speech_recognition = rospy.Publisher('sp_speech_recognizer/start', Empty, queue_size=1)
        self.pub_stop_speech_recognition = rospy.Publisher('sp_speech_recognizer/stop', Empty, queue_size=1)

        self.pub_set_idle_motion = rospy.Publisher('set_enable_idle_motion', SetIdleMotion, queue_size=10)

        msg = SetIdleMotion()
        msg.enabled = True
        msg.with_leaning_forward = False
        self.pub_set_idle_motion.publish(msg)

        rospy.loginfo('\033[92m[%s]\033[0m initialized...'%rospy.get_name())

    def handle_raising_events(self, msg):
        pass

    def handle_robot_is_saying(self, msg):
        if msg.data:    
            # Robot started saying
            rospy.loginfo('\033[92m[%s]\033[0m Robot\'s Turn...')
            
            msg = SetIdleMotion()
            msg.enabled = False
            msg.with_leaning_forward = False

            self.pub_set_idle_motion.publish(msg)
            self.pub_stop_speech_recognition.publish()
        else:           
            # Robot completed saying
            rospy.loginfo('\033[92m[%s]\033[0m User\'s Turn...')

            msg = SetIdleMotion()
            msg.enabled = True
            msg.with_leaning_forward = True

            self.pub_set_idle_motion.publish(msg)
            self.pub_start_speech_recognition.publish()


if __name__ == '__main__':
    rospy.init_node('turn_detector', anonymous=False)
    m = TurnDetectorNode()
    rospy.spin()