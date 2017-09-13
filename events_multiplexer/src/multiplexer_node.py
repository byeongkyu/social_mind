#!/usr/bin/python
#-*- encoding: utf8 -*-

import json
import rospy
from mind_msgs.msg import ForwardingEvent, RaisingEvents
from mind_msgs.srv import ReadData, ReadDataRequest
import Queue

class MultiplexerNode:
    def __init__(self):
        rospy.init_node('multiplexer_node', anonymous=False)

        rospy.loginfo('\033[93m[%s]\033[0m wait for bringup social_events_memory...'%rospy.get_name())
        rospy.wait_for_service('social_events_memory/read_data')
        self.rd_event_mem = rospy.ServiceProxy('social_events_memory/read_data', ReadData)
        rospy.Subscriber('forwarding_event', ForwardingEvent, self.handle_social_events)
        self.pub_rasing_event = rospy.Publisher('raising_events', RaisingEvents, queue_size=10)

        self.events_queue = Queue.Queue()
        self.data_queue = Queue.Queue()
        self.recognized_words_queue = Queue.Queue()

        event_period = rospy.get_param('~event_period', 0.5)
        rospy.Timer(rospy.Duration(event_period), self.handle_trigger_events)

        rospy.loginfo('\033[93m[%s]\033[0m initialized...'%rospy.get_name())
        rospy.spin()


    def handle_social_events(self, msg):
        if msg.event == 'speech_recognized':
            # req = ReadDataRequest()
            # req.perception_name = 'speech_recognition'
            # req.query = '{}'
            # req.data.append('recognized_word')
            #
            # result = self.rd_event_mem(req)
            # if result.result:
            #     result = json.loads(result.data)
            #     self.recognized_words_queue.put(result['recognized_word'])
            parse_data = json.loads(msg.data)
            self.recognized_words_queue.put(parse_data['recognized_word'])

        self.events_queue.put(msg.event)
        self.data_queue.put(msg.data)


    def handle_trigger_events(self, event):
        if self.events_queue.empty() and self.recognized_words_queue.empty():
            return

        event_data = RaisingEvents()
        event_data.header.stamp = rospy.Time.now()

        try:
            event_data.recognized_word = self.recognized_words_queue.get_nowait()
            self.recognized_words_queue.task_done()
        except Queue.Empty, e:
            pass

        while True:
            try:
                event_data.events.append(self.events_queue.get_nowait())
                event_data.data.append(self.data_queue.get_nowait())
                self.events_queue.task_done()
                self.data_queue.task_done()
            except Queue.Empty, e:
                break

        self.pub_rasing_event.publish(event_data)

if __name__ == '__main__':
    m = MultiplexerNode()
