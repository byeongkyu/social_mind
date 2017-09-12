#!/usr/bin/env python
#-*- encoding: utf8 -*-

import os
import yaml
import json
import rospy
from importlib import import_module
from mind_msgs.msg import ForwardingEvent
from mind_msgs.srv import WriteData, WriteDataRequest, ReadData, RegisterData, RegisterDataRequest
from std_msgs.msg import Empty


class PerceptionBase(object):
    def __init__(self, node_name):
        rospy.init_node(node_name, anonymous=False)

        try:
            conf_file = rospy.get_param('~config_file')
        except KeyError as e:
            rospy.logerr('You should set the parameter for perception config file...')
            quit()

        with open(os.path.abspath(conf_file)) as f:
            self.conf_data = yaml.load(f.read())
            rospy.loginfo('loading perception config file... %d perception(s) exists...'%len(self.conf_data.keys()))
            for item in self.conf_data.keys():
                rospy.loginfo('\033[92m  - %s: %d event(s) and %d data(s).\033[0m'%(item, len(self.conf_data[item]['events']), len(self.conf_data[item]['data'])))

        self.dict_srv_wr = {}
        self.dict_srv_rd = {}

        for item in self.conf_data.keys():
            if self.conf_data[item].has_key('target_memory'):
                memory_name = self.conf_data[item]['target_memory']
                rospy.loginfo('\033[94m[%s]\033[0m wait for bringup %s...'%(rospy.get_name(), memory_name))

                rospy.wait_for_service('/%s/write_data'%memory_name)
                self.dict_srv_wr[memory_name] = rospy.ServiceProxy('/%s/write_data'%memory_name, WriteData)
                self.dict_srv_rd[memory_name] = rospy.ServiceProxy('/%s/read_data'%memory_name, ReadData)

                self.register_data_to_memory(memory_name, item, self.conf_data[item]['data'])


        try:
            for item in self.conf_data.keys():
                callback_conf = self.conf_data[item]['callback_config']
                for k, v in callback_conf.items():
                    topic_name = k
                    data = v[0].split('/')

                    msg_class = getattr(import_module(data[0]), data[1])
                    handle_function = getattr(self, v[1])
                    rospy.Subscriber(topic_name, msg_class, handle_function)

        except AttributeError:
            raise NotImplementedError('Handle function \033[95m %s \033[0m does not implement.'%v[1])

        self.is_enable_perception = True
        rospy.Subscriber('%s/start'%rospy.get_name(), Empty, self.handle_start_perception)
        rospy.Subscriber('%s/stop'%rospy.get_name(), Empty, self.handle_stop_perception)
        

        self.pub_event = rospy.Publisher('forwarding_event', ForwardingEvent, queue_size=10)
        rospy.loginfo('\033[94m[%s]\033[0m initialize perception_base done...'%rospy.get_name())


    def handle_start_perception(self, msg):
        self.is_enable_perception = True
        rospy.loginfo('%s is enabled...'%rospy.get_name())

    def handle_stop_perception(self, msg):
        self.is_enable_perception = False
        rospy.loginfo('%s is disabled...'%rospy.get_name())

    def raise_event(self, perception_item, event):
        if not perception_item in self.conf_data.keys():
            rospy.logwarn('<%s> perception is not member of perception configuration...'%perception_item)
            return
        if not event in self.conf_data[perception_item]['events']:
            rospy.logwarn('<%s> event is not member of event list of perception configuration...'%event)
            return

        if not self.is_enable_perception:
            return

        msg = ForwardingEvent()
        msg.header.stamp = rospy.Time.now()
        msg.event = event
        msg.by = perception_item

        self.pub_event.publish(msg)

    def register_data_to_memory(self, memory_name, perception_name, data):
        rospy.wait_for_service('/%s/register_data'%memory_name)
        srv_register = rospy.ServiceProxy('/%s/register_data'%memory_name, RegisterData)

        srv_req = RegisterDataRequest()
        srv_req.perception_name = perception_name
        srv_req.data = json.dumps(data)

        return srv_register(srv_req)

    def save_to_memory(self, perception_name, data={}):
        if data == {}:
            rospy.logwarn('Empty data inserted...')
            return

        for item in data.keys():
            if not item in self.conf_data[perception_name]['data'].keys():
                rospy.logwarn('[%s -- wrong data inserted [%s]...'%(perception_name, item))
                return

        srv_req = WriteDataRequest()
        srv_req.perception_name = perception_name
        srv_req.data = json.dumps(data)
        srv_req.by = rospy.get_name()

        target_memory = self.conf_data[perception_name]['target_memory']

        try:
            rospy.wait_for_service('/%s/write_data'%target_memory)
            self.dict_srv_wr[target_memory](srv_req)
        except rospy.ServiceException as e:
            pass

    def read_from_memory(self, target_memory, data):
        if data == {}:
            rospy.logwarn('Empty data requested...')
            return

        req = ReadDataRequest()
        req.perception_name = data['perception_name']
        req.query = data['query']
        for item in data['data']:
            req.data.append(item)

        resonse = self.dict_srv_rd[target_memory](req)
        if not response.result:
            return {}

        results = json.loads(response.data)
        return results
