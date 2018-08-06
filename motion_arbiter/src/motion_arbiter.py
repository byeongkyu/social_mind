#!/usr/bin/env python
# -*- encoding: utf8 -*-

import json
import operator
import Queue
import re
import os
import signal
from threading import Thread
from matplotlib import pyplot as plt
import matplotlib

import actionlib
import rospy
from std_msgs.msg import Bool, Empty, String

from mind_msgs.msg import (LogItem, RenderItemGoal, RenderSceneAction, RenderSceneGoal, Reply, ReplyAnalyzed)
from mind_msgs.srv import ReadData, ReadDataRequest

MAX_QUEUE_SIZE = 10
TIME_FOR_CHARACTER = 0.18
SIZE_FOR_CHARACTER = 3


class OverridingType:
    QUEUE = 0
    CROSS = 1
    OVERRIDE = 2


class SceneQueueData:
    def __init__(self):
        self.sm = {'render': '', 'offset': 0.0}
        self.say = {'render': '', 'offset': 0.0}
        self.gaze = {'render': '', 'offset': 0.0}
        self.pointing = {'render': '', 'offset': 0.0}
        self.sound = {'render': '', 'offset': 0.0}
        self.expression = {'render': '', 'offset': 0.0}
        self.screen = {'render': '', 'offset': 0.0}
        self.mobility = {'render': '', 'offset': 0.0}
        self.br = {'time': 0}
        self.log = ''

    def __str__(self):
        rospy.loginfo('scene item')
        print('=' * 30)
        print ' [SM]        : ', self.sm
        print ' [SAY]       : ', self.say
        print ' [GAZE]      : ', self.gaze
        print ' [POINTING]  : ', self.pointing
        print ' [SOUND]     : ', self.sound
        print ' [EXPRESSION]: ', self.expression
        print ' [SCREEN]    : ', self.screen
        print ' [MOBILITY]  : ', self.mobility
        print ' [BR]        : ', self.br
        print ' [LOG]       : ', self.log
        print('-' * 30)
        return ''

    def get_plot_data(self):
        offset = self.br['time']

        data = []
        data.append(['SAY',        self.say['render'],        (offset + self.say['offset'], float(len(self.say['render']) / SIZE_FOR_CHARACTER * TIME_FOR_CHARACTER)), (2, 3.6), '#a2a415'])
        data.append(['SM',         self.sm['render'],         (offset + self.sm['offset'], 4.0),         (6, 3.6), '#607c8e']) #TBD
        data.append(['GAZE',       self.gaze['render'],       (offset + self.gaze['offset'], 2.0),       (10, 3.6), '#fac205']) #TBD
        data.append(['POINTING',   self.pointing['render'],   (offset + self.pointing['offset'], 2.0),   (14, 3.6), '#c7fdb5']) #TBD
        data.append(['SOUND',      self.sound['render'],      (offset + self.sound['offset'], 4.0),      (18, 3.6), '#06c2ac']) #TBD
        data.append(['EXPRESSION', self.expression['render'], (offset + self.expression['offset'], 4.0), (22, 3.6), '#c875c4']) #TBD
        data.append(['SCREEN',     self.screen['render'],     (offset + self.screen['offset'], 4.0),     (26, 3.6), '#88b378']) #TBD
        data.append(['MOBILITY',   self.mobility['render'],   (offset + self.mobility['offset'], 4.0),   (30, 3.6), '#82cafc']) #TBD
        return data


class MotionArbiter:
    def __init__(self):
        self.enable_visualization = rospy.get_param('~visualization', False)

        aa = SceneQueueData()
        aa.get_plot_data()

        if self.enable_visualization:
            matplotlib.rcParams['toolbar'] = 'None'
            plt.ion()
            self.fig = plt.figure(figsize=(8, 2))
            self.fig.canvas.set_window_title('Generated Timeline')
            self.ax = plt.subplot(111)

            self.plot_item = SceneQueueData()
            self.plot_to_matplotlib(self.plot_item.get_plot_data())

            self.fig.canvas.draw()
            plt.pause(0.001)
            plt.show(block=False)

            signal.signal(signal.SIGUSR2, self.handle_update_figure)
            signal.siginterrupt(signal.SIGUSR2, False)

        self.is_rendering = False

        rospy.loginfo('\033[91m[%s]\033[0m waiting for bringup social_mind...'%rospy.get_name())
        self.rd_memory = {}
        try:
            rospy.wait_for_service('social_events_memory/read_data')
            self.rd_memory['social_events_memory'] = rospy.ServiceProxy('social_events_memory/read_data', ReadData)
            rospy.wait_for_service('environmental_memory/read_data')
            self.rd_memory['environmental_memory'] = rospy.ServiceProxy('environmental_memory/read_data', ReadData)
        except rospy.exceptions.ROSInterruptException as e:
            rospy.logerr(e)
            quit()

        self.renderer_client = actionlib.SimpleActionClient('render_scene', RenderSceneAction)
        rospy.loginfo('\033[91m[%s]\033[0m waiting for motion_renderer to start...'%rospy.get_name())

        try:
            self.renderer_client.wait_for_server()
        except rospy.exceptions.ROSInterruptException as e:
            quit()

        rospy.Subscriber('reply_analyzed', ReplyAnalyzed, self.handle_reply_analyzed)
        rospy.Subscriber('reply_deprecated', Reply, self.handle_domain_reply)
        self.pub_empty_queue = rospy.Publisher('scene_queue_empty', String, queue_size=10)
        self.pub_log_item = rospy.Publisher('log', LogItem, queue_size=10)

        self.pub_start_speech_recognizer = rospy.Publisher('sp_speech_recognizer/start', Empty, queue_size=1)
        self.pub_stop_speech_recognizer = rospy.Publisher('sp_speech_recognizer/stop', Empty, queue_size=1)

        self.pub_start_robot_speech = rospy.Publisher('robot_speech/start', Empty, queue_size=1)
        self.pub_stop_robot_speech = rospy.Publisher('robot_speech/stop', Empty, queue_size=1)

        self.is_speaking_started = False
        rospy.Subscriber('start_of_speech', Empty, self.handle_start_of_speech)
        rospy.Subscriber('end_of_speech', Empty, self.handle_end_of_speech)

        # rospy.Subscriber('emotion_status', EmotionStatus, self.handle_emotion_status, queue_size=10)
        # self.current_emotion = 'neutral'
        # self.current_emotion_intensity = 1.0

        self.pub_set_idle_motion = rospy.Publisher('idle_motion/set_enabled', Bool, queue_size=10)
        self.pub_set_idle_motion.publish(True)

        self.scene_queue = Queue.Queue(MAX_QUEUE_SIZE)
        self.scene_handle_thread = Thread(target=self.handle_scene_queue)
        self.scene_handle_thread.start()

        rospy.loginfo("\033[91m[%s]\033[0m initialized." % rospy.get_name())

    def plot_to_matplotlib(self, item):
        self.ax.cla()

        tick_labels = []
        ticks = []

        for i in item:
            tick_labels.append(i[0])
            ticks.append(i[3][0] + 2)

            if i[1] != '':
                self.ax.broken_barh([i[2]], i[3], facecolor=i[4])
            else:
                self.ax.broken_barh([(0, 0)], (0, 0))

        self.ax.set_yticklabels(tick_labels)
        self.ax.set_yticks(ticks)
        self.ax.set_ylim(0, 36)
        self.ax.invert_yaxis()
        self.fig.tight_layout()

    def save_plot_item(self, item):
        self.plot_item = item

    def handle_update_figure(self, signum, frame):
        self.plot_to_matplotlib(self.plot_item.get_plot_data())
        self.fig.canvas.draw()
        plt.pause(0.001)

    def handle_start_of_speech(self, msg):
        self.is_speaking_started = True

    def handle_end_of_speech(self, msg):
        self.is_speaking_started = False

    def handle_reply_analyzed(self, msg):
        reply_list = []

        # Save the replies
        for i in range(len(msg.sents)):
            reply_list.append( (msg.sents[i],
                msg.act_type[i].split('/')[1],
                msg.act_type[i].split('/')[0],
                msg.entities[i].entity,
                list(msg.entities[i].entity_index)
                ) )

        # Generate Timeline for each reply
        for reply in reply_list:
            print reply
            reply_tags = re.findall('({[^}]+})', reply[0])
            reply_text = re.sub('({[^}]+})', '', reply[0]).strip()

            scene_item = SceneQueueData()
            scene_item.br['time'] = 0

            if len(reply_tags) > 0:
                for tag in reply_tags[0].strip('{}').split('|'):
                    tag = tag.split('=')
                    tag_type = tag[0].strip().lower()
                    tag_content = tag[1].strip().lower()

                    if tag_type == 'expression':
                        scene_item.expression['render'] = tag_content
                        scene_item.expression['offset'] = 0.0 #TBD
                    elif tag_type == 'gaze':
                        scene_item.gaze['render'] = tag_content
                        scene_item.gaze['offset'] = 0.0 #TBD

            scene_item.sm['render'] = 'tag:' + reply[2].lower()
            scene_item.sm['offset'] = 0.0 #TBD

            if reply_text.strip() != '':
                scene_item.say['render'] = reply_text.strip()

                if reply[2] == 'GREETING':
                    scene_item.say['offset'] = 1.5 # TBD
                else:
                    scene_item.say['offset'] = 0.0

            self.scene_queue.put(scene_item)

    def handle_domain_reply(self, msg):
        rospy.logwarn('This topic is deprecated. Please use /reply.')
        recv_msg = msg.reply

        # 메시지 중에 <br=?> 태그가 있는 경우, 씬을 분리하고, 뒤쪽의 씬에 Delay 옵션을 줘야 한다.
        reply_list = []
        br_tags = re.findall('(<br=[^>]+>)', recv_msg)

        if len(br_tags) == 0:
            reply_list = [(recv_msg, 0)]
        else:
            split_target = recv_msg
            for i in range(len(br_tags)):
                if i == 0:
                    reply_list.append( (split_target.split(br_tags[i], 1)[0], 0) )
                else:
                    reply_list.append( (split_target.split(br_tags[i], 1)[0], int(br_tags[i-1].split('=')[-1].rstrip('>'))) )
                split_target = split_target.split(br_tags[i], 1)[1]
            reply_list.append( (split_target.split(br_tags[-1], 1)[0], int(br_tags[i-1].split('=')[-1].rstrip('>'))) )

        for recv_reply in reply_list:
            if recv_reply[0] == '':
                continue

            tags = re.findall('(<[^>]+>)', recv_reply[0])

            scene_item = SceneQueueData()
            scene_item.br['time'] = recv_reply[1]

            # scene_item.sm['render'] = 'tag:neutral'
            # scene_item.sm['offset'] = 0.0

            # scene_item.expression['render'] ='neutral'
            # scene_item.expression['offset'] = 0.0

            #scene_item.pointing = {}
            #scene_item.gaze = {}
            #scene_item.emotion = {}
            overriding = OverridingType.QUEUE

            tag_msg = recv_reply[0]
            for tag in tags:
                for other_tag in tags:
                    if other_tag != tag:
                        tag_msg.replace(other_tag, '')

                index = tag_msg.index(tag)
                tag_msg = tag_msg.replace(tag, '')

                tag = tag.lstrip('<')
                tag = tag.rstrip('>')
                tag = tag.split('=')

                if tag[0].strip() == 'sm':
                    scene_item.sm = {}
                    scene_item.sm['render'] = tag[1].strip()
                    scene_item.sm['offset'] = float(index / SIZE_FOR_CHARACTER * TIME_FOR_CHARACTER)
                elif tag[0].strip() == 'gaze':
                    scene_item.gaze['render'] = tag[1].strip()
                    scene_item.gaze['offset'] = float(index / SIZE_FOR_CHARACTER * TIME_FOR_CHARACTER)
                elif tag[0].strip() == 'pointing':
                    scene_item.pointing['render'] = tag[1].strip()
                    scene_item.pointing['offset'] = float(index / SIZE_FOR_CHARACTER * TIME_FOR_CHARACTER)
                elif tag[0].strip() == 'expression':
                    scene_item.expression['render'] = tag[1].strip()
                    scene_item.expression['offset'] = float(index / SIZE_FOR_CHARACTER * TIME_FOR_CHARACTER)
                elif tag[0].strip() == 'screen':
                    scene_item.screen['render'] = tag[1].strip()
                    scene_item.screen['offset'] = float(index / SIZE_FOR_CHARACTER * TIME_FOR_CHARACTER)
                elif tag[0].strip() == 'mobility':
                    scene_item.mobility['render'] = tag[1].strip()
                    scene_item.mobility['offset'] = float(index / SIZE_FOR_CHARACTER * TIME_FOR_CHARACTER)
                elif tag[0].strip() == 'sound':
                    scene_item.sound['render'] = tag[1].strip()
                    scene_item.sound['offset'] = float(index / SIZE_FOR_CHARACTER * TIME_FOR_CHARACTER)
                elif tag[0].strip() == 'overriding':
                    overriding = int(tag[1].strip())
                elif tag[0].strip() == 'log':
                    scene_item.log = tag[1].strip()

            if tag_msg.strip() != '':
                scene_item.say['render'] = tag_msg.strip()
                scene_item.say['offset'] = 0.0

            if tag_msg.strip() != '' and scene_item.sm == {}:
                scene_item.sm['render'] = 'tag:neutral'
                scene_item.sm['offset'] = 0.0

            if scene_item.pointing['render'] != '':
                scene_item.sm['render'] = ''
                scene_item.sm['offset'] = 0.0

            if overriding == OverridingType.QUEUE:
                self.scene_queue.put(scene_item)
            elif overriding == OverridingType.CROSS:
                backup_item = []

                while not self.scene_queue.empty():
                    backup_item.append(self.scene_queue.get())
                    self.scene_queue.task_done()

                self.scene_queue.put(scene_item)
                for item in backup_item:
                    self.scene_queue.put(item)

            elif overriding == OverridingType.OVERRIDE:
                self.renderer_client.cancel_all_goals()
                with self.scene_queue.mutex:
                    self.scene_queue.queue.clear()
                self.scene_queue.put(scene_item)

    def handle_scene_queue(self):
        rospy.wait_for_service('social_events_memory/read_data')
        rd_memory = rospy.ServiceProxy('social_events_memory/read_data', ReadData)

        while not rospy.is_shutdown():
            if not self.scene_queue.empty():
                goal = RenderSceneGoal()
                scene_dict = {}
                scene_item = self.scene_queue.get()
                self.scene_queue.task_done()

                if self.enable_visualization:
                    self.save_plot_item(scene_item)
                    os.kill(os.getpid(), signal.SIGUSR2)

                # Point and Semantic motion are exclusive relationship. If pointing exists, sm is ignored.
                if scene_item.pointing['render'] != '':
                    target_data = scene_item.pointing['render'].split(':')
                    try:
                        req = ReadDataRequest()
                        req.perception_name = target_data[0]
                        req.query = '{"name": "%s"}'%target_data[1]
                        req.data.append('name')
                        req.data.append('xyz')
                        req.data.append('frame_id')
                        response = self.rd_memory['environmental_memory'](req)

                        if response.result:
                            rospy.logdebug("read from social_mind for %s: %s"%(target_data[1], response.data))
                            # scene_item.pointing['render'] = 'pointing=' + response.data
                            scene_dict['sm'] = {}
                            scene_dict['sm']['render'] = 'pointing=' + response.data
                            scene_dict['sm']['offset'] = scene_item.pointing['offset']
                        else:
                            rospy.logwarn("Can't find the information if %s"%target_data[1])
                            scene_dict['sm'] = {'render': 'gesture=tag:neutral', 'offset': scene_item.pointing['offset']}

                    except rospy.ServiceException, e:
                        rospy.logerr("service call failed: %s" % e)
                    except ValueError:
                        scene_dict['sm'] = {'render': 'gesture=tag:neutral', 'offset': scene_item.pointing['offset']}
                else:
                    if scene_item.sm['render'] != '':
                        scene_dict['sm'] = {}
                        scene_dict['sm']['render'] = 'gesture=' + scene_item.sm['render']
                        scene_dict['sm']['offset'] = scene_item.sm['offset']

                if scene_item.gaze['render'] != '':
                    target_data = scene_item.gaze['render'].split(':')
                    try:
                        req = ReadDataRequest()
                        req.perception_name = target_data[0]
                        req.query = '{"name": "%s"}'%target_data[1]
                        req.data.append('name')
                        # req.data.append('xyz')
                        # req.data.append('frame_id')
                        response = self.rd_memory['environmental_memory'](req)

                        if response.result:
                            rospy.logdebug("read from environmental_memory for %s: %s"%(target_data[1], response.data))
                            scene_dict['gaze'] = {}
                            scene_dict['gaze']['render'] = response.data
                            scene_dict['gaze']['offset'] = scene_item.gaze['offset']
                        else:
                            rospy.logwarn("Can't find the information for %s"%target_data[1])

                    except rospy.ServiceException, e:
                        rospy.logerr("service call failed: %s" % e)

                '''
                sm = {}
                say = ''
                gaze = {}
                sound = {}
                expression = {}
                screen = {}
                mobility = {}
                br = {}
                log = ''
                '''

                if scene_item.log != '':
                    msg_log_item = LogItem()
                    log_item = scene_item.log.split('/')
                    for data in log_item:
                        msg_log_item.log_items.append(data)

                    msg_log_item.header.stamp = rospy.Time.now()
                    self.pub_log_item.publish(msg_log_item)

                scene_dict['say'] = scene_item.say
                scene_dict['sound'] = scene_item.sound
                scene_dict['expression'] = scene_item.expression
                scene_dict['screen'] = scene_item.screen
                scene_dict['mobility'] = scene_item.mobility
                scene_dict['gaze'] = scene_item.gaze
                scene_dict['br'] = scene_item.br

                # 감정은 소셜 메모리에서 읽어온다.
                scene_dict['emotion'] = {}
                scene_dict['emotion']['current_emotion'] = 'neutral'
                scene_dict['emotion']['intensity'] = 1.0

                self.pub_set_idle_motion.publish(False)

                goal.render_scene = json.dumps(scene_dict)
                self.renderer_client.send_goal(goal, done_cb=self.render_done, feedback_cb=self.render_feedback, active_cb=self.render_active)
                self.is_rendering = True
                rospy.sleep(0.1)

                while not rospy.is_shutdown() and self.is_rendering:
                    rospy.sleep(0.1)
            else:
                rospy.sleep(0.2)

            self.pub_set_idle_motion.publish(True)
            scene_item = {}

    def render_active(self):
        rospy.loginfo('\033[91m[%s]\033[0m scene rendering started...'%rospy.get_name())
        self.is_rendering = True
        self.pub_stop_speech_recognizer.publish()
        self.pub_start_robot_speech.publish()

    def render_feedback(self, feedback):
        rospy.loginfo('\033[91m[%s]\033[0m scene rendering feedback...'%rospy.get_name())

    def render_done(self, state, result):
        rospy.loginfo('\033[91m[%s]\033[0m scene rendering done...'%rospy.get_name())
        self.is_rendering = False

        if self.scene_queue.empty():
            self.pub_empty_queue.publish(json.dumps('{}'))

        rospy.sleep(0.2)
        self.pub_start_speech_recognizer.publish()
        self.pub_stop_robot_speech.publish()





if __name__ == '__main__':
    rospy.init_node('motion_arbiter', anonymous=False)
    m = MotionArbiter()
    rospy.spin()
