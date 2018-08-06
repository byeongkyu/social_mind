#!/usr/bin/env python
#-*- encoding: utf8 -*-

import json
import random

import rospy
import actionlib

from std_msgs.msg import String
from mind_msgs.msg import RenderSceneAction, RenderSceneFeedback, RenderSceneResult
from mind_msgs.msg import RenderItemAction, RenderItemGoal
from mind_msgs.srv import GetInstalledGestures


class MotionRenderer:
    is_rendering = {}
    render_client = {}
    cb_start = {}
    cb_done = {}

    def __init__(self):
        rospy.init_node('motion_renderer', anonymous=False)

        self.server = actionlib.SimpleActionServer('render_scene', RenderSceneAction, self.execute_callback, False)
        self.server.register_preempt_callback(self.preempt_callback)
        self.server.start()

        rospy.loginfo('\033[94m[%s]\033[0m wait for render item...'%rospy.get_name())
        try:
            rospy.wait_for_service('get_installed_gestures')
        except rospy.exceptions.ROSException as e:
            rospy.logerr(e)
            quit()

        self.gazefocus_pub = rospy.Publisher('gaze_focusing', String, queue_size=10)

        self.render_client['say'] = actionlib.SimpleActionClient('render_speech', RenderItemAction)
        self.render_client['say'].wait_for_server()

        self.render_client['sm'] = actionlib.SimpleActionClient('render_gesture', RenderItemAction)
        self.render_client['sm'].wait_for_server()

        self.get_motion = rospy.ServiceProxy('get_installed_gestures', GetInstalledGestures)
        json_data = self.get_motion()
        self.motion_tag = json.loads(json_data.gestures)
        rospy.loginfo('\033[94m[%s]\033[0m success to get motion_tag from gesture server' % rospy.get_name())

        self.render_client['expression'] = actionlib.SimpleActionClient('render_facial_expression', RenderItemAction)
        self.render_client['expression'].wait_for_server()
        rospy.loginfo('\033[94m[%s]\033[0m ready to connect facial_expression'%rospy.get_name())

        self.render_client['screen'] = actionlib.SimpleActionClient('render_screen', RenderItemAction)
        self.render_client['screen'].wait_for_server()
        rospy.loginfo('\033[94m[%s]\033[0m ready to connect screen'%rospy.get_name())

        self.render_client['mobility'] = actionlib.SimpleActionClient('render_mobility', RenderItemAction)
        self.render_client['mobility'].wait_for_server()
        rospy.loginfo('\033[94m[%s]\033[0m ready to connect mobility'%rospy.get_name())

        self.render_client['sound'] = actionlib.SimpleActionClient('render_sound', RenderItemAction)
        self.render_client['sound'].wait_for_server()
        rospy.loginfo('\033[94m[%s]\033[0m ready to connect sound'%rospy.get_name())

        # Status flags
        self.is_rendering['say'] = False
        self.is_rendering['sm'] = False
        self.is_rendering['expression'] = False
        self.is_rendering['screen'] = False
        self.is_rendering['sound'] = False
        self.is_rendering['mobility'] = False

        self.return_to_last_expression = False

        # Register callback functions.
        self.cb_start['say'] = self.handle_render_say_start
        self.cb_done['say'] = self.handle_render_say_done

        self.cb_start['sm'] = self.handle_render_sm_start
        self.cb_done['sm'] = self.handle_render_sm_done

        self.cb_start['expression'] = self.handle_render_expression_start
        self.cb_done['expression'] = self.handle_render_expression_done

        self.cb_start['screen'] = self.handle_render_screen_start
        self.cb_done['screen'] = self.handle_render_screen_done

        self.cb_start['mobility'] = self.handle_render_mobility_start
        self.cb_done['mobility'] = self.handle_render_mobility_done

        self.cb_start['sound'] = self.handle_render_sound_start
        self.cb_done['sound'] = self.handle_render_sound_done

        rospy.loginfo("\033[94m[%s]\033[0m initialized." % rospy.get_name())
        rospy.spin()

    def handle_render_say_done(self, state, result):
        self.is_rendering['say'] = False

    def handle_render_say_start(self):
        self.is_rendering['say'] = True

    def handle_render_sm_done(self, state, result):
        self.is_rendering['sm'] = False

    def handle_render_sm_start(self):
        self.is_rendering['sm'] = True

    def handle_render_expression_done(self, state, result):
        self.is_rendering['expression'] = False

    def handle_render_expression_start(self):
        self.is_rendering['expression'] = True

    def handle_render_screen_done(self, state, result):
        self.is_rendering['screen'] = False

    def handle_render_screen_start(self):
        self.is_rendering['screen'] = True

    def handle_render_mobility_done(self, state, result):
        self.is_rendering['mobility'] = False

    def handle_render_mobility_start(self):
        self.is_rendering['mobility'] = True

    def handle_render_sound_done(self, state, result):
        self.is_rendering['sound'] = False

    def handle_render_sound_start(self):
        self.is_rendering['sound'] = True

    def preempt_callback(self):
        rospy.logwarn('\033[94m[%s]\033[0m rendering preempted.' % rospy.get_name())
        for k in self.is_rendering.keys():
            if self.is_rendering[k]:
                self.render_client[k].cancel_all_goals()

    def execute_callback(self, goal):
        rospy.loginfo('\033[94m[%s]\033[0m rendering started.' % rospy.get_name())
        result = RenderSceneResult()

        render_scene = json.loads(goal.render_scene)
        render_scene_time = {}
        for k, v in render_scene.items():
            if k != 'emotion' and k != 'br' and v['render'] != '':
                render_scene_time[k] = v['offset']

        try:
            if render_scene['expression'] != {}:
                # render_scene['expression']['render'] = render_scene['expression']['render'].rstrip('~')
                self.return_to_last_expression = False
        except KeyError:
            pass

        delay_time = render_scene['br']['time']

        for i in range(delay_time * 10):
            rospy.sleep(0.1)

        # Sort by delay time
        scene_item_sorted_by_time = sorted(render_scene_time, key=render_scene_time.get)
        first_offset_time = render_scene[scene_item_sorted_by_time[0]]['offset']
        rospy.sleep(first_offset_time)

        for i in range(0, len(scene_item_sorted_by_time) - 1):
            if scene_item_sorted_by_time[i] == 'gaze':
                focusing_name = render_scene[scene_item_sorted_by_time[i]]['render']
                self.gazefocus_pub.publish(focusing_name)
            else:
                item_goal = RenderItemGoal()
                item_goal.name = scene_item_sorted_by_time[i]
                item_goal.data = render_scene[scene_item_sorted_by_time[i]]['render']

                # if item_goal.data == '':
                #     continue

                self.render_client[scene_item_sorted_by_time[i]].send_goal(
                    goal=item_goal,
                    done_cb=self.cb_done[scene_item_sorted_by_time[i]],
                    active_cb=self.cb_start[scene_item_sorted_by_time[i]])

                while not rospy.is_shutdown() and not self.is_rendering[scene_item_sorted_by_time[i]]:
                    rospy.sleep(0.01)
                    pass

            delta_time = render_scene[scene_item_sorted_by_time[i+1]]['offset'] - render_scene[scene_item_sorted_by_time[i]]['offset']
            rospy.sleep(delta_time)

        if scene_item_sorted_by_time[-1] == 'gaze':
            focusing_name = render_scene[scene_item_sorted_by_time[-1]]['render']
            self.gazefocus_pub.publish(focusing_name)
        else:
            item_goal = RenderItemGoal()
            item_goal.name = scene_item_sorted_by_time[-1]
            item_goal.data = render_scene[scene_item_sorted_by_time[-1]]['render']

            self.render_client[scene_item_sorted_by_time[-1]].send_goal(
                goal=item_goal,
                done_cb=self.cb_done[scene_item_sorted_by_time[-1]],
                active_cb=self.cb_start[scene_item_sorted_by_time[-1]])

            while not rospy.is_shutdown() and not self.is_rendering[scene_item_sorted_by_time[-1]]:
                rospy.sleep(0.01)
                pass

        while not rospy.is_shutdown():
            rendering = False
            for i in scene_item_sorted_by_time:
                if i != 'gaze':
                    rendering = rendering or self.is_rendering[i]
            if not rendering:
                break
            rospy.sleep(0.1)
        self.gazefocus_pub.publish('')

        if self.return_to_last_expression:
            item_goal = RenderItemGoal()
            item_goal.name = 'expression'
            item_goal.data = render_scene['emotion']['current_emotion']
            self.render_client['expression'].send_goal(
                goal=item_goal,
                done_cb=self.cb_done['expression'],
                active_cb=self.cb_start['expression'])

            while not rospy.is_shutdown() and not self.is_rendering['expression']:
                rospy.sleep(0.01)
            while not rospy.is_shutdown() and self.is_rendering['expression']:
                rospy.sleep(0.01)
            self.return_to_last_expression = False

        '''
        if goal.emotion == 'neutral':
        self.pub_face_emotion.publish(
            SetFacialExpression.NEUTRAL, goal.emotion_intensity)
        elif goal.emotion == 'happiness':
        self.pub_face_emotion.publish(
            SetFacialExpression.HAPPINESS, goal.emotion_intensity)
        elif goal.emotion == 'surprise':
        self.pub_face_emotion.publish(
            SetFacialExpression.HAPPINESS, goal.emotion_intensity)
        elif goal.emotion == 'anger':
        self.pub_face_emotion.publish(
            SetFacialExpression.HAPPINESS, goal.emotion_intensity)
        elif goal.emotion == 'sadness':
        self.pub_face_emotion.publish(
            SetFacialExpression.HAPPINESS, goal.emotion_intensity)
        elif goal.emotion == 'disgust':
        self.pub_face_emotion.publish(
            SetFacialExpression.HAPPINESS, goal.emotion_intensity)
        elif goal.emotion == 'fear':
        self.pub_face_emotion.publish(
            SetFacialExpression.HAPPINESS, goal.emotion_intensity)

        if goal.gesture != '':
        # When robot requested play gesture, the idle motion is disabled temporary
        self.is_playing_now = True

        # print goal.gesture
        recv_data = goal.gesture.split(':')
        if recv_data[0] == 'sm':
        # print recv_data
        if recv_data[1] in self.motion_tag:
        gesture_name = self.motion_tag[recv_data[1]][
            random.randrange(0, len(self.motion_tag[recv_data[1]]))]
        else:
        gesture_name = recv_data[1]
        elif recv_data[0] == 'pm':
        gesture_name = recv_data[1]

        gesture_goal = GestureActionGoal(gesture=gesture_name)
        self.gesture_client.send_goal(goal=gesture_goal, done_cb=self.gesture_done_cb,
                                      feedback_cb=self.gesture_playing_cb, active_cb=self.gesture_start_cb)

        # rospy.sleep(2)

        if goal.say != '':
        # When robot is speaking, the speech_recognition is disabled temporary
        self.is_speaking_now = True
        self.is_gesture_only = False
        speech_goal = SpeechActionGoal(say=goal.say)
        self.speech_client.send_goal(goal=speech_goal, done_cb=self.speech_done_cb,
                                     feedback_cb=self.speech_speaking_cb, active_cb=self.speech_start_cb)
        else:
        # Gesture Only
        self.is_gesture_only = True

        while self.is_speaking_now or self.is_playing_now:
        # rospy.logwarn('%d %d'%(self.is_speaking_now, self.is_playing_now))

        if self.is_gesture_only:
        rospy.sleep(0.2)
        continue

        if not self.is_speaking_now and self.is_playing_now:
        self.sync_count_gesture += 1
        if self.sync_count_gesture > 3:
        self.gesture_client.cancel_all_goals()
        self.sync_count_gesture = 0

        rospy.sleep(0.2)

        self.pub_face_emotion.publish(SetFacialExpression.PREVIOUS_FACE, 1.0)
        '''
        rospy.loginfo('\033[94m[%s]\033[0m rendering completed.' % rospy.get_name())

        result.result = True
        self.server.set_succeeded(result)


if __name__ == '__main__':
    m = MotionRenderer()
