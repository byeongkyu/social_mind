#!/usr/bin/python
#-*- encoding: utf8 -*-

import random
import json
import threading
import rospy

from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from mind_msgs.msg import RaisingEvents, GazeCommand
from mind_msgs.srv import ReadData, ReadDataRequest


GAZE_CONTROLLER_PERIOD = 0.2
GLANCE_TIMEOUT_MEAN = 6.0
IDLE_TIMEOUT_MEAN = 10.0

class GazeState:
    IDLE = 0
    FOCUSING = 1
    TRACKING = 2
    GLANCE = 3
    UNKNOWN = -1

class GazeNode:
    def __init__(self):
        rospy.init_node('gaze', anonymous=False)

        self.lock = threading.RLock()
        with self.lock:
            self.current_state = GazeState.IDLE
            self.last_state = self.current_state

        # Initialize Variables
        self.glance_timeout = 0
        self.glance_timecount = 0
        self.glance_played = False

        self.idle_timeout = 0
        self.idle_timecount = 0
        self.idle_played = False

        self.default_height = rospy.get_param('~default_height', 0.4)

        rospy.loginfo('\033[92m[%s]\033[0m waiting for bringup social_mind...'%rospy.get_name())
        rospy.wait_for_service('environmental_memory/read_data')
        rospy.wait_for_service('social_events_memory/read_data')

        self.rd_memory = {}
        self.rd_memory['environmental_memory'] = rospy.ServiceProxy('environmental_memory/read_data', ReadData)
        self.rd_memory['social_events_memory'] = rospy.ServiceProxy('social_events_memory/read_data', ReadData)

        rospy.Subscriber('raising_events', RaisingEvents, self.handle_raising_events)
        rospy.Subscriber('gaze_focusing', String, self.handle_gaze_focusing)
        self.pub_gaze_cmd = rospy.Publisher('gaze_command', GazeCommand, queue_size=10)
        self.pub_viz_gaze_cmd = rospy.Publisher('visualization_gaze_cmd', PointStamped, queue_size=10)

        rospy.Timer(rospy.Duration(GAZE_CONTROLLER_PERIOD), self.handle_gaze_controller)
        rospy.loginfo('\033[92m[%s]\033[0m initialized...'%rospy.get_name())
        rospy.spin()

    def handle_raising_events(self, msg):
        if len(msg.events) == 0:
            return

        if 'loud_sound_detected' in msg.events:
            with self.lock:
                if self.current_state != GazeState.GLANCE:
                    self.last_state = self.current_state
                    self.current_state = GazeState.GLANCE
        elif 'person_appeared' in msg.events or 'face_detected' in msg.events:
            with self.lock:
                if self.current_state != GazeState.TRACKING:
                    self.last_state = self.current_state
                    self.current_state = GazeState.TRACKING

    def handle_gaze_focusing(self, msg):
        # 환경 메모리에서 전달된 이름에 대한 정보가 있는지 확인해보고, 있으면 타겟설정, 없으면 현재모드 유지
        if msg.data != '':
            with self.lock:
                if self.current_state != GazeState.FOCUSING:
                    self.last_state = self.current_state
                    self.current_state = GazeState.FOCUSING
                self.focusing_target = msg.data
        elif self.current_state == GazeState.FOCUSING and msg.data == '':
            with self.lock:
                self.current_state = self.last_state
                self.focusing_target = ''

    def handle_gaze_controller(self, event):
        # 0.2ms (조정가능) 주기로 동작되는 컨트롤러 모드에 따라 동작을 달리한다.
        if self.current_state == GazeState.IDLE:
            # 4 ~ 6초 간격으로 랜덤 타켓 포지션
            if not self.idle_played:
                self.idle_timecount = 0
                self.idle_timeout = random.randrange(
                    IDLE_TIMEOUT_MEAN/GAZE_CONTROLLER_PERIOD, (IDLE_TIMEOUT_MEAN+2.0)/GAZE_CONTROLLER_PERIOD)
                self.idle_played = True
            else:
                self.idle_timecount += 1
                if self.idle_timecount > self.idle_timeout:
                    cmd = GazeCommand()
                    # cmd.target_point.header.stamp = rospy.Time.now()
                    cmd.target_point.header.frame_id = 'base_link'
                    cmd.target_point.point.x = 2.0
                    cmd.target_point.point.y = random.randrange(-10, 10) / 10.0
                    cmd.target_point.point.z = self.default_height + (random.randrange(-2, 5) / 10.0)
                    cmd.max_speed = random.randrange(5, 15) / 100.0

                    self.pub_gaze_cmd.publish(cmd)
                    self.pub_viz_gaze_cmd.publish(cmd.target_point)

                    self.idle_timecount = 0
                    self.idle_timeout = random.randrange(
                        IDLE_TIMEOUT_MEAN/GAZE_CONTROLLER_PERIOD, (IDLE_TIMEOUT_MEAN+2.0)/GAZE_CONTROLLER_PERIOD)

        elif self.current_state == GazeState.GLANCE:
            if not self.glance_played:
                req = ReadDataRequest()
                req.perception_name = 'loud_sound_detection'
                req.query = '{}'
                req.data.append('xyz')
                req.data.append('frame_id')
                response = self.rd_memory['social_events_memory'](req)

                if not response.result:
                    self.current_state = self.last_state
                    return

                result_data = json.loads(response.data)

                cmd = GazeCommand()
                cmd.target_point.header.frame_id = 'base_footprint'#result_data['frame_id']
                cmd.target_point.point.x = 1.0
                cmd.target_point.point.z = 0.6 + (random.randrange(0, 30) / 100.0)

                if result_data['xyz'][1] < -0.2:   #Right Side
                    cmd.target_point.point.y = -1.0 * random.randrange(10, 20) / 10.0
                else:
                    cmd.target_point.point.y = random.randrange(10, 20) / 10.0
                cmd.max_speed = 1.0

                self.pub_gaze_cmd.publish(cmd)
                self.pub_viz_gaze_cmd.publish(cmd.target_point)

                rospy.loginfo('\033[92m[%s]\033[0m changed the state - [GLANCE]...'%rospy.get_name())

                self.glance_timecount = 0
                self.glance_timeout = random.randrange(
                    GLANCE_TIMEOUT_MEAN/GAZE_CONTROLLER_PERIOD, (GLANCE_TIMEOUT_MEAN+1.0)/GAZE_CONTROLLER_PERIOD)
                self.glance_played = True
            else:
                self.glance_timecount += 1
                if self.glance_timecount > self.glance_timeout:
                    self.glance_played = False
                    self.glance_timecount = 0

                    self.lock.acquire()
                    self.current_state = self.last_state
                    self.lock.release()
                    rospy.loginfo('\033[92m[%s]\033[0m return from GLANCE to last state...'%rospy.get_name())

        elif self.current_state == GazeState.FOCUSING:
            target_type = ''
            target_name = ''

            try:
                target_type, target_name = self.focusing_target.split(':')
            except ValueError:
                with self.lock:
                    self.current_state = self.last_state
                return

            req = ReadDataRequest()
            req.perception_name = target_type
            req.query = '{"name": "%s"}'%target_name
            req.data.append('xyz')
            req.data.append('frame_id')
            response = self.rd_memory['environmental_memory'](req)

            if response.result:
                rospy.logdebug("read from environmental_memory for %s: %s"%(target_name, response.data))
                result_data = json.loads(response.data)

                cmd = GazeCommand()
                cmd.target_point.header.frame_id = result_data['frame_id']
                cmd.target_point.point.x = result_data['xyz'][0]
                cmd.target_point.point.y = result_data['xyz'][1]
                cmd.target_point.point.z = result_data['xyz'][2]
                cmd.max_speed = 0.2

                self.pub_gaze_cmd.publish(cmd)
                self.pub_viz_gaze_cmd.publish(cmd.target_point)
            else:
                rospy.logwarn('Can not find the information of %s in memory...'%target_name)
                with self.lock:
                    self.current_state = self.last_state


        elif self.current_state == GazeState.TRACKING:
            # 환경 메모리에서 사람들에 대한 정보를 받아옴
            # 1명일때, 2명 이상일때 플래닝 필요
            req = ReadDataRequest()
            req.perception_name = 'face_detection'
            req.query = '{}'
            req.data.append('count')
            response = self.rd_memory['social_events_memory'](req)

            result_data = json.loads(response.data)
            try:
                if result_data['count'] == 0:
                    with self.lock:
                        self.current_state = self.last_state
                    return
                else:
                    req = ReadDataRequest()
                    req.perception_name = 'persons'
                    req.query = '{}'
                    req.data = ['~']
                    response = self.rd_memory['environmental_memory'](req)

                    ret_data = json.loads(response.data)

                    try:
                        cmd = GazeCommand()
                        cmd.target_point.header.frame_id = ret_data[0]['frame_id']
                        cmd.target_point.point.x = ret_data[0]['xyz'][0]
                        cmd.target_point.point.y = ret_data[0]['xyz'][1]
                        cmd.target_point.point.z = ret_data[0]['xyz'][2]
                        cmd.max_speed = 0.2

                        self.pub_gaze_cmd.publish(cmd)
                        self.pub_viz_gaze_cmd.publish(cmd.target_point)
                    except KeyError:
                        pass
            except KeyError:
                pass

if __name__ == '__main__':
    m = GazeNode()
