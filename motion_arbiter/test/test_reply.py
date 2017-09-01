#!/usr/bin/env python
#-*- encoding: utf8 -*-

import rospy
from mhri_msgs.msg import Reply


def main():
    rospy.init_node('test_reply_node', anonymous=False)
    pub_msg = rospy.Publisher('/reply', Reply, queue_size=10)

    while True:
        input_msg = raw_input('reply: ')        
        if 'quit' in input_msg:
            quit()

        msg = Reply()
        msg.header.stamp = rospy.Time.now()
        msg.reply = input_msg

        pub_msg.publish(msg)

if __name__ == '__main__':
    main()
