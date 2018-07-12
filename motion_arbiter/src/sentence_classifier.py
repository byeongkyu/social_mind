#!/usr/bin/env python
#-*- encoding: utf8 -*-

from nltk.corpus import stopwords
from nltk import word_tokenize, pos_tag, classify, NaiveBayesClassifier
import nltk.data

import pickle
import re
import rospy
import rospkg
import os

from mind_msgs.msg import Reply
from mind_msgs.msg import ReplyAnalyzed, EntitiesIndex

STOPWORDS = set(stopwords.words('english'))

def dialogue_act_features(sentence):
    features = {}
    sentence_filtered = re.sub("[\'.,#!?:-]", '', sentence)

    for word in word_tokenize(sentence_filtered):
        if word not in STOPWORDS:
            features['contains({})'.format(word.lower())] = True
    return features


class SentenceClassifier:
    def __init__(self):
        self.sent_detector = nltk.data.load('tokenizers/punkt/english.pickle')
        try:
            with open(rospkg.RosPack().get_path('motion_arbiter') + '/config/classifier.pickle') as f:
                self.classifier = pickle.load(f)
        except IOError as e:
            rospy.logerr(e)
            exit(-1)
        rospy.loginfo('Loaded classifier succeed.')

        rospy.Subscriber('reply', Reply, self.handle_domain_reply)
        self.pub_reply_analyzed = rospy.Publisher('reply_analyzed', ReplyAnalyzed, queue_size=10)
        rospy.loginfo("\033[93m[%s]\033[0m initialized." % rospy.get_name())

    def handle_domain_reply(self, msg):
        sents = self.sent_detector.tokenize(msg.reply.strip())

        msg = ReplyAnalyzed()
        msg.header.stamp = rospy.Time.now()

        for sent in sents:
            feature = dialogue_act_features(sent)
            result = self.classifier.classify(feature)

            entity = EntitiesIndex()
            for i in pos_tag(word_tokenize(sent)):
                if(i[1] in ['RB', 'PRP', 'NN', 'PRP$']):
                    entity.entity.append(i[0])
                    entity.entity_index.append(sent.index(i[0]))

            msg.entities.append(entity)
            msg.sents.append(sent)
            msg.act_type.append(result + ',%d'%len(sent))

        self.pub_reply_analyzed.publish(msg)


if __name__ == '__main__':
    rospy.init_node('sentence_classifier', anonymous=False)
    m = SentenceClassifier()
    rospy.spin()

