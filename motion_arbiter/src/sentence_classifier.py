#!/usr/bin/env python
#-*- encoding: utf8 -*-

import os
import pickle
import re
import warnings
warnings.filterwarnings('ignore', category=RuntimeWarning)

import nltk.data
import rospkg
import rospy
from nltk import NaiveBayesClassifier, classify, pos_tag, word_tokenize
from nltk.corpus import stopwords

from mind_msgs.msg import EntitiesIndex, Reply, ReplyAnalyzed


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
            # sperate tags and text
            sent_tags = re.findall('(%[^}]+%)', sent)
            sent_text = re.sub('(%[^}]+%)', '', sent).strip()

            # if task manager select intent we use it, or we use classifier for select intent
            result = ''
            remain_tags = ''
            if not any('sm=' in tag for tag in sent_tags):
                feature = dialogue_act_features(sent_text)
                result = self.classifier.classify(feature)

                if sent_tags != []:
                    remain_tags = sent_tags[0]
            else:
                tag_text = sent_tags[0].strip('{}').split('|')
                matching = [s for s in tag_text if "sm=" in s]
                if len(matching) > 1:
                    rospy.logwarn('Only one sm tags allowed...')
                result = matching[0].split('=')[1]
                for s in tag_text:
                    if not "sm=" in s:
                        remain_tags += s + '|'
                if remain_tags != '':
                    remain_tags = '{' + remain_tags.rstrip('|') + '}'

            # select entities
            entity = EntitiesIndex()
            for i in pos_tag(word_tokenize(sent_text)):
                if(i[1] in ['RB', 'PRP', 'NN', 'PRP$']):
                    entity.entity.append(i[0])
                    entity.entity_index.append(sent_text.index(i[0]))

            msg.entities.append(entity)
            msg.sents.append(remain_tags + ' ' + sent_text)
            msg.act_type.append(result + '/%d'%len(sent_text))

        self.pub_reply_analyzed.publish(msg)


if __name__ == '__main__':
    rospy.init_node('sentence_classifier', anonymous=False)
    m = SentenceClassifier()
    rospy.spin()
