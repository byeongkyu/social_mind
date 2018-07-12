#!/usr/bin/env python
#-*- encoding: utf8 -*-

from nltk.corpus import stopwords
from nltk import word_tokenize, classify, NaiveBayesClassifier
import pickle
import re
import rospy
import rospkg
import os
from datetime import datetime

STOPWORDS = set(stopwords.words('english'))

def dialogue_act_features(sentence):
    features = {}
    sentence_filtered = re.sub("[\'.,#!?:-]", '', sentence)

    for word in word_tokenize(sentence_filtered):
        if word not in STOPWORDS:
            features['contains({})'.format(word.lower())] = True
    return features


def main():
    rospy.init_node('train_classifier', anonymous=False)

    try:
        utterance_file = rospy.get_param('~utterance_data')
    except KeyError:
        rospy.logerr('set param utterance_data...')
        exit()

    utterance_file = os.path.expanduser(utterance_file)
    utterance_file = os.path.abspath(utterance_file)

    try:
        featuresets = []
        with open(utterance_file) as f:
            for line in f:
                line_data = line.split('/')
                featuresets.append((dialogue_act_features(line_data[1]), line_data[0]))

        size = int(len(featuresets) * 0.1)
        train_set, test_set = featuresets[size:], featuresets[:size]

        classifier = NaiveBayesClassifier.train(train_set)
        classifier.show_most_informative_features(10)
        # rospy.loginfo(classify.accuracy(classifier, test_set))

        save_path = rospkg.RosPack().get_path('motion_arbiter') + '/config'
        # check existance data
        if os.path.isfile(save_path + '/classifier.pickle'):
            os.rename(save_path + '/classifier.pickle', save_path + '/classifier.pickle.backup_'
                + datetime.now().strftime('%Y-%m-%d_%H_%M_%S'))

        with open(save_path + '/classifier.pickle', 'wb') as f:
            pickle.dump(classifier, f)

        rospy.loginfo('Done. training for classfier. Accuracy is %f'%classify.accuracy(classifier, test_set))

    except IOError as e:
        rospy.logerr(e)
        exit()

if __name__ == '__main__':
    main()