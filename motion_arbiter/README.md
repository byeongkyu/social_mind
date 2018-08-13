# Motion Arbiter

## Training sentence classifier

- download nltk data
        $ rosrun motion_arbiter download_nltk_data.py 

- make utterance data with label

        $ cat utterance_data.txt

        GREETING/hi , i want to make a restaurant reservation .
        REQUEST/for what date ?
        THANK_YOU/thanks !
        AFFIRM/that ' s right .
        CANT_UNDERSTAND/what ?
        NEGATE/no .
        GOOD_BYE/bye .
        SELECT/would you like 7 pm or 8 pm ?
        NOTIFY_SUCCESS/your reservation is confirmed for 6 people at il fornaio next monday at 8 pm .
        INFORM/2 people and anytime is good
        OFFER/is 6.15 pm ok?
        NOTIFY_FAILURE/i was unable to get a reservation for 2 people at 7 : 15 pm for los altos grill . sorry !
        CANT_UNDERSTAND/i don ' t undersand
        OTHER/san francisco weather
        ...

- train classifier usage,

        $ rosrun motion_arbiter train_classifier.py _utterance_data:=<path>/utterance_data.txt

    and then, the classifier will saved to motion_arbiter/config/classifier.pickle, previous classifier name will be changed classifier_backup\_\<date\>.pickle

## Senetence Classifier

This node classify the input sentence to 14 types as show below:
    GREETING, REQUEST, REQUEST_ALT, THANK_YOU, AFFIRM, CANT_UNDERSTAND, NEGATE, GOOD_BYE, SELECT, NOTIFY_SUCCESS, INFORM, OFFER, NOTIFY_FAILURE, OTHER   

### usage

        $ rosrun motion_arbiter sentence_classifier.py

#### Subscribe

- reply (mind_msgs/Reply)

        Header header
        string reply

#### Publish

- reply_analyzed (mind_msgs/ReplyAnalyzed)

        Header header
        string[] sents
        string[] act_type
        EntitiesIndex[] entities

    example)

        ---
        header:
        seq: 3
        stamp:
            secs: 1531373284
            nsecs: 219924926
        frame_id: ''
        sents: [hi!, my name is brown.]
        act_type: ['GREETING,3', 'REQUEST,17']
        entities:
        -
            entity: [hi]
            entity_index: [0]
        -
            entity: [my, name]
            entity_index: [0, 3]
        ---
