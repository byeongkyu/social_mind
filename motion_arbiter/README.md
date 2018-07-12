# Motion Arbiter

## Train sentence classifier

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