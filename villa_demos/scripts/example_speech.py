#!/usr/bin/python
# -*- coding: utf-8 -*-


from hsrb_interface import Robot
import rospy
import math

robot = Robot()
base = robot.try_get('omni_base')
tts = robot.try_get('default_tts')
whole_body = robot.try_get('whole_body')
tts.language = tts.ENGLISH

if __name__=='__main__':
    tts.say(u'We will do 22 pushups!')
    rospy.sleep(1)
    tts.say(u'Are you ready?')
    rospy.sleep(2)
    tts.say(u'Here we go!')
    rospy.sleep(1)
    for i in range(0,22):
        current_count = i+1
        tts.say(str(i+1))
        if (current_count == 11):
            rospy.sleep(0.2)
            tts.say("You're half way there!")
        rospy.sleep(1.5)
    tts.say(u'GOOD JOB!')