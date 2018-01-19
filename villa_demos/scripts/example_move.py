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

start_x = base.pose[0]
start_y = base.pose[1]
start_theta = base.pose[2]
goal_list = [ (0.5, 0.5,-3.14/4.0), (0,0,0), (0.5, -0.5, 3.14/4.0), (0,0,0) ]



def relative_go_to(x,y):
   try:
        base.go(x,y, base.pose[2], relative=True)
   except:
        rospy.logerr('Fail to go')

def go_to(x,y,theta=0):
   try:
        base.go(start_x + x,start_y + y, start_theta + theta)
   except:
        rospy.logerr('Fail to go')

def rotate(rads):
    try:
        base.go(0, 0, rads, 10, relative=True)
    except:
        rospy.logerr('Fail go')


if __name__=='__main__':
    tts.say(u'Ok')
    rospy.sleep(5)
    print base.pose
    current_goal = goal_list[0]

    for iter in range(0, 2):
        ig = 0
        for goal in goal_list:
            #if ig == 1:
                #tts.say(u'going home')
            go_to(goal[0], goal[1], goal[2])
            ig += 1
