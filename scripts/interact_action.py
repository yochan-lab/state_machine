#!/usr/bin/env python

__author__ = 'daniel'

import random, time

import rospy
import roslib
roslib.load_manifest('state_machine')
from actionlib import *

import state_machine.msg

class InteractServer(object):
    def __init__(self, name):
        self._action_name = name
        self._feedback = state_machine.msg.interactFeedback()
        self._result = state_machine.msg.interactResult()
        self._server = SimpleActionServer("interact",
                                          state_machine.msg.interactAction,
                                          execute_cb=self.execute_cb,
                                          auto_start = False)
        self._server.start()

    def execute_cb(self, goal):
        print "interacting"
        self._feedback.isInteracting = True

        if self._server.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._server.set_preempted()
            return
        self._server.publish_feedback(self._feedback)
        time.sleep(5)

        print "done interacting"
        self._result.action = random.randint(0,2)
        self._server.set_succeeded(self._result)



if __name__ == '__main__':
    rospy.init_node('interact_server')
    InteractServer(rospy.get_name())
    print "Interact Server started"
    rospy.spin()