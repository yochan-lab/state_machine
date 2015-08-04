#!/usr/bin/env python

__author__ = 'daniel'

import roslib
roslib.load_manifest('state_machine')
import rospy
import actionlib
import random

from move_base_msgs.msg import *
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import state_machine.msg


class RoamServer(object):
    _feedback = state_machine.msg.roamFeedback()
    _result = state_machine.msg.roamResult()

    def __init__(self, name):
        self._action_name = name
        self._server = actionlib.SimpleActionServer("roam", state_machine.msg.roamAction, execute_cb=self.roam, auto_start = False)
        self._server.start()


        #self._client = SimpleActionClient('move_base', MoveBaseAction)
        #self._client.wait_for_server(rospy.Duration(secs=10))

        self._locations = [
        PoseStamped(pose=Pose(Point(2.237, 2.296, 0.000), Quaternion(0.000, 0.000, -0.038, 0.999))), #tony
        PoseStamped(pose=Pose(Point(-5.286, 5.447, 0.000), Quaternion(0.000, 0.000, 0.992, -0.127))), #entrance
        PoseStamped(pose=Pose(Point(3.326, 38.098, 0.000), Quaternion(0.000, 0.000, 0.723, 0.691)))] #conference
        print "Roam Server started"

    def roam(self, goal):
        self._feedback.there_yet = False
        r = rospy.Rate(1)
        target = MoveBaseActionGoal().goal
        target.target_pose = self._locations[random.randint(0,2)] #pose stamped
        target.target_pose.header.frame_id = "map"
	self._client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	self._client.wait_for_server(rospy.Duration(secs=10))
        self._client.send_goal(target)

#        i = 0
#        print "starting count"

#        while i < 10:
#            self._feedback.there_yet = False
#            if self._server.is_preempt_requested():
#                rospy.loginfo('%s: Preempted' % self._action_name)
#                self._server.set_preempted()
#                return
#            i += 1
#            print i
#            if i == 50:
#                self._feedback.there_yet = True
#            self._server.publish_feedback(self._feedback)
#            r.sleep()

        while self._client.get_state() != actionlib.GoalStatus.SUCCEEDED: #check movebase success.
            #check for premption
            self._feedback.there_yet = False
            if self._server.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._client.cancel_all_goals()
		self._server.set_preempted()
                return
            # elif self._client.get_state() == GoalStatus.SUCCEEDED:
            #     self._feedback.there_yet = True
            # self._server.publish_feedback(self._feedback)
            r.sleep()

	self._feedback.there_yet = True
	self._server.publish_feedback(self._feedback)
        self._result.made_it = True
        self._server.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('roam_server')
    RoamServer(rospy.get_name())
    rospy.spin()
