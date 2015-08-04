#!/usr/bin/env python

__author__ = 'daniel'

import traceback


import rospy
import roslib
roslib.load_manifest('state_machine')
import actionlib

from std_msgs.msg import String, Int32
from geometry_msgs.msg import Vector3
import state_machine.msg

import rosplan_interface as interface

class StateTracker(object):
    class State(object):
        def __init__(self, ID):
            self._ID = ID
            self._active = False

        def enter(self):
            raise NotImplementedError

        def leave(self):
            raise NotImplementedError

        def get_id(self):
            return self._ID

        def get_active(self):
            return self._active

        def set_active(self, value):
            if self._active != value: #only execute if different
                self._active = value
                self.enter() if value else self.leave()

        active = property(get_active, set_active)

    class Idle(State):
        def enter(self):
            print "entering idle"
            self.idle_client = actionlib.SimpleActionClient("roam", state_machine.msg.roamAction)
            self.idle_client.wait_for_server(rospy.Duration(5))
            # while self.active:
            goal = state_machine.msg.roamGoal(waypoint=20)
            self.idle_client.send_goal(goal)
            self.idle_client.wait_for_result()
            #     while self.active and self.client.get_state() != actionlib.GoalStatus.SUCCEEDED:
            #         # print self.client.get_state()
            #         rospy.sleep(1.0)
            self.active = False

        def leave(self):
            print "leaving idle"
            try:
                if self.idle_client.get_state() != actionlib.GoalStatus.SUCCEEDED:
                    self.idle_client.cancel_goal()
            except AttributeError:
                pass

    class Execute(State):
        def enter(self):
            print "entering execute"
            while self.active and len(tasks) > 0:
                interface.clear_goals()
                for goal in tasks[0]:
                    interface.add_goal(goal)
                interface.plan()
                print "executing task"
                rospy.sleep(3)
		# print interface.get_dispatch_status()
                while not interface.is_done(): # rosplan not done yet
                    # print interface.get_dispatch_status()
		    if not self.active:
                        return
                    rospy.sleep(.1)
                print "completed task"
		tasks.pop(0)
            self.active = False

        def leave(self):
            print "leaving execute"
            interface.cancel()
            interface.clear_goals()

    class Interact(State):
        def enter(self):
            print "now interact"
            self.client = actionlib.SimpleActionClient("interact", state_machine.msg.interactAction)
            self.client.wait_for_server()
            self.client.cancel_all_goals()
            goal = state_machine.msg.interactGoal(interaction=True)
            self.client.send_goal(goal)
            print self.active
            if self.client.wait_for_result():
                result = self.client.get_result()
                if hasattr(result, 'action'):
                    tasks.append(result.action) #add goal to tasks.
                    rospy.sleep(10)
            self.active = False

        def leave(self):
            print "not interact"
            self.client.cancel_all_goals()

    def set_active_state(self, ID):
        print "setting states: ", ID, " active"
        for s in self.states:
            if s.get_id() != ID:
                s.active = False
                # s.get_id(), ": ", s.active
        for s in self.states:
            if s.get_id() == ID:
                s.active = True
                # s.get_id(), ": ", s.active

    def update_state(self, data=None, *args, **kwargs):
        if data is not None and isinstance(data, Int32):
            self.set_active_state(2)
        elif len(tasks) > 0 and not self._interact.active:
            self.set_active_state(1)
        elif len(tasks) == 0 and not self._interact.active and not self._execute.active:
            self.set_active_state(0)

    def __init__(self):
        # self._trigger = rospy.Subscriber("interact_trigger", String, self.update_state);
        self._trigger = rospy.Subscriber("/label_output", Int32, self.update_state)
        self._timer = rospy.Timer(rospy.Duration(5), self.update_state)

        self.states = [self.Interact(2),
                       self.Execute(1),
                       self.Idle(0)]
        self._interact = self.states[0]
        self._execute = self.states[1]
        self._idle = self.states[2]

tasks = []

if __name__ == '__main__':
    rospy.init_node('state_machine_node')
    interface.init()
    StateTracker()
    rospy.spin()
