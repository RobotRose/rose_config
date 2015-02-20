#!/usr/bin/python
"""Rose's wheel units make Rose move around. This file contains some classes to help test their accuracy and repeatablity"""

import roslib

import rospy
import std_msgs
import geometry_msgs

import numpy as np
import sys
from math import *
import actionlib

from rose_base_msgs.msg import wheelunit_statesAction, wheelunit_statesActionGoal, wheelunit_states

class WheelUnitTester(object):
    """Test accuracy and precision of wheel unit control"""
    def __init__(self, base):
        self.client = actionlib.SimpleActionClient(
            '/platform_controller', wheelunit_statesAction)

    def test_angles(self):
        rospy.loginfo("Waiting for server")
        self.client.wait_for_server()

        rospy.loginfo("Server found, sending goal")
        #import ipdb;ipdb.set_trace()
        # break
        for goal in self.generate_angle_goals():
            self.client.send_goal_and_wait(
                goal.goal, execute_timeout=rospy.Duration.from_sec(5.0))
            rospy.sleep(rospy.Duration.from_sec(1.0))

    def generate_angle_goals(self, start=-500, stop=500, step=100, reset_angle=True):
        for angle in range(start, stop, step):
            goal = wheelunit_statesActionGoal()
            goal.goal.requested_state.angle_FL = angle
            yield goal
            goal.goal.requested_state.angle_FL = 0
            yield goal

        for angle in range(start, stop, step):
            goal = wheelunit_statesActionGoal()
            goal.goal.requested_state.angle_FR = angle
            yield goal
            goal.goal.requested_state.angle_FR = 0
            yield goal

        for angle in range(start, stop, step):
            goal = wheelunit_statesActionGoal()
            goal.goal.requested_state.angle_BL = angle
            yield goal
            goal.goal.requested_state.angle_BL = 0
            yield goal

        for angle in range(start, stop, step):
            goal = wheelunit_statesActionGoal()
            goal.goal.requested_state.angle_BR = angle
            yield goal
            goal.goal.requested_state.angle_BR = 0
            yield goal

    # /platform_controller goals does not use angles but rather something more linked to the controllers.
    def test_turning_positioning_accuracy_and_repeatability(self, wheel, signal=500, sleep_seconds=-1):
        """Test whether the wheel orientations are accurate and repatable. 
        Expectations: small angular deviations in expected frame pose.
        Measurement method: Angular measurement at different steering positions. Repeat 10x"""
        def generate_goals():
            for i in range(10):
                goal = wheelunit_statesActionGoal()
                setattr(goal.goal.requested_state, wheel, signal)
                yield goal
                setattr(goal.goal.requested_state, wheel, 0)
                yield goal

        rospy.loginfo("Waiting for server")
        self.client.wait_for_server()

        for goal in generate_goals():
            if not rospy.is_shutdown():
                rospy.logdebug(
                    "Sending & awaiting goal {0}".format(goal.goal.requested_state))
                self.client.send_goal_and_wait(
                    goal.goal, execute_timeout=rospy.Duration.from_sec(5.0))
                if sleep_seconds != -1:
                    rospy.sleep(rospy.Duration.from_sec(sleep_seconds))
                else:
                    comment = raw_input("Continue? ")
                    rospy.loginfo(comment)

    def forward(self, movetime, movespeed):
        goal = wheelunit_statesActionGoal()
        goal.goal.requested_state = wheelunit_states(
            0, 0, 0, 0, movespeed, movespeed, movespeed, movespeed)  # slow forward
        rospy.loginfo("Requesting to move with speed {0}".format(movespeed))
        self.client.send_goal_and_wait(
            goal.goal, execute_timeout=rospy.Duration.from_sec(5.0))

        rospy.loginfo("Moving forward for {0} seconds...".format(movetime))
        rospy.sleep(rospy.Duration.from_sec(movetime))

    def stop(self, stoptime=10):
        goal = wheelunit_statesActionGoal()
        rospy.loginfo("Stopping")
        goal.goal.requested_state = wheelunit_states(
            0, 0, 0, 0, 0, 0, 0, 0)  # stop
        self.client.send_goal_and_wait(
            goal.goal, execute_timeout=rospy.Duration.from_sec(5.0))
        rospy.sleep(rospy.Duration.from_sec(stoptime))

    def backward(self, movetime, movespeed):
        goal = wheelunit_statesActionGoal()
        goal.goal.requested_state = wheelunit_states(
            0, 0, 0, 0, -movespeed, -movespeed, -movespeed, -movespeed)  # slow backward
        rospy.loginfo("Requesting to move with speed {0}".format(movespeed))
        self.client.send_goal_and_wait(
            goal.goal, execute_timeout=rospy.Duration.from_sec(5.0))

        rospy.loginfo("Moving backward for {0} seconds...".format(movetime))
        rospy.sleep(rospy.Duration.from_sec(movetime))

    def moving_accuracy_and_repeatability_forward(self, movetime=2, movespeed=5000):
        """Test the moving accuracy and repeatability.
        Expectations are hat there will be a small deviation in both X and Y
        Method: Move back and forth over a straight line for some time at a given speed. 
        Then, move in the inverse direction, so the same speed * -1 for the same duration.

        At the start location, place a mark on a piece of tape on the wheels and the ground, with a stripe where they meet. 
        After going forward, the robots waits some time (so we can measure the distance to the start location).
        Then the robot moves back to where it supposedly was. This will have show some inaccuracy in the X and Y direction."""
        rospy.loginfo("Waiting for server")
        self.client.wait_for_server()

        self.forward(movetime, movespeed)
        self.stop(stoptime=10)
        self.backward(movetime, movespeed)
        self.stop(stoptime=0)    

    def moving_accuracy_and_repeatability_backward(self, movetime=2, movespeed=5000):
        """Test the moving accuracy and repeatability.
        Expectations are hat there will be a small deviation in both X and Y
        Method: Move back and forth over a straight line for some time at a given speed. 
        Then, move in the inverse direction, so the same speed * -1 for the same duration.

        At the start location, place a mark on a piece of tape on the wheels and the ground, with a stripe where they meet. 
        After going forward, the robots waits some time (so we can measure the distance to the start location).
        Then the robot moves back to where it supposedly was. This will have show some inaccuracy in the X and Y direction."""
        rospy.loginfo("Waiting for server")
        self.client.wait_for_server()

        self.backward(movetime, movespeed)
        self.stop(stoptime=10)
        self.forward(movetime, movespeed)
        self.stop(stoptime=0)

if __name__ == '__main__':
    rospy.init_node('wheelunits_test')

    tester = WheelUnitTester()
    rospy.loginfo("Tester created")

    def moving_accuracy_and_repeatability_backward():
        tester.moving_accuracy_and_repeatability_backward(
            movetime=5, movespeed=10000)

    def moving_accuracy_and_repeatability_forward():
        tester.moving_accuracy_and_repeatability_forward(
            movetime=5, movespeed=10000)
