#!/usr/bin/python

"""Odometry Test.
Odometry is estimating *change* in position over time. This test program is meant to measure the accuracy of these estimations.

This program has two main phases: driving and analysis. 

In the driving phase, the robot moves around and data is recorded in a rosbag file. 
The analysis phase, the performance of the odometry process is analysed based on the data in the rosbag file.

Usage:
  odometry_test.py drive [--dir=<measurements_path>] (--start=<startindex> --end=<endindex> | --shape=<shape> | --manual)
  odometry_test.py analyze --dir=<measurements_path> [--rotate_gt=<rotate_gt>]
  odometry_test.py -h | --help
  odometry_test.py --version

Options:
  -h --help     Show this screen.
  --version     Show version.
  --dir=<measurements_path>  Where to store/load measurements
  --start=<startindex>  Which test motion to start with
  --end=<endindex>  Which test motion to end with
  --shape=<shape>  What shape to drive.
  --manual  Drive the robot manually with e.g. a joystick or even push it around
  --rotate_gt An AR marker may be mounted on the robot at an angle (in degrees) around its Z axis. You can correct this with this option
"""

# The requirement specification for odometry specifies that the error in odometry estimates may be 1%.
# If the odometry measures a distance X_odom, then the real distance (measured externally) must be between X_odom * MAX_ERROR and X_odom * -MAX_ERROR.
# The error is defined as error = (X_real - X_odom) / X_odom
# E.g. If X_odom is 5.00m, then to keep the error below 1%, the absolute error must remain less than 0.05m, and thus X_real must be between 5.05m and 4.95m.
# (X_real - X_odom) / X_odom = error
# (5.0    - 4.95)   / 5.0    = 0.01
MAX_ERROR = 0.01

import roslib

import rospy
import std_msgs
import geometry_msgs
from geometry_msgs.msg import Twist, Vector3
import actionlib

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os, sys
from os.path import expanduser
from math import *
import itertools
import pprint
from termcolor import colored, cprint
from datetime import datetime
import time
from docopt import docopt

import tf
import math

from nav_msgs.msg import Odometry

#Standard command velocities used to compose test motions from:
slow_forward    = Twist(Vector3( 0.10,   0   ,   0.0), Vector3(0.0, 0.0,  0  ))
slow_backward   = Twist(Vector3(-0.10,   0   ,   0.0), Vector3(0.0, 0.0,  0  ))
fast_forward    = Twist(Vector3( 0.20,   0   ,   0.0), Vector3(0.0, 0.0,  0  ))
fast_backward   = Twist(Vector3(-0.20,   0   ,   0.0), Vector3(0.0, 0.0,  0  ))
slow_left       = Twist(Vector3( 0.0 ,   0.10,   0.0), Vector3(0.0, 0.0,  0  ))
slow_right      = Twist(Vector3( 0.0 ,  -0.10,   0.0), Vector3(0.0, 0.0,  0  ))
fast_left       = Twist(Vector3( 0.0 ,   0.20,   0.0), Vector3(0.0, 0.0,  0  ))
fast_right      = Twist(Vector3( 0.0 ,  -0.20,   0.0), Vector3(0.0, 0.0,  0  ))
slow_ccw        = Twist(Vector3( 0.0 ,   0   ,   0.0), Vector3(0.0, 0.0,  0.3))
slow_cw         = Twist(Vector3( 0.0 ,   0   ,   0.0), Vector3(0.0, 0.0, -0.3))
fast_ccw        = Twist(Vector3( 0.0 ,   0   ,   0.0), Vector3(0.0, 0.0,  0.6))
fast_cw         = Twist(Vector3( 0.0 ,   0   ,   0.0), Vector3(0.0, 0.0, -0.6))
speeds = [slow_forward, slow_backward, fast_forward, fast_backward, slow_left, slow_right, fast_left, fast_right, slow_ccw, slow_cw, fast_ccw, fast_cw]

durations = [10, 20]#, 30]  # seconds

simple_motions_generator = itertools.product(durations, speeds)
simple_motions = list(simple_motions_generator)
indexed_simple_motions = {index:(motion[1], motion[0]) for index, motion in enumerate(simple_motions)}

def ros_time_to_datetime(ros_time):
    return datetime.fromtimestamp(ros_time.to_sec())


class Recorder(object):
    
    """Baseclass for data recorders."""
    def __init__(self, description, headers=None, autostart=False):
        """Initialize a new Recorder. By default, it does not start recording directly, only after calling start()
        @param description what data is this recorder recording.
        @param headers What columns to create in the datastore?
        @param autostart Start recording directly or not?"""
        self.dataframe = pd.DataFrame(columns=headers)
        self.dataframe.index.name = "timestamp"

        self.description = description

        self.recording = autostart

    def add_row(self, index, row):
        """Add a row to the datastore at a given index
        @param index may be a timestamp
        @param row a list of data. Must correspond with the headers passed at construction
        """
        self.dataframe.loc[index] = row

    def start(self):
        self.recording = True

    def stop(self):
        self.recording = False

    def save(self, filepath):
        self.dataframe.to_csv(path_or_buf=filepath)


class OdomRecorder(Recorder):
    
    """Records odometry messages for later analysis.
    An nav_msgs/Odometry message look like this:
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    string child_frame_id
    geometry_msgs/PoseWithCovariance pose
      geometry_msgs/Pose pose
        geometry_msgs/Point position
          float64 x
          float64 y
          float64 z
        geometry_msgs/Quaternion orientation
          float64 x
          float64 y
          float64 z
          float64 w
      float64[36] covariance
    geometry_msgs/TwistWithCovariance twist
      geometry_msgs/Twist twist
        geometry_msgs/Vector3 linear
          float64 x
          float64 y
          float64 z
        geometry_msgs/Vector3 angular
          float64 x
          float64 y
          float64 z
      float64[36] covariance

      Only the pose.position and pose.orientation are recorded with their timestamp
    """
    def __init__(self):
        headers = [ "pose.pos.x", "pose.pos.y","pose.pos.z", 
                    "pose.ori.x", "pose.ori.y", "pose.ori.z", "pose.ori.w", 
                    "twist.lin.x", "twist.lin.y","twist.lin.z", 
                    "twist.ang.x", "twist.ang.y", "twist.ang.z"]
        Recorder.__init__(self, description="odometry", headers=headers) #We want to save 14 datafields of each Odom-message
        
        self.odom_subscriber = rospy.Subscriber(
            "/odom", Odometry, self.process_odometry_msg)

        self.integratedTwist = geometry_msgs.msg.Twist()
        self.startPose = geometry_msgs.msg.Pose()

        self.latestPose = None

        self.integratedAngleZ = 0
        self.previousOdomStamp = rospy.Time.now()
        

        self._listeners = [] #listeners is a list of functions that are called when an odom-message is received

    def add_listener(self, listener):
        """This recorder also serves as a trigger for other functions/recorders. You can add functions to be triggered here."""
        self._listeners += [listener]

    def process_odometry_msg(self, odom):
        """Record odometry message and trigger other listeners for this event"""
        if self.recording:
            row = [ odom.pose.pose.position.x,
                    odom.pose.pose.position.y,
                    odom.pose.pose.position.z,
                    odom.pose.pose.orientation.x,
                    odom.pose.pose.orientation.y,
                    odom.pose.pose.orientation.z,
                    odom.pose.pose.orientation.w,
        
                    odom.twist.twist.linear.x,
                    odom.twist.twist.linear.y,
                    odom.twist.twist.linear.z,
                    odom.twist.twist.angular.x,
                    odom.twist.twist.angular.y,
                    odom.twist.twist.angular.z]
            self.add_row(ros_time_to_datetime(odom.header.stamp), row)

        for listener in self._listeners:
            listener(odom.header.stamp)


class TfRecorder(Recorder):

    """Records TF messages for later analysis"""
    def __init__(self, listener, target_frame, source_frame, timeout=rospy.Duration(1.0)):
        headers = [ "tf.pos.x", "tf.pos.y","tf.pos.z", 
                    "tf.ori.x", "tf.ori.y", "tf.ori.z", "tf.ori.w"]
        Recorder.__init__(self, headers=headers, description="TF-{0}-{1}".format(target_frame, source_frame).replace("/",'')) # 8 columns: time, position.{x,y,z}, orientation.{x,y,z,w}
        self.tf = listener
        self.timeout = timeout

        self.target_frame, self.source_frame = target_frame, source_frame

    def record_tf_at(self, time):
        if self.recording:
            try:
                self.tf.waitForTransform(self.target_frame, self.source_frame, rospy.Time(0), self.timeout)
                time = self.tf.getLatestCommonTime(self.target_frame, self.source_frame)
                position, quaternion = self.tf.lookupTransform(self.target_frame, self.source_frame, time)  # -> position, quaternion

                row = [ position[0], position[1], position[2],
                        quaternion[0], quaternion[1], quaternion[2], quaternion[3]]

                self.add_row(ros_time_to_datetime(time), row)
            except tf.Exception, e:
                rospy.logerr(e)


class RoseBase(object):

    """Controls Robot Rose's base"""

    def __init__(self):
        self._cmd_vel = rospy.Publisher('/cmd_vel', Twist)

    def move(self, goal):
        """Start moving to the specified goal
        @param goal PoseStamped where the robot should move to"""
        pass

    def force_drive_raw(self, vx, vy, vth, duration):
        """Drive the robot with the given speeds in x, y and theta (around Z) for some duration."""
        v = Twist()        # Initialize velocity
        v.linear.x = vx
        v.linear.y = vy
        v.angular.z = vth
       
        return self.force_drive(v, duration)  

    def force_drive(self, twist, duration):
        """Drive the robot with the given Twist for some duration.
        The robot is stopped after this duration"""
        t_start = rospy.Time.now()

        # Drive
        while (rospy.Time.now() - t_start) < rospy.Duration(duration) and not rospy.is_shutdown():
            try:
                self._cmd_vel.publish(twist)
                rospy.sleep(0.1)
            except KeyboardInterrupt:
                return False

        # Stop driving
        stop = Twist()        # Initialize zero velocity
        self._cmd_vel.publish(stop)

        return True

    def get_location(self):
        return get_location(self._robot_name, self._tf_listener)

    def set_initial_pose(self, x, y, phi):

        initial_pose = geometry_msgs.msg.PoseWithCovarianceStamped()

        initial_pose.header.frame_id = "/map"

        initial_pose.pose.pose.position.x = x
        initial_pose.pose.pose.position.y = y
        initial_pose.pose.pose.position.z = 0.0
        initial_pose.pose.pose.orientation = transformations.euler_z_to_quaternion(phi)
        initial_pose.pose.covariance = [0.25,  0.0,  0.0,  0.0,  0.0,  0.0, 
                                         0.0, 0.25,  0.0,  0.0,  0.0,  0.0, 
                                         0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  
                                         0.0,  0.0,  0.0,  0.06853891945200942, 0.0, 0.0, 
                                         0.0,  0.0,  0.0,  0.0,  0.0,  0.0, 
                                         0.0,  0.0,  0.0,  0.0,  0.0,  0.0]

        #rospy.loginfo("initalpose = {0}".format(initial_pose))

        self._initial_pose_publisher.publish(initial_pose)

        return True


def get_location(tf_listener):
    try:
        time = rospy.Time.now()
        tf_listener.waitForTransform(
            "/map", "/base_link", time, rospy.Duration(20.0))
        (ro_trans, ro_rot) = tf_listener.lookupTransform(
            "/map", "/base_link", time)

        position = geometry_msgs.msg.Point()
        orientation = geometry_msgs.msg.Quaternion()

        position.x = ro_trans[0]
        position.y = ro_trans[1]
        orientation.x = ro_rot[0]
        orientation.y = ro_rot[1]
        orientation.z = ro_rot[2]
        orientation.w = ro_rot[3]

        target_pose = geometry_msgs.msg.PoseStamped(
            pose=geometry_msgs.msg.Pose(position=position, orientation=orientation))
        target_pose.header.frame_id = "/map"
        target_pose.header.stamp = time
        return target_pose

    except (tf.LookupException, tf.ConnectivityException):
        rospy.logerr("tf request failed!!!")
        target_pose = geometry_msgs.msg.PoseStamped(
            pose=geometry_msgs.msg.Pose(position=position, orientation=orientation))
        target_pose.header.frame_id = "/map"
        return target_pose

class TfHelper(object):

    """Publish TF transform at some rate"""
    def __init__(self, broadcaster, pos, quat, child, parent, interval=rospy.Duration(0.1)):
        self.broadcaster = broadcaster
        self.pos, self.quat = pos, quat
        self.parent, self.child = parent, child
        self.timer = rospy.Timer(interval, self.publish, oneshot=False)

    def publish(self, *args, **kwargs):
        try:
            self.broadcaster.sendTransform(self.pos, self.quat, rospy.Time.now(), self.child, self.parent)
        except rospy.ROSException, e:
            # import ipdb; ipdb.set_trace()
            rospy.logerr(e)

    def stop(self):
        self.timer.shutdown()


class OdomTester(object):

    """Main class for testing Odometry. 
    Sets up a bunch of TF-transforms to make analysis easier and records data for later analysis. 
    Also allows to drive the robot around autonomously"""

    ODOM_FRAME = "/odom"
    MEASURED_BASE_FRAME = "/base_link"
    WORLD_FRAME = "/world_marker"
    TRUE_BASE_FRAME = "/base_link_observed"
    ROBOT_MARKER_FRAME = "/robot_marker_1"

    def __init__(self, base, filedir):
        """Initialize a new OdomTester. Call start() before actually starting
        @param base A RoseBase-instance to drive the robot around with
        @param filedir where to store several data files?"""
        self.base = base
        self.filedir = filedir

        self.tf = tf.TransformListener()
        self.odom_tf_recorder = TfRecorder(self.tf, OdomTester.MEASURED_BASE_FRAME, OdomTester.MEASURED_BASE_FRAME+"_start")
        self.groundtruth_tf_recorder = TfRecorder(self.tf, OdomTester.ROBOT_MARKER_FRAME, OdomTester.ROBOT_MARKER_FRAME+"_start")
        self.odom_recorder = OdomRecorder()

        self.recorders = [self.odom_tf_recorder, self.groundtruth_tf_recorder, self.odom_recorder]

        self.odom_recorder.add_listener(self.odom_tf_recorder.record_tf_at)
        self.odom_recorder.add_listener(self.groundtruth_tf_recorder.record_tf_at)

        self.measurement_reference = None
        self.groundtruth_reference = None

        self.shapes = {"square2":self.test_square2, "square3":self.test_square3}

    def start(self):
        """Sets up s needed TF transforms and starts recording data"""
        self.store_start_frames()

        for recorder in self.recorders:
            recorder.start()

    def stop(self):
        """Stps all data recording and saves their results"""
        if not os.path.exists(self.filedir):
            os.makedirs(self.filedir)

        for recorder in self.recorders:
            recorder.stop()
            path = os.path.join(self.filedir, recorder.description+".csv")
            recorder.save(path)
            rospy.loginfo("Data is saved to {0}".format(path))

        for tfHelper in [self.measurement_reference, self.groundtruth_reference]:
            tfHelper.stop()

    def store_start_frames(self):
        """Before a test, the frame (/odom --> /base_link) is stored and keeps publishing as (/odom --> /base_link_start).
        During the test, the transform (/base_link_start --> /base_link) can be recorded for the odometry version of the trajectory.
        Similarly, (/world_marker --> /robot_marker_1) is stored and keeps publishing as (/world_marker --> /robot_marker_1_start).
        During the test, (/robot_marker_1_start --> /robot_marker_1) is recorded for the ground truth version of the trajectory.
        In the end, these can be compared of course."""
        self.tf.waitForTransform(OdomTester.MEASURED_BASE_FRAME, OdomTester.ODOM_FRAME, rospy.Time(0), rospy.Duration(5.0))
        base_in_odom_start      = self.tf.lookupTransform(OdomTester.MEASURED_BASE_FRAME, OdomTester.ODOM_FRAME, rospy.Time(0))

        self.tf.waitForTransform(OdomTester.ROBOT_MARKER_FRAME, OdomTester.WORLD_FRAME, rospy.Time(0), rospy.Duration(5.0))
        mark_in_world_start   = self.tf.lookupTransform(OdomTester.ROBOT_MARKER_FRAME, OdomTester.WORLD_FRAME, rospy.Time(0))

        br = tf.TransformBroadcaster()
        self.measurement_reference = TfHelper(br, base_in_odom_start[0],  base_in_odom_start[1],  OdomTester.MEASURED_BASE_FRAME+"_start",  OdomTester.ODOM_FRAME)
        self.groundtruth_reference = TfHelper(br, mark_in_world_start[0], mark_in_world_start[1], OdomTester.ROBOT_MARKER_FRAME+"_start",   OdomTester.WORLD_FRAME)

        rospy.loginfo("Waiting for TFs to appear")
        time.sleep(0.5)

    def test_simple_motions(self, start=0, end=None):
        """Perform selected simple motions
        @param start start of motion selection
        @param end end of motion selection"""
        cprint("Stand back. Rose is doing SCIENCE!", 'green', attrs=['blink'])
        
        if not end: end = len(simple_motions) - 1

        selected_indices = range(start, end+1)

        selected_motions = {index:motion for index, motion in indexed_simple_motions.iteritems() if index in selected_indices}

        print "Testing {0} motions with indices {1}:".format(len(selected_motions), selected_indices)
        for index, motion in indexed_simple_motions.iteritems():
            motion_description = "Motion {0}\t: Linear: ({1.linear}), Angular: ({1.angular}) for {2}s".format(index, motion[0], motion[1]).replace("\n", " ")
            if not index in selected_indices:
                cprint(motion_description, 'grey')
            else:
                cprint(motion_description, 'white', attrs=['bold'])

        print "-"* 20

        for index, motion in selected_motions.iteritems():
            try:
                if not rospy.is_shutdown():
                    time.sleep(1)
                    self.test_motion(index, motion)
            except KeyboardInterrupt:
                return

    def test_motion(self, index, motion):
        """Perform a single motion"""
        # import ipdb; ipdb.set_trace()
        print "Executing motion {0}\t: Linear: ({1.linear}), Angular: ({1.angular}) for {2}s".format(index, motion[0], motion[1]).replace("\n", " ")
        
        self.base.force_drive(motion[0], motion[1])

    def test_square2(self):
        """Drive in a square with turns in the corners"""
        cprint("Stand back. Rose is doing SCIENCE in a square with turning in the corners!", 'green', attrs=['blink'])
        self.base.force_drive(Twist(Vector3( 0.1 ,   0   ,   0.0), Vector3(0.0, 0.0, 0.0)), 10) #1m forward
        self.base.force_drive(Twist(Vector3( 0.0 ,   0.1 ,   0.0), Vector3(0.0, 0.0, 0.0)), 10) #1m left
        self.base.force_drive(Twist(Vector3( 0.0 ,   0.1 ,   0.0), Vector3(0.0, 0.0, -0.31415)), 5) #90deg counter clockwise
        self.base.force_drive(Twist(Vector3( 0.0 ,  -0.1 ,   0.0), Vector3(0.0, 0.0, 0.0)), 10) #1m right
        self.base.force_drive(Twist(Vector3( 0.1 ,   0   ,   0.0), Vector3(0.0, 0.0, 0.0)), 10) #1m forward
        self.base.force_drive(Twist(Vector3( 0.0 ,   0.1 ,   0.0), Vector3(0.0, 0.0,0.31415)), 5) #90deg clockwise

    def test_square3(self):
        """Drive in a square with strafing, not turning"""
        cprint("Stand back. Rose is doing SCIENCE in a square with strafing", 'green', attrs=['blink'])
        self.base.force_drive(Twist(Vector3( 0.1 ,   0   ,   0.0), Vector3(0.0, 0.0, 0.0)), 10) #1m forward
        self.base.force_drive(Twist(Vector3( 0.0 ,   0.1 ,   0.0), Vector3(0.0, 0.0, 0.0)), 10) #1m left
        self.base.force_drive(Twist(Vector3(-0.1 ,   0.0 ,   0.0), Vector3(0.0, 0.0, 0.0)), 10) #1m backward
        self.base.force_drive(Twist(Vector3( 0.0 ,  -0.1 ,   0.0), Vector3(0.0, 0.0, 0.0)), 10) #1m right


class OdomAnalyzer(object):

    """Analyze odometry performance based on 3 data streams: 
    - external TF measurements (from a kinect as ground truth)
    - Odometry messages,
    - internal TF measurements (from odometry)"""

    def __init__(self, path):
        """Initialize an analyzer by providing filepaths with data acquired when running the OdomTester
        @param path where to load data files from, as saved by OdomTester"""
        self.path = path

        self.ground_truth_tf    = pd.read_csv( os.path.join(path, "TF-robot_marker_1-robot_marker_1_start.csv"))
        self.internal_tf        = pd.read_csv( os.path.join(path, "TF-base_link-base_link_start.csv"))
        self.messages           = pd.read_csv( os.path.join(path, "odometry.csv"))

    def preprocess_data(self, rotate_ground_truth=0):
        #Align the use timestamps as index
        data = [self.ground_truth_tf, self.internal_tf, self.messages]
        for datum in data:
            datum.set_index(datum.columns[0], inplace=True)
            datum.index.name="timestamp"
            datum.index = pd.DatetimeIndex(datum.index)


        self.ground_truth_tf = OdomAnalyzer.rotate(self.ground_truth_tf, math.radians(rotate_ground_truth))  # Give ground_truth the same orientation as odom (manually entered by user!)
        self.ground_truth_tf = OdomAnalyzer.subtract_delta(self.ground_truth_tf, 
                                                            OdomAnalyzer.startpoint_delta(self.ground_truth_tf, self.internal_tf))  # Then align the starts of both trajectories for easy comparision

        OdomAnalyzer.append_distance(self.ground_truth_tf)
        OdomAnalyzer.append_distance(self.internal_tf)

        self.messages = OdomAnalyzer.remap_axes_inverse(self.messages)  # odom TF and odom messages are the inverse of each other. This method undo's the inversion by inverting again.

        self.ground_truth_tf_downsampled = self.ground_truth_tf.resample('500L', closed='left', label='left') #Resample to... S for seconds, L for millis
        self.internal_tf_downsampled     = self.internal_tf.resample(    '500L', closed='left', label='left')

    @staticmethod
    def append_distance(dataframe):
        diff = dataframe.diff()
        dataframe["delta"] = np.sqrt(np.square(diff["tf.pos.x"]) + np.square(diff["tf.pos.y"]))
        dataframe["distance"] = dataframe["delta"].abs().cumsum()
        return diff

    @staticmethod
    def error(A, B):
        """Error = sqrt((A.x-B.x)^2 + (A.y-B.y)^2)"""
        error = np.sqrt(np.square(A["tf.pos.x"]-B["tf.pos.x"]) + np.square(A["tf.pos.y"]-B["tf.pos.y"]))
        return error

    @staticmethod
    def rotate(dataframe, angle_rad):
        """Rotate the trajectory in the dataframe"""
        remapped = pd.DataFrame()
        remapped['tf.pos.x'] = dataframe.apply(lambda row: row['tf.pos.x']*cos(angle_rad) - row['tf.pos.y']*sin(angle_rad), axis=1)
        remapped['tf.pos.y'] = dataframe.apply(lambda row: row['tf.pos.x']*sin(angle_rad) + row['tf.pos.y']*cos(angle_rad), axis=1)
        remapped.index = dataframe.index
        return remapped

    @staticmethod
    def remap_axes_inverse(dataframe):
        """Mirror a trajectory in both X and Y"""
        remapped = pd.DataFrame()
        remapped['pose.pos.x'] = dataframe['pose.pos.x'].apply(lambda x: -x)
        remapped['pose.pos.y'] = dataframe['pose.pos.y'].apply(lambda y: -y)
        remapped.index = dataframe.index
        return remapped
    
    @staticmethod
    def startpoint_delta(A, B):
        """Calculate the difference between the start of 2 trajectories"""
        delta = {'tf.pos.x':A['tf.pos.x'][0]-B['tf.pos.x'][0], 
                 'tf.pos.y':A['tf.pos.y'][0]-B['tf.pos.y'][0]}
        return delta

    @staticmethod
    def subtract_delta(original, delta):
        """Remove some difference from a dataframe"""
        shifted = pd.DataFrame()
        shifted['tf.pos.x'] = original['tf.pos.x'].apply(lambda x: x-delta['tf.pos.x'])
        shifted['tf.pos.y'] = original['tf.pos.y'].apply(lambda x: x-delta['tf.pos.y'])
        shifted.index = original.index
        return shifted

    def plot(self, show=False):
        """Analyze the data loaded at construction and plot the results in the path given at construction
        @param rotate_ground_truth An AR-marker may be attached to the robot at an angle. 
            This allows you to correct this by giving a rotation angle (in degrees) by which to rotate the ground truth trajectory with"""

        def make_title():
            """Use the path to generate a title/filename for the plot"""
            head, tail = os.path.split(self.path)
            if not tail:
                head, tail = os.path.split(head)
            title = tail + "_plot.png"
            return title

        axes = plt.subplot(2,1,1)
        
        title = make_title()
        plt.title(title)
        
        axes.plot(self.ground_truth_tf["tf.pos.x"], self.ground_truth_tf["tf.pos.y"], label="ext. TF")
        axes.plot(self.internal_tf["tf.pos.x"], self.internal_tf["tf.pos.y"], label="int. TF")
        axes.plot(self.messages["pose.pos.x"], self.messages["pose.pos.y"], label="odom")
        axes.set_aspect("equal")
        # axes.legend(loc='best') #Outcommented now, but may be enabled when needed.

        Xa = self.ground_truth_tf_downsampled["tf.pos.x"]
        Ya = self.ground_truth_tf_downsampled["tf.pos.y"]
        Xb = self.internal_tf_downsampled["tf.pos.x"]
        Yb = self.internal_tf_downsampled["tf.pos.y"]
        U =  Xb - Xa
        V =  Yb - Ya

        axes.quiver(Xa,Ya, U,V,angles='xy',scale_units='xy',scale=1, color=(0, 0, 0, 0.5))

        ax2 = plt.subplot(2,1,2)
        # ax2.set_aspect("equal") #Not useful for more accurate odometry because the data on the Y axis will be small
        error = OdomAnalyzer.error(self.ground_truth_tf_downsampled, self.internal_tf_downsampled)
        error_analysis = pd.DataFrame()
        error_analysis["distance"] = self.internal_tf_downsampled["distance"]
        error_analysis["error"] = error
        error_analysis["max_error"] = [MAX_ERROR]*len(error_analysis)
        error_analysis["error/distance"] = error_analysis["error"] / error_analysis["distance"]
        error_analysis["diff(error)*10"] = error_analysis["error"].diff() * 10
        
        error_analysis.dropna(inplace=True)  # delete rows with NaN's
        error_analysis.plot(x='distance', ax=ax2) #, legend=False
        
        plotpath = os.path.join(self.path, title)
        plt.savefig(plotpath)
        if show: plt.show()

        print "{0}".format(plotpath)

        max_error_over_distance = max(error_analysis["error/distance"])
        if max_error_over_distance > MAX_ERROR:
            cprint("The max error/distance is {0:.4f} > {1}".format(max_error_over_distance, MAX_ERROR), 'red')
        else:
            cprint("The max error/distance is {0:.4f} < {1}".format(max_error_over_distance, MAX_ERROR), 'green')

if __name__ == '__main__':
    arguments = docopt(__doc__, version='Odometry Test 1.1')

    path = arguments["--dir"]
    if not path:
        path = "measurement_{0}".format(datetime.now().strftime("%Y_%m_%d_%H_%M"))

    if arguments["drive"]:
        rospy.init_node('odom_test')
        base = RoseBase()
        tester = OdomTester(base, path)
        rospy.loginfo("Tester created")
    
        tester.start()

        if arguments["--manual"]:
            cprint("Please drive yourself. Be sure to keep the markers both in test-camera sight.", 'green')
            while not rospy.is_shutdown():
                rospy.sleep(0.5)
        elif arguments["--start"] != None and arguments["--end"] != None:
            tester.test_simple_motions(start=arguments["--start"], end=arguments["--end"])
        elif arguments["--shape"]:
            try:
                tester.shapes[arguments['--shape']]()
            except KeyError, ke:
                cprint("No such shape '{0}'".format(arguments["--shape"]))

        tester.stop()
        
    elif arguments["analyze"]:
        analyzer = OdomAnalyzer(expanduser(path))

        rotate = 0
        if arguments["--rotate_gt"]: rotate = float(arguments["--rotate_gt"])  # Allows the user to correct a mistake in test setup :-(
        
        analyzer.preprocess_data(rotate)
        analyzer.plot()




