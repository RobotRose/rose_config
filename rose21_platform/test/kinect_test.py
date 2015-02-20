#!/usr/bin/python

"""Kinect Test.
The Kinect combined with a marker-tracker can estimate the pose of markers in 3D space.
This data has a certain accuracy, precision and jitter/noise and this test wants to analyze that. 

A ground_truth_configuration indicates how far the markers are spaced in reality so this can be compared to the measurements

Usage:
  kinect_test.py measure <ground_truth_configuration> --dir=<measurements_path>
  kinect_test.py analyze <ground_truth_configuration> --dir=<measurements_path>
  kinect_test.py (-h | --help)
  kinect_test.py --version

Options:
  -h --help     Show this screen.
  --version     Show version.
  --dir=<measurements_path>  Where to store/load measurements
"""

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
from math import *
import itertools
import pprint
from termcolor import colored, cprint
from datetime import datetime
import time
from docopt import docopt

import tf
from odometry_test import TfRecorder

CAMERA_FRAME = "/test_camera_link"

#PATTERN A & C
#   0--1--2 every - and | in A are 81.2  +40.6 = 121.8mm = 0.1218m
#   |  |  | every - and | in C are 70.9  +35.4 = 106.3mm = 0.1063m
#   |  |  |
#   3--4--5
#   |  |  |
#   |  |  |  
#   6--7--8


config_A_spacings = {   ('ar_marker_0', 'ar_marker_1')   : 0.1218,
                        ('ar_marker_0', 'ar_marker_3')   : 0.1218,
                        ('ar_marker_1', 'ar_marker_2')   : 0.1218,
                        ('ar_marker_1', 'ar_marker_4')   : 0.1218,
                        ('ar_marker_2', 'ar_marker_5')   : 0.1218,
                        ('ar_marker_3', 'ar_marker_4')   : 0.1218,
                        ('ar_marker_3', 'ar_marker_6')   : 0.1218,
                        ('ar_marker_4', 'ar_marker_5')   : 0.1218,
                        ('ar_marker_4', 'ar_marker_7')   : 0.1218,
                        ('ar_marker_5', 'ar_marker_8')   : 0.1218,
                        ('ar_marker_6', 'ar_marker_7')   : 0.1218,
                        ('ar_marker_7', 'ar_marker_8')   : 0.1218}

config_C_spacings = {   ('ar_marker_0', 'ar_marker_1')   : 0.1063,
                        ('ar_marker_0', 'ar_marker_3')   : 0.1063,
                        ('ar_marker_1', 'ar_marker_2')   : 0.1063,
                        ('ar_marker_1', 'ar_marker_4')   : 0.1063,
                        ('ar_marker_2', 'ar_marker_5')   : 0.1063,
                        ('ar_marker_3', 'ar_marker_4')   : 0.1063,
                        ('ar_marker_3', 'ar_marker_6')   : 0.1063,
                        ('ar_marker_4', 'ar_marker_5')   : 0.1063,
                        ('ar_marker_4', 'ar_marker_7')   : 0.1063,
                        ('ar_marker_5', 'ar_marker_8')   : 0.1063,
                        ('ar_marker_6', 'ar_marker_7')   : 0.1063,
                        ('ar_marker_7', 'ar_marker_8')   : 0.1063}

#PATTERN B
#   09--10--11  Distances between markers are described below:
#   |   |   |
#   |   |   |
#   12--13--14
#   |   |   |
#   |   |   |  
#   15--16--17

config_B_spacings = { ('ar_marker_15', 'ar_marker_16')   : 0.261,
                      ('ar_marker_10', 'ar_marker_11')   : 0.266,
                      ('ar_marker_10', 'ar_marker_9')    : 0.260,
                      ('ar_marker_16', 'ar_marker_17')   : 0.267,
                      ('ar_marker_13', 'ar_marker_16')   : 0.136,
                      ('ar_marker_12', 'ar_marker_13')   : 0.263,
                      ('ar_marker_14', 'ar_marker_17')   : 0.135,
                      ('ar_marker_10', 'ar_marker_13')   : 0.156,
                      ('ar_marker_11', 'ar_marker_14')   : 0.163,
                      ('ar_marker_12', 'ar_marker_9')    : 0.154,
                      ('ar_marker_13', 'ar_marker_14')   : 0.268,
                      ('ar_marker_12', 'ar_marker_15')   : 0.138}

config_D_spacings = {   ('ar_marker_16', 'ar_marker_17')   : 1.247}
config_E_spacings = {   ('ar_marker_16', 'ar_marker_17')   : 1.239}

spacings = {'A':config_A_spacings, 'B':config_B_spacings, 'C':config_C_spacings, 'D':config_D_spacings, 'E':config_E_spacings}
marker_sizes = {'A':4.06, 'B':5.6, 'C':7.09, 'D':12.0, 'E':30.3} #Maker size in centimeters

config_B_Z_heights = {   
                'ar_marker_3': 0.950,    #[m] from ground plane, e.g. the height from /base_link
                'ar_marker_4': 0.945,    #[m] from ground plane, e.g. the height from /base_link
                'ar_marker_5': 0.938,    #[m] from ground plane, e.g. the height from /base_link
                'ar_marker_6': 0.950,    #[m] from ground plane, e.g. the height from /base_link
                'ar_marker_7': 0.945,    #[m] from ground plane, e.g. the height from /base_link
                'ar_marker_8': 0.938,    #[m] from ground plane, e.g. the height from /base_link

                'ar_marker_9':  1.924,    #[m] from ground plane, e.g. the height from /base_link
                'ar_marker_10': 1.926,    #[m] from ground plane, e.g. the height from /base_link
                'ar_marker_11': 1.924,    #[m] from ground plane, e.g. the height from /base_link
                'ar_marker_12': 1.770,    #[m] from ground plane, e.g. the height from /base_link
                'ar_marker_13': 1.770,    #[m] from ground plane, e.g. the height from /base_link
                'ar_marker_14': 1.762,    #[m] from ground plane, e.g. the height from /base_link
                'ar_marker_15': 1.632,    #[m] from ground plane, e.g. the height from /base_link
                'ar_marker_16': 1.633,    #[m] from ground plane, e.g. the height from /base_link
                'ar_marker_17': 1.626    #[m] from ground plane, e.g. the height from /base_link
            }


class KinectTester(object):
    """Measure Kinect accuracy, precision and noise"""

    def __init__(self, filedir, ground_truth_spacings):
        self.tf = tf.TransformListener()
        self.filedir = filedir

        self.timer = None
        tf_timeout = rospy.Duration(0.001)

        self.recorders = {TfRecorder(self.tf, pair[0], pair[1], tf_timeout) for pair, distance in ground_truth_spacings.iteritems()}
        
        markers = sorted(set([marker for pair in ground_truth_spacings.keys() for marker in pair]))
        self.recorders.update({TfRecorder(self.tf, CAMERA_FRAME, marker, tf_timeout) for marker in markers})

    def trigger(self, time):
        rospy.loginfo("Triggering {0} TfRecorders...".format(len(self.recorders)))
        for recorder in self.recorders:
            recorder.record_tf_at(time)
        rospy.loginfo("Triggered {0} TfRecorders".format(len(self.recorders)))

    def start(self, interval=rospy.Duration(0.1)):
        for recorder in self.recorders:
            recorder.start()

        self.timer = rospy.Timer(interval, self.trigger, oneshot=False)

    def stop(self):
        self.timer.shutdown()

        if not os.path.exists(self.filedir):
            os.makedirs(self.filedir)

        for recorder in self.recorders:
            recorder.stop()
            path = os.path.join(self.filedir, recorder.description+".csv")
            recorder.save(path)
            rospy.loginfo("Data is saved to {0}".format(path))


class Validation(object):
    def __init__(self, parent, ground_truth_distance, child):
        self.parent = parent
        self.child = child
        self.ground_truth_distance = ground_truth_distance

        self.measurements = None

    def load_data(self, measurements_path):
        self.measurements = pd.read_csv(measurements_path+"TF-{parent}-{child}.csv".format(parent=self.parent.replace('/',''), child=self.child.replace('/','')))

    def analyze_signal(self, signal):
        basic = {   "Mean [m]"          :   float(signal.mean()),
                    "StdDev [m]"        :   float(signal.std()), 
                    "Var"               :   float(signal.var())}
        return basic

    def compare_signal(self, signal, ground_truth):
        extended = {"Truth [m]"         :   ground_truth,
                    "Error=mean-truth: ":   float(abs(signal.mean() - self.ground_truth_distance))}
        return extended

    def analyze_tf(self):
        def distance(x,y,z):
            import math
            return math.sqrt(x**2 + y**2 + z**2)
        
        extra_stats = {}

        if "marker" in self.parent and "marker" in self.child:
            try:
                self.measurements['Distance'] = self.measurements.apply(lambda row: distance(row['tf.pos.x'], row['tf.pos.y'], row['tf.pos.z']), axis=1)
                extra_stats = self.compare_signal(self.measurements['Distance'], self.ground_truth_distance)
            except:
                pass 

        selected_signals = ['tf.pos.x', 'tf.pos.y', 'tf.pos.z', 'Distance']
        stats = {signal: self.analyze_signal(self.measurements[signal]) for signal in self.measurements.columns if signal in selected_signals}

        if "Distance" in stats: stats["Distance"].update(extra_stats)

        return stats

    def __str__(self):
        return "Validation({0}, {2:.3f}, {1})".format(self.parent, self.child, self.ground_truth_distance if self.ground_truth_distance else 0)


class KinectAnalyzer(object):
    """Analyze Kinect accuracy, precision and noise"""

    def __init__(self, measurements_path, ground_truth_spacings):
        self.measurements_path = measurements_path
        """Initialize an analyzer by providing filepaths with data acquired when running the KinectTester"""

        self.validations = {Validation(pair[0], distance, pair[1]) for pair, distance in ground_truth_spacings.iteritems()}
        
        markers = sorted(set([marker for pair in ground_truth_spacings.keys() for marker in pair]))
        self.validations.update({Validation(CAMERA_FRAME, None, marker) for marker in markers})

    def analyze_tfs(self):
        results = {}
        for validation in self.validations:
            try:
                validation.load_data(self.measurements_path)
                result = validation.analyze_tf()
                results[str(validation)] = result
            except IOError, ioe:
                cprint(ioe, 'red')
        
        return results

    def plot_tf(self, tf_data):
        with pd.plot_params.use('x_compat', True):
            cam_marker.plot(y="tf.pos.x", label="tf.pos.x")
            cam_marker.plot(y="tf.pos.y", label="tf.pos.y")
            cam_marker.plot(y="tf.pos.z", label="tf.pos.z")

if __name__ == '__main__':
    rospy.init_node('kinect_test')

    arguments = docopt(__doc__, version='Kinect Test 2.0')
    # print arguments

    ground_truth_configuration = arguments["<ground_truth_configuration>"]
    marker_size = marker_sizes[ground_truth_configuration]
    ground_truth_spacings = spacings[ground_truth_configuration]
    
    path = arguments["--dir"]
    if not path:
        path = "measurement_{0}".format(datetime.now().strftime("%Y_%m_%d_%H_%M"))
    
    if arguments["measure"]:
        tester = KinectTester(path, ground_truth_spacings)
        rospy.loginfo("Tester created")

        cprint("Please run $ roslaunch rose21_platform kinect_test.launch marker_size:={0}".format(marker_size), 'green')
        raw_input("Press enter when fully launched")
        
        tester.start(interval=rospy.Duration(0.05))

        while not rospy.is_shutdown():
            try:
                time.sleep(1)
            except KeyboardInterrupt:
                break

        tester.stop()
    elif arguments["analyze"]:
        analyzer = KinectAnalyzer(path, ground_truth_spacings)
        results = analyzer.analyze_tfs()
        # pprint.pprint(results)

        # print "-"*20

        import yaml
        summary_path = path+'summary.yml'
        with open(summary_path, 'w') as outfile:
            outfile.write( yaml.dump(results, default_flow_style=False) )
            cprint("Saved analysis summary to {0}".format(summary_path), 'green')