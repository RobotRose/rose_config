Odometry
============

Introduction
------------
Odometry is the use of motion data to estimate the robot's (change in) position over time.
For Robot Rose, this means using wheel orientations and rotations to do that.
The node platform_tf listens to the states of the wheelunits and publishes both a ROS Odometry message and a transform between the /odom-frame and the /base_link frame. 

An Odometry message consists of:

- a PoseWithCovariance of the /base_link frame relative to the /odom-frame, indicating the position of the robot
- a TwistWithCovariance of the /base_link frame, indicating its speed. 
The TF between /odom and /base_link should be corresponding with the PoseWithCovariance.

Odometry, in Robot Rose, is used as the motion update for AMCL and for relative positioning.
Odometry can generally not be used for long-distance position estimation.
This is because of the inherent inaccuracy of odometry introduced through slip and other unmeasurable inaccuracies.

If these inaccuracies are known, they can be an input for AMCL as well. 

The task of odometry is to measure change in position an orientation (which together comprise the robot's *pose*). 
Odometry can be judged by the accuracy and precision of those measurements.


Requirements
------------

### Functional
ID | Description       											                                                                  | Motivation 
---|:----------------------------------------------------------------------------------------------------------------------------:|-----------
F1 | The odometry process must be accurate                                                                                        | Accurately knowing where the robot is, is essential for fast navigation and positioning
F2 | The odometry process must adhere to the [ROS odometry description](http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom).| Integration with rest of system
F3 | The odometry process must inform other process with a high enough frequency.												  | Integration with rest of system

### Technical
ID |Rel. to | Description                                                                              | Motivation 
---|--------|:----------------------------------------------------------------------------------------:|-----------
T1 | F1     | Measurements in change of position shall be accurate to 1%.                              | Errors larger than 1cm will be corrected by a higher level process (AMCL etc.)
T2 | F1     | Measurements in change of orientation shall be accurate to 1%                            | The arm can compensate orientation errors in the base up to a few degrees.
T3 | F2     | TF transform /odom to /base_link shall correspond with pose-field in an Odometry message | Both sources of information must be consistent
T4 | F3     | Odometry messages shall be published at a rate of 50Hz                                   | Both sources of information must be consistent
T5 | F1     | Measurements of speed shall be accurate to 10%                                           | TODO
T6 | F1,F3  | Timestamps of Odometry messages are accurate TODO                                        | TODO

Dependencies
------------
On which functions, nodes etc does this functionality directly rely?

* wheel_controller

The test also relies on:

* ar_track_alvar
* External Kinect

Test setup
----------
 
 - All dependencies are met
 - All dependent nodes are running and validated
 - Flat terrain
 - Indoors
 - No obstacles
 - Linoleum floor
 - A Kinect mounted overhead with ar_track_alvar listening to its data.

The ar_track_alvar measurements itself also have some built-in inaccuracies. 
The accuracy of ar_track_alvar's measurements (of distance between markers) at a distance of 1.5m are better than 1mm. 

To set up all nodes:

- rosepc2 $ roslaunch rose21_platform platform.launch
- cockpit $ roslaunch rose21_platform odometry_test.launch
- cockpit $ rosrun rose21_platform odometry_test.py drive

Tests and Results
-----------------

To validate that the odometry process meets its requirements, we have to compare its output with an external source of reference.
Odometry measures position and speed and is required to output an update for these values at a certain rate.
We thus have to measure:

- The update rate
- Real position of the robot
- Real speed of the robot

Measuring position/relative distances in free space with a tape measure is time consuming and not accurate enough for the purpooses of this test.
Instead, an automated, marker-based approach with a Kinect 3D camera will be used. 
To get the most accurate measurement, the Kinect must measure relative distances between markers rather than absolute distances to the Kinect itself. 
Therefore, markers will be attached to both the ground plane and the base of Robot Rose. 
Rose will get 2 markers to more accurately measure orientation.
A Kinect will be mounted over head so it can observe the markers in a wide area.

### Procedure
To test the accuracy and precision of odometry, a command to move is sent to the robot by publishing a speed for some time on the /cmd_vel-topic.
What this movement command exactly is does not matter per se for this test:
only the real change in position and orientation compared to the measured (by odometry) change is relevant. 
The accuracy of measurements may however be affected by the speed or the robot and perhaps the change in speed (acceleration).

The real speed of the robot will be determined by differentiating the measured positions: the difference between two positions divided by the difference in time. 

Each test will follow this procedure:
0. A ROS bag  file is starting to record all the relevant topics: /tf, /odom and /wheel_controller/wheelunit_states. This can be used to analyse the raw data again after the test.
1. The positions of the /base_link-frame and the /robot_marker_1-frame are remembered and published during the test with a _start-postfix. 
	This allows to get the position of /base_link relative to /base_link_start etc during the test. 
2. We start recording Odometry messages and the transforms between /base_link relative to /base_link_start and /robot_marker_1 relative to /robot_marker_1_start
3. The robot is given a command to drive some speed for some time to eventually drive some distance. (There is no need to exactly control the distance, only to measure it accurately)
4. Analyze the end-poses and trajectories from both the kinect/markers and from odometry.

### Test 1: Simple driving

ID  | Test parameters                                        | | | | Real values (measured by ar_track_alvar) | | | Measured/Odometry values       | | | Error (max 1% of real values)     | | |
----|:------------------------------------------------------:|-|-|-|------------------------------------------|-|-|--------------------------------|-|-|-----------------------------------|-|-|
    | duration[s] | speed_X[m/s] | speed_Y[m/s] | speed_Phi[rad/s] | delta_X[m]  | delta_Y[m] | delta_Phi[m] | odom_X[m] | odom_Y[m] | odom_Phi[m]| error_X[m] | error_Y[m] | error_Phi[m]|
 1  | 10          |  0.10        |  0           |  0               |             |            |              |           |           |            |            |            |             |
 2  | 10          | -0.10        |  0           |  0               |             |            |              |           |           |            |            |            |             |
 3  | 10          |  0.20        |  0           |  0               |             |            |              |           |           |            |            |            |             |
 4  | 10          | -0.20        |  0           |  0               |             |            |              |           |           |            |            |            |             |
 5  | 10          |  0.0         |  0.10        |  0               |             |            |              |           |           |            |            |            |             |
 6  | 10          |  0.0         | -0.10        |  0               |             |            |              |           |           |            |            |            |             |
 7  | 10          |  0.0         |  0.20        |  0               |             |            |              |           |           |            |            |            |             |
 8  | 10          |  0.0         | -0.20        |  0               |             |            |              |           |           |            |            |            |             |
 9  | 10          |  0.0         |  0           |  0.3             |             |            |              |           |           |            |            |            |             |
 10 | 10          |  0.0         |  0           | -0.3             |             |            |              |           |           |            |            |            |             |
 11 | 10          |  0.0         |  0           |  0.6             |             |            |              |           |           |            |            |            |             |
 12 | 10          |  0.0         |  0           | -0.6             |             |            |              |           |           |            |            |            |             |
 13 | 20          |  0.10        |  0           |  0               |             |            |              |           |           |            |            |            |             |
 14 | 20          | -0.10        |  0           |  0               |             |            |              |           |           |            |            |            |             |
 15 | 20          |  0.20        |  0           |  0               |             |            |              |           |           |            |            |            |             |
 16 | 20          | -0.20        |  0           |  0               |             |            |              |           |           |            |            |            |             |
 17 | 20          |  0.0         |  0.10        |  0               |             |            |              |           |           |            |            |            |             |
 18 | 20          |  0.0         | -0.10        |  0               |             |            |              |           |           |            |            |            |             |
 19 | 20          |  0.0         |  0.20        |  0               |             |            |              |           |           |            |            |            |             |
 20 | 20          |  0.0         | -0.20        |  0               |             |            |              |           |           |            |            |            |             |
 21 | 20          |  0.0         |  0           |  0.3             |             |            |              |           |           |            |            |            |             |
 22 | 20          |  0.0         |  0           | -0.3             |             |            |              |           |           |            |            |            |             |
 23 | 20          |  0.0         |  0           |  0.6             |             |            |              |           |           |            |            |            |             |
 24 | 20          |  0.0         |  0           | -0.6             |             |            |              |           |           |            |            |            |             |
 25 | 30          |  0.10        |  0           |  0               |             |            |              |           |           |            |            |            |             |
 26 | 30          | -0.10        |  0           |  0               |             |            |              |           |           |            |            |            |             |
 27 | 30          |  0.20        |  0           |  0               |             |            |              |           |           |            |            |            |             |
 28 | 30          | -0.20        |  0           |  0               |             |            |              |           |           |            |            |            |             |
 29 | 30          |  0.0         |  0.10        |  0               |             |            |              |           |           |            |            |            |             |
 30 | 30          |  0.0         | -0.10        |  0               |             |            |              |           |           |            |            |            |             |
 31 | 30          |  0.0         |  0.20        |  0               |             |            |              |           |           |            |            |            |             |
 32 | 30          |  0.0         | -0.20        |  0               |             |            |              |           |           |            |            |            |             |
 33 | 30          |  0.0         |  0           |  0.3             |             |            |              |           |           |            |            |            |             |
 34 | 30          |  0.0         |  0           | -0.3             |             |            |              |           |           |            |            |            |             |
 35 | 30          |  0.0         |  0           |  0.6             |             |            |              |           |           |            |            |            |             |
 36 | 30          |  0.0         |  0           | -0.6             |             |            |              |           |           |            |            |            |             |
 37 | 10          |  0.10        |  0.10        |  0               |             |            |              |           |           |            |            |            |             |
 38 | 10          | -0.10        | -0.10        |  0               |             |            |              |           |           |            |            |            |             |
 39 | 10          |  0.20        |  0.20        |  0               |             |            |              |           |           |            |            |            |             |
 40 | 10          | -0.20        | -0.20        |  0               |             |            |              |           |           |            |            |            |             |

### Test 2: Driving shapes
In order to test combined movements when the wheels turn during the test, the robot will drive in a square, circle pattern and finally a slalom pattern.
Note again that the goal of driving in these shapes is not to end up at the start pose again but rather to compare odometry and real measurements. 

Shape             |Step   | Test parameters                                        | | | | Real values (measured by ar_track_alvar) | | | Measured/Odometry values       | | | Error (max 1% of real values)     | | |
------------------|-------|:------------------------------------------------------:|-|-|-|------------------------------------------|-|-|--------------------------------|-|-|-----------------------------------|-|-|
                  |       | duration[s] | speed_X[m/s] | speed_Y[m/s] | speed_Phi[rad/s] | delta_X[m]  | delta_Y[m] | delta_Phi[m] | odom_X[m] | odom_Y[m] | odom_Phi[m]| error_X[m] | error_Y[m] | error_Phi[m]|
Square 1m         | 1     | 10          |  0.05        |  0           |  0               |             |            |              |           |           |            |            |            |             |
                  | 2     | 5           |  0.0         |  0           |  0.314           |             |            |              |           |           |            |            |            |             |
                  | 3     | 10          |  0.05        |  0           |  0               |             |            |              |           |           |            |            |            |             |
                  | 4     | 5           |  0.0         |  0           |  0.314           |             |            |              |           |           |            |            |            |             |
                  | 5     | 10          |  0.05        |  0           |  0               |             |            |              |           |           |            |            |            |             |
                  | 6     | 5           |  0.0         |  0           |  0.314           |             |            |              |           |           |            |            |            |             |
                  | 7     | 10          |  0.05        |  0           |  0               |             |            |              |           |           |            |            |            |             |
                  | 8     | 5           |  0.0         |  0           |  0.314           |             |            |              |           |           |            |            |            |             |

Shape             |Step   | Test parameters                                        | | | | Real values (measured by ar_track_alvar) | | | Measured/Odometry values       | | | Error (max 1% of real values)     | | |
------------------|-------|:------------------------------------------------------:|-|-|-|------------------------------------------|-|-|--------------------------------|-|-|-----------------------------------|-|-|
                  |       | duration[s] | speed_X[m/s] | speed_Y[m/s] | speed_Phi[rad/s] | delta_X[m]  | delta_Y[m] | delta_Phi[m] | odom_X[m] | odom_Y[m] | odom_Phi[m]| error_X[m] | error_Y[m] | error_Phi[m]|
Square 2m         | 1     | 20          |  0.05        |  0           |  0               |             |            |              |           |           |            |            |            |             |
                  | 2     | 5           |  0.0         |  0           |  0.314           |             |            |              |           |           |            |            |            |             |
                  | 3     | 20          |  0.05        |  0           |  0               |             |            |              |           |           |            |            |            |             |
                  | 4     | 5           |  0.0         |  0           |  0.314           |             |            |              |           |           |            |            |            |             |
                  | 5     | 20          |  0.05        |  0           |  0               |             |            |              |           |           |            |            |            |             |
                  | 6     | 5           |  0.0         |  0           |  0.314           |             |            |              |           |           |            |            |            |             |
                  | 7     | 20          |  0.05        |  0           |  0               |             |            |              |           |           |            |            |            |             |
                  | 8     | 5           |  0.0         |  0           |  0.314           |             |            |              |           |           |            |            |            |             |

Driving in a circle requires to both turn and move forward at the same time. 
To drive a 0.75m radius circle with an angular speed of 0.4rad/s, the forward speed must be radius * angular speed = 0.75 * 0.4 = 0.3 [m/s]
The distance covered in a circle is 2 * pi * radius = 2 * pi * 0.75 = 4.71m.
With a speed of 0.3[m/s], we must keep driving for distance/speed = 4.71 / 0.3 = 15.7s

Shape             |Step   | Test parameters                                        | | | | Real values (measured by ar_track_alvar) | | | Measured/Odometry values       | | | Error (max 1% of real values)     | | |
------------------|-------|:------------------------------------------------------:|-|-|-|-------------------------------------|-|-|--------------------------------|-|-|-----------------------------------|-|-|
                  |       | duration[s] | speed_X[m/s] | speed_Y[m/s] | speed_Phi[rad/s] | delta_X[m]  | delta_Y[m] | delta_Phi[m] | odom_X[m] | odom_Y[m] | odom_Phi[m]| error_X[m] | error_Y[m] | error_Phi[m]|
Circle            | 1     | 15.7        |  0.3         |  0           |  0.4             |             |            |              |           |           |            |            |            |             |

The test below has the purpose to determine whether varying the steering angle while driving is of any influence on the accuracy. 

Shape             |Step   | Test parameters                                        | | | | Real values (measured by ar_track_alvar) | | | Measured/Odometry values       | | | Error (max 1% of real values)     | | |
------------------|-------|:------------------------------------------------------:|-|-|-|------------------------------------------|-|-|--------------------------------|-|-|-----------------------------------|-|-|
                  |       | duration[s] | speed_X[m/s] | speed_Y[m/s] | speed_Phi[rad/s] | delta_X[m]  | delta_Y[m] | delta_Phi[m] | odom_X[m] | odom_Y[m] | odom_Phi[m]| error_X[m] | error_Y[m] | error_Phi[m]|
Circle            | 1     | 3           |  0.3         |  0           |  0.4             |             |            |              |           |           |            |            |            |             |
                  | 2     | 3           |  0.3         |  0           | -0.4             |             |            |              |           |           |            |            |            |             |
                  | 3     | 3           |  0.3         |  0           |  0.4             |             |            |              |           |           |            |            |            |             |
                  | 4     | 3           |  0.3         |  0           | -0.4             |             |            |              |           |           |            |            |            |             |

Improvements
------------
* Possiblility for improvement
