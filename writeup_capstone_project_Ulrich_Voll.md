# **SDC Capstone Project (ROS)** 




**Ulrich Voll, voll@algorithmic-consultants.com**

-------------------

Notes to the reviewer: 

This is a single submission, simulator only. Stopping at traffic lights in the simulator using ground truth information, no detector/classifier.


Please refer to my github repo [https://github.com/uv10000/CarND-Capstone](https://github.com/uv10000/CarND-Capstone).


My solution is just enough to satisfy the  [project rubric](https://review.udacity.com/#!/rubrics/1969/view). Yet I would have liked to acchieved more, but there were severe time constraints due to my full time job and my two lovely little kids. 

For the same reason I decided not to join a team, thereby giving away the option to test the code on the real Carla vehicle.

Thank you for reviewing!


---------------

0.1 Overview

0.2 Set-Up

1 Code along the lines of the project walkthrough part 1 -> waypoints are published (Simulator) 

2 Code along the lines of the project walkthtrough part 2 -> vehicle follows waypoints (Simulator)

3 Code along the lines of the project walkthtrough part 3 -> vehicle stops correctly at traffic lights (Simulator, ground truth only) 

4 Conclusion, open threads




[//]: # (Image References)

[image1]: ./final-project-ros-graph-v2.png "Model Visualization"

---------------------

## 0.1 Overview 


![alt text][image1] 

I contributed code to the following node-stubs in the above ROS architecture/framework provided by Udacity, details see below. The vehicle drives around the test track, in simulator mode (only), stopping at red traffic lights using ground truth information. 

- Traffic Light Detection Node
  
      Timing: event based, upon new video frame
      
      Input: 
        /base_waypoints (static information)
        /image_color (video frames 600x800 RGB)
        /vehicle/traffic_lights (traffic light positions incl ground truth information)
        /current_pose
    
      Output: 
        /traffic_waypoint (location of next red traffic-light ahead) 
      
  I made use of ground truth information only, instead of a self-written detector/classifier, yet this is still non-trivial code.

- Waypoint Updater Node

 
      Timing: 50 Hz

      Input: 
        /base_waypoints (static)
        /traffic_waypoint (location of next red traffic light ahead) 
        /current_pose
    
      Output:
        /final_waypoints (waypoints sent toward low level control, not truly "final") 
      


- DBW Node (Drive By Wire)

 
      Timing: 50 Hz

      Input: 
      /current_velocity (twist, i.e. both linear and angular velocities)
      /vehicle/dbw_enabled (boolean, relevant only for real vehicle, stubbe to True) 
      /twist_cmd (desired velocities, twist, i.e. bot linear and angular)
    
      Output:
      /vehicle/steering_cmd
      /vehicle/throttle_cmd
      /vehicle/brake_cmd



---------

## 0.2 Set-Up  

I used a descendant of the  Ubuntu 16.04  VirtualBox VM we were provided in class. It has ROS kinetic installed. 

Upon cloning my github repo [https://github.com/uv10000/CarND-Capstone], please go to the ros subfolder and run

      catkin_make
      source /devel/setup.bash
      roslaunch launch/styx.launch

Port forwarding in the VM is configured as follows

      Rule 1  TCP   127.0.0.1  4567   4567  (host port/guest port)
      ssh     TCP   127.0.0.1  49852  22 

Deviating from the Readme.md, install requirements as follows 

    cd CarND-Capstone
    python -m pip install -r requirements.txt

We recomment to run rosdep. 

Please start the unity-simulator on the host machine as described in the Readme.md, do not forget to remove the checkmark "manual" and set the checkmark "camera". 

I did not manage to install the Cuda/Tensorflow environment on the VM without severe side effects on ROS functionality. Fortunately I was able to roll back my VM.







---





## 1 My code along the lines of the project walkthrough part 1 -> waypoints are published (Simulator) 

Main file: waypoint_updater.py, class WaypointUpdater, discussion below

Remark 1: In my understanding, and strictly speaking, the definition of a trajectory is *a sequence of   tuples of points in time and space (poses)*, obviously with time-values increasing along the sequence. In the ROS-Framework provided by Udacity the term "trajectory" seems to describe a sequence of tuples consisting of points in space (poses) and velocities (twists). This is not exactly the same but -- without having a formal proof -- those two things seems to be pretty much equivalent. 

Remark 2: The name "final_waypoints" is somewhat misleading. Those waypoints are handed over to another "black-box" node, called "Waypoint Follower", I think it essentially contains a pure-pursuit controller, yielding twist-shaped commands.  Those twist_commands are then input to the (our) "DBW Node", finally yielding throttle, brake and steering commands to the vehicle as discussed below. This is somewhat resembling a cascade-controller, with the PID like inner part implemented in (our) node "DBW Node".

Remark 3: I spent *far to much time* on the issue of the car stopping near the end of the first round. I made many additions inside my own code in order to correctly account for modulus-effects (due to the waypoints forming a closed loop). Onĺy very late I discovered the trivial fact that Python lists do not apply modulus operations automatically (as opposet to numpy arrays which  seem to do just this). I therefore actively padded the list of "final" waypoints sent towards the vehicle with the missing values. Now, in the simulator, I could see the previously missing waypoints leading onwards. Nevertheless the vehicle stopped in the same position near the end of the first round, ignoring the onward waypoints. I gave up on that - it almost seems as if the reason is inside the "black box" node "Waypoint Follower". 


### 1.1 \__init__(self)
Added subscribers for /current_pose, /base_waypoints, /traffic_waypoint

Added publisher /final_waypoints (remark: they are not final)

Helper variables (uninitialized, None)

Starting a loop, in order to acchieve a 50 Hz "sampling rate".

### 1.2 self.loop()

"Main loop"

Infinite while loop (ok until rospy.is_shutdown==True), every 20 ms (50 Hz)

Do nothing as long as helper variables are not yet initialized

Get id of closest waypoint 

      self.get_closest_waypoint_id()

Compute and publish LOOKAHEAD_WPS=200 waypoints based on new closest waypoint

    self.publish_waypoints(closest_waypoint_id)

do nothing until 20ms are over


### 1.2 self.get_closest_waypoint_id()

Determine id of closest waypoint ahead of current pose, resembling the ceil() command 

Using a KD-tree for efficiency, finding the id of the closest waypoint to current pose, as suggested in walk-through

Consider current pose, and the vector connecting the closest waypoint and its predecessor in order to find out if the closest waypoint is ahead of or behind the vehicle using elementary geometry (scalar products ...). 

If the closes waypoint it is indeed behind, takt the next waypoint. This resembles a ceil()-operator, rounding to the next largest discrete value. 


### 1.3 self.publish_waypoints(closest_waypoint_id)

Compute a "lane", a sequence of LOOKAHEAD_WPS=200 waypoints starting at closest_waypoint

    self.generate_lane()

Lane() is a datatype provided bp the ROS framework, sequence of waypoints, at least semantically. 

Cut out a contiguous subseqence of LOOKAHEAD_WPS=200 waypoints from the global waypoints.

We took into account modulus-considerations, but this was not sufficient to make the car continue at the end of the first lap. 

Essentially check if the stopline of the next red light is at least LOOKAHEAD_WPS=200 waypoints ahead.

If yes, just return the aforementioned subsequence, called "horizon_waypoints" in the code.

Otherwise compute and return a modified trajectory/lane using 

    self.decelerate_waypoints(horizon_waypoints,closest_idx)

### 1.4 self.decelerate_waypoints(horizon_waypoints,closest_idx)

For all waypoints in the "horizon_waypoints" reduce the velocities of the "horizon_waypoints" according to a monotonically decreasing function of the distance to the horizon of the respective waypoint. Using sqrt as suggested in walkthrough. 

### 1.5 Further Functions

Apart from the callback functions that are called upon incoming events, there are a few fairly self-explaing helper functions. Notably when the list of "base_waypoints" is first received an appropriate KD-tree helper variable is created in order to speed up finding closest waypoints later on.

----------------


## 2 My code along the lines of the project walkthtrough part 2 -> vehicle follows waypoints (Simulator)

Main file: dbw_node.py, class DBWNode, discussion below
Helper files: lowpass.py, pid.py, twist_controller.py, yaw_controller.py

Remark 4: Please refer to Remark 2 above. The DBWNode only contains part of the controller, comparable to the inner part of a cascade controller. The so called "final waypoints" from the waypoint_updater node are not direct input to the DBW "Node". They are first converted by the "black box" "Waypoint Follower". It seems that this is implementing a pure pursuit controller in c++.

Remark 5: In the real vehicle there will be a flag "dbw_enabled", this is never sent in the simulated vehicle where it effectively is stubbed to True by setting the corresponding helper variable to True, initially. In the real vehicle this is used to prevent the I-portion of PID controllers from winding up, during Drive by wire functionality may be disabled by the driver. 

### 2.1 \__init__(self)
Added subscribers for /current_velocity, /twist_cmd, /dbw_enabled

Added publishers /vehicle/steering_cmd,  /vehicle/brake_cmd, /vehicle/trottle_cmd

Helper variables (uninitialized, None), most of them for buffering the commands 

Again, starting a loop, in order to acchieve a 50 Hz "sampling rate"

### 2.2 self.loop()

"Main loop"

Infinite while loop (ok until rospy.is_shutdown==True), every 20 ms (50 Hz)

Do nothing as long as helper variables are not yet initialized

Call 

    throttle, brake, steer = 
      controller.control(
        current_velocity, 
        current_yaw_rate, 
        desired velocity, 
        current_velocity, 
        dbw_enabled
      )

Here "controller" is of type Controller() which is a wrapper class provided by Udacity, containing both longitudinal and lateral control. 

Publish the throttle, break and steer commands (using a helper-function "publish(...)")

do nothing until 20ms are over


### 2.3 Class Controller()

In the file twist_controller.py resides the controller converting the actual values and the desired values into control command for throttle, steering and brake. 

In order to accommodate the "dbw_enabled" flag we reset the throttle controller (with internal memory in its integral, anti-windup ...) and return 0.0 respectively for all three commands if "dbw_enabled" is False. 

### 2.3.1 Longitudinal Control

Note that there are two actuators, braking and throttle, but both are somewhat incomplete. Throttle can only lead to non-negative torques and braking can only lead to non-positive torques at the wheels. So we need to perform some arbitration between the two, see below.

The actual velocity is first filtered by a standard PT1 filter (tau=0.5, t_samp=0.02) provided by Udacity. 

Udacity also provided a template for a PID controller, which is used for longitudinal control only. Its main input is the error "desired velocity" minus "actual velocity", and it provides a throttle value, that may be modified further down in the code when taking care of the braking value. 

The PID controller accepts adaptive sampling times, to accomadate for jitter in the 50 Hz messages' arrival. We gratefully adopted the parameters suggested in the project walk-trough. 

After a (preliminary) trottle value has been determined, a suitable brake value is derived from it as follows. The throttle value may also get adapted in the process:

a) Unless otherwise specified, brake=0.0 and throttle is left untouched, as it came from the PID controller.

b) HOLD. If desired velocity is zero and actual velocity is less than the minimum value 0.1 (this effectively means actual velocity is also zero, since negative velocities should never arise), throttle is set to 0.0 and brake is set to a value to hold the car (400 Nm, as suggested in the walk-trough). This is to compensate the thrust by the automatic transmission when idling at zero throttle.  

c) DECELERATE. If throttle less than 10% and actual velocity exceeds desired velocity engage the brake as follows: 
- set trottle to 0.0
- determine a desired deceleration monotonically increasing in the velocity error but saturated by "decel_limit". A P-controller with saturation, effectively. 
- set the brake torque to a value corresponding to, i.e. proportional to this deceleration. 

Remark 6: In my eyes it there is no need to compute the torque required using a (however simple) physics modelling, since there is an arbitrary factor between velocity error and torqe involved anyhow. The arbitrary factor (of the P-controller) just happens to be 1.0 in the suggestion made during the project walk-through. Most numbers are equal to 42, up to a factor.

### 2.3.1 Lateral Control

Remember that this node contains only parts of the controls converting waypoints into command to the vehicles' actuators. 

For lateral control we are given a desired yaw-rate. 

Gratefully following Udacity's suggestion I used their yaw-controller. 

Input is actual velocity and desired yaw rate, but (surprisingl) not the actual yaw rate. 

When looking at the code it seems the Udacity yaw controller is open-loop, or "feed-forward", w.r.t. lateral control  in the following sense:
Given actual velocity and desired yaw rate, solve a simple kinematic bicycle model for the steering angle. 

This is a simple look-up-table style conversion of the desired yaw rate into a steering angle, depending on vehicle speed and a couple of vehicle parameters.

Strictly speaking this mechanism is not 100% open loop as it depends on the actual (measured) velocity, but it does not consume the actual yaw rate, as I had originally expected.  

### 2.4 Further Functions

Apart from the aforementioned helper classes/functions there are the necessary callback-functions for the subscribed topics.





-----------
## 3 My code along the lines of the project walkthtrough part 3 -> vehicle stops correctly at traffic lights (Simulator, ground truth only) 

Main file: tl_detector.py, class TLDetector()

Remark 7:My code makes use the ground truth information about traffic light state in the /vehicle/traffic_lights topic which is present in the simulator only, instead of a CNN-detector. Still this is non-trivial code and makes the vehicle stop at red traffic lights in the simulator. 


### 2.1 \__init__(self)

Added subscribers for /current_pose, /base_waypoints, /vehicle/traffic_lights /image_color

Added publisher /traffic_waypoint

Helper variables (uninitialized, None)

Do *not* start a loop, instead use rospy.spin(). There is no main loop, methods will be trigered event-based, e.g. upon receiving a new video frame.


### 2.2 self.loop()

"Main loop"

As mentioned above there is no need for a main loop here, as things are triggered event-based. This corresponding code-stub is obsolete.

-----------------
## 4 Conclusion, open threads


