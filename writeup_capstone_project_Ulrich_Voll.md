# **SDC Capstone Project (ROS)** 




**Ulrich Voll, voll@algorithmic-consultants.com**

-------------------

Notes to the reviewer: 

This is a single submission, simulator only. Stopping at traffic lights in the simulator using ground truth information, no detector/classifier, can go on for a large number of successive rounds in the simulator.


Please refer to my github repo [https://github.com/uv10000/CarND-Capstone](https://github.com/uv10000/CarND-Capstone).


My solution is just enough to satisfy the  [project rubric](https://review.udacity.com/#!/rubrics/1969/view). Yet I would have liked to acchieved more, but there were severe time constraints due to my full time job and my two lovely little kids. 

For the same reason I decided not to join a team, thereby giving away the option to test the code on the real Carla vehicle.

Thank you for reviewing!

---------------

0.1 Overview

0.2 Set-Up

1 My code related to walkthrough part 1 -> waypoints are published (Simulator) 

2 My code related to walkthrough part part 2 -> vehicle follows waypoints (Simulator)

3 My code related to walkthrough part 3 -> vehicle stops correctly at traffic lights (Simulator, ground truth only) 

4 Conclusion, open threads




[//]: # (Image References)

[image1]: ./final-project-ros-graph-v2.png "Model Visualization"

---------------------
---

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

I had to to run rosdep at some stage. 

Please start the unity-simulator on the host machine as described in the Readme.md, do not forget to remove the checkmark "manual" and set the checkmark "camera". 

I did not, as of Feb 10 2019, manage to install the Cuda/Tensorflow environment on my VM without severe side effects on ROS functionality. Fortunately I was able to roll back my VM. 

My submission does not include a traffic sign detector but resorts to ground truth, available in the simulator only.







---
---





## 1 My code related to walkthrough part 1 -> waypoints are published (Simulator) 

Main file: waypoint_updater.py, class WaypointUpdater, discussion below

Remark 1: In my understanding, and strictly speaking, the definition of a trajectory is *a sequence of   tuples of points in time and space (poses)*, obviously with time-values increasing along the sequence. In the ROS-Framework provided by Udacity the term "trajectory" seems to describe a sequence of tuples consisting of points in space (poses) and velocities (twists). This is not exactly the same but -- without having a formal proof -- those two things seems to be pretty much equivalent. 

Remark 2: The name "final_waypoints" is somewhat misleading. Those waypoints are first handed over to another "black-box" node, called "Waypoint Follower". From looking at the sources I can tell that is essentially contains a pure-pursuit controller, yielding twist-shaped commands. I think that this "hidden part", written in C++, contains the substantial part of the "low level controller", converting waypoints into commands to the vehicle's actuators. Those twist_commands are then input to the (our) "DBW Node", finally yielding throttle, brake and steering commands to the vehicle as discussed below. This is somewhat resembling a cascade-controller, with the PID like inner part implemented in (our) node "DBW Node".

Remark 3: I spent *far to much time* on the issue of the car stopping near the end of the first round. I made many additions inside my own code in order to correctly account for modulus-effects (due to the waypoints forming a closed loop). Only very late I realised the basic fact that Python lists do not apply modulus operations automatically (as opposed to numpy arrays which  seem to do just this). I therefore actively padded the list of "final" waypoints sent towards the vehicle with the missing values. Now, in the simulator, I could see the previously missing waypoints leading onwards. Nevertheless, the vehicle stopped in the same position near the end of the first round, ignoring the onward waypoints. I gave up on that - it almost seems as if the reason is inside the "black box" node "Waypoint Follower".

LAST MINUTE ADDENDUM TO REMARK 3: I found the bug/reason for the problem! It was in the node "Waypoint Loader", the desired velocities in the final waypoints were deliberately decelerated in the code provided by Udacity. Remove this, and the car will go on forever. Possibly this is only a necessary condition, I can not tell as I modified the code in a number of places in order to accommodate issues related to modulus correctly. I do not know how many of those adaptations were strictly necessary. At least none of them seems to be wrong. Well nobody told me to focus exclusively on the nodes discussed in the walkthrough. This is quite literally a case of "thinking out of the box" -- therefore I Can't be angry with you guys at Udacity, but it did cost me a lot of time! 

---
### 1.1 \__init__(self)
Added subscribers for /current_pose, /base_waypoints, /traffic_waypoint

Added publisher /final_waypoints (remark: they are not final)

Helper variables (uninitialized, None)

Starting a loop, in order to achieve a 50 Hz "sampling rate".

---
### 1.2 self.loop()

"Main loop"

"Infinite" while loop (ok until rospy.is_shutdown==True), every 20 ms (50 Hz)

Do nothing as long as helper variables are not yet initialized

Get id of closest waypoint 

      self.get_closest_waypoint_id()

Compute and publish LOOKAHEAD_WPS=200 waypoints based on new closest waypoint

    self.publish_waypoints(closest_waypoint_id)

Thereafter do nothing until 20ms are over

---
### 1.2 self.get_closest_waypoint_id()

Determine id of closest waypoint ahead of current pose, resembling the ceil() command 

Using a KD-tree for efficiency, finding the id of the closest waypoint to current pose, as suggested in walk-through

Consider current pose, and the vector connecting the closest waypoint and its predecessor in order to find out if the closest waypoint is ahead of or behind the vehicle using elementary geometry (scalar products ...). 

If the closes waypoint it is indeed behind, take the next waypoint. This resembles a ceil()-operator, rounding to the next largest discrete value. 

---
### 1.3 self.publish_waypoints(closest_waypoint_id)

Compute a "lane", a sequence of LOOKAHEAD_WPS=200 waypoints starting at closest_waypoint

    self.generate_lane()

Lane() is a datatype provided bp the ROS framework, sequence of waypoints, at least semantically. 

Cut out a contiguous sub-sequence of LOOKAHEAD_WPS=200 waypoints from the global waypoints, just ahead of the car.

I took into account modulus-considerations, but this was not sufficient to make the car continue at the end of the first lap. ADDENDUM: I found the reason and fixed it. In the node "Waypoint Loader" the desired speeds for the final waypoints were deliberately decelerated, which I removed.

Essentially check if the stopline of the next red light is at least LOOKAHEAD_WPS=200 waypoints ahead.

If yes, just return the aforementioned sub-sequence, called "horizon_waypoints" in the code.

Otherwise compute and return a modified trajectory/lane using 

    self.decelerate_waypoints(horizon_waypoints,closest_idx)

---
### 1.4 self.decelerate_waypoints(horizon_waypoints,closest_idx)

For all waypoints in the "horizon_waypoints" reduce the velocities of the "horizon_waypoints" according to a monotonically decreasing function of the distance to the horizon of the respective waypoint (LOOKAHEAD_WPS=200 waypoints ahead of the car). 

I am using sqrt as a monotonically decreasing function as suggested in the walkthrough. 

Note that the code never modifies the original waypoints, but merely "patches" the velocities of the "horizon waypoints" (= a copy of a contiguous stretch of LOOKAHEAD_WPS=200 waypoints ahead of the car). 

---
### 1.5 Further Functions

Apart from the inevitable callback functions that are called upon incoming events, there are a few fairly self-explanatory helper functions. Notably when the list of "base_waypoints" is first received an appropriate KD-tree helper variable is created in order to speed up finding closest waypoints later on.

----------------
---


## 2 My code related to walkthrough part 2 -> vehicle follows waypoints (Simulator)

Main file: dbw_node.py, class DBWNode, discussion below
Helper files: lowpass.py, pid.py, twist_controller.py, yaw_controller.py

Remark 4: Please refer to Remark 2 above. The DBWNode only contains part of the "low level controller", comparable to the inner part of a cascade controller. The so called "final waypoints" from the waypoint_updater node are not direct input to the DBW "Node". They are first converted by the "black box" "Waypoint Follower". It seems that this is implementing a pure pursuit controller in C++.

Remark 5: In the real vehicle there will be a flag "dbw_enabled", this is never sent in the simulated vehicle where it effectively is stubbed to True by setting the corresponding helper variable to True, initially. In the real vehicle this is used to prevent the I-portion of PID controllers from winding up, while Drive-By-WTHereire functionality may be disabled by the driver. 

---
### 2.1 \__init__(self)
Added subscribers for /current_velocity, /twist_cmd, /dbw_enabled

Added publishers /vehicle/steering_cmd,  /vehicle/brake_cmd, /vehicle/trottle_cmd

Helper variables (uninitialized, None), most of them for buffering the commands 

Again, starting a loop, in order to achieve a 50 Hz "sampling rate"

---
### 2.2 self.loop()

"Main loop"

"Infinite" while loop (ok until rospy.is_shutdown==True), every 20 ms (50 Hz)

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

Thereafter do nothing until 20ms are over

---
### 2.3 Class Controller()

In the file twist_controller.py resides the controller class provided by Udacity converting the actual values and the desired velocity values into control command for throttle, steering and brake. 

In order to accommodate the "dbw_enabled" flag we reset the throttle controller (with internal memory in its integral, anti-windup ...) and return 0.0 respectively for all three commands whenever "dbw_enabled" is False. 

---
### 2.3.1 Longitudinal Control

Note that there are two actuators, braking and throttle, but both are somewhat incomplete if considered on their own. Throttle can only lead to non-negative torques and braking can only lead to non-positive torques at the wheels. So we need to perform some arbitration between the two, for details see below.

The actual velocity is first filtered by a standard PT1 filter (tau=0.5, t_samp=0.02) provided by Udacity. 

Udacity also provided a template for a PID controller, which is used for longitudinal control (only!). Its main input is the error "desired velocity" minus "actual velocity", and it provides a throttle value, that may be modified further down in the code when taking care of the braking value. 

The PID controller accepts adaptive sampling times, to accommodate for jitter in the 50 Hz messages' arrival. We gratefully adopted the parameters suggested in the project walk-trough. 

After a (preliminary) throttle value has been determined, a suitable brake value is derived from it as follows. The throttle value may also get adapted in the process:

a) NORMAL DRIVING. Unless otherwise specified, brake=0.0 and throttle is left untouched, as it came from the PID controller.

b) HOLD/STANDSTILL. If "desired velocity" is zero and "actual velocity" is less than the minimum value 0.1 (this effectively means that "actual velocity" is also zero, since negative velocities should never arise), throttle is set to 0.0 and brake is set to an appropriate value to hold the car (400 Nm, as suggested in the walk-trough, and watch the sign!). This is to compensate the thrust by the automatic transmission when idling at zero throttle.  

c) DECELERATE. If throttle is less than 10% and actual velocity exceeds desired velocity engage the brake as follows: 
- set throttle to 0.0
- determine a desired deceleration monotonically increasing in the velocity error but saturated by "decel_limit". A P-controller with saturation, effectively. 
- set the brake torque to a value corresponding to, i.e. proportional to this deceleration. 

Remark 6: In my eyes there is no need to compute the torque required using a (however simple) physics modelling, since there is an arbitrary "P" factor between velocity error and braking torque involved, anyhow. The arbitrary factor (of the P-controller) just happens to be 1.0 in the suggestion made during the project walk-through. Most numbers are equal to 42, up to a factor, not so sure about 0 and i ...



---
### 2.3.1 Lateral Control

Remember that this DBW node contains only parts of the low level controls converting waypoints into command to the vehicles' actuators. 

For lateral control we are given a desired yaw-rate. 

Gratefully following Udacity's suggestions, I used their yaw-controller. 

Input is actual velocity and desired yaw rate, but (surprisingly to me) not the actual yaw rate. 

Explanation: 

Inspecting the code reveals that  the Udacity yaw controller is open-loop, or "feed-forward", w.r.t. lateral control  in the following sense:
Given actual velocity, a couple of vehicle parameters and the desired yaw rate, solve a simple kinematic bicycle model for the steering angle. 

This is a simple look-up-table style conversion of the desired yaw rate into a steering angle, depending on vehicle speed and a couple of vehicle parameters.  

Strictly speaking this mechanism is only a 100% open loop if the longitudinal velocity remains constant, as it depends on the actual (measured) velocity, but it does indeed not consume the actual yaw rate, as I had originally expected.  

However note the suggestions in the walkthrough to dampen the steering command depending on the yaw rate error (which would introduce a dependency on the desired yaw rate, leading to closed loop behaviour).

---
### 2.4 Further Functions

Apart from the aforementioned helper classes/functions there are the necessary callback-functions for the subscribed topics.




---
-----------
## 3 My code related to walkthrough part 3 -> vehicle stops correctly at traffic lights (Simulator, ground truth only) 

Main file: tl_detector.py, class TLDetector()

Remark 7: My code makes use the ground truth information about traffic light state in the "/vehicle/traffic_lights" topic which is present in the simulator only, instead of a CNN-detector. Still there is non-trivial code and makes the vehicle stop at red traffic lights in the simulator. 

The only thing that is missing is a detector/classifier.

---
### 3.1 \__init__(self)

Added subscribers for /current_pose, /base_waypoints, /vehicle/traffic_lights, /image_color

Added publisher /traffic_waypoint

Helper variables (uninitialized, None)

In this node there are similar mechanisms in place as in the waypoint_updater (KD-tree, ....) but those two nodes can not share common variables as they run in different unix-processes. 

Do *not* start a loop, instead use rospy.spin(). 

There is no need for a main loop, functions/methods will be triggered event-based, e.g. upon receiving a new video frame.

---
### 3.2 self.loop()

"Main loop"

As mentioned above there is no need for a main loop in this node, as everything is triggered event-based. This corresponding code-stub is obsolete, I retained it with an explanatory comment. 

---
### 3.2 self.image_cb()

As mentioned before in this node there is no main loop, everything is happening event-based. 

This callback function, called each time a new image arrives, is explicitly stated as it performs, or at least triggers the bulk  of the work. 

It publishes the index (Uint32) of the next red traffic light ahead in the topic /traffic_waypoint. If there is no red traffic light ahead it sends out "-1".

Most work is delegated to the function self.process_traffic_lights() discussed below. It returns "light_wp" and "state" of the next traffic light ahead. 

These values trigger stateful behaviour as follows.

At each new event (video frame) do:

- if: Traffic light colour changes.
-> adapt internal state accordingly and reset a debounce-counter "self.state_count" to zero. 

- elif: Debounce counter exceeds threshold.
-> New signal is clearly confirmed. If new state = RED, remember "light wp" as "last wp", output "light_wp", otherwise "-1"   

- else: No color change but not confirmed/debounced yet. -> Output "last_wp" for the time being.


- increment the debounce-counter (in all cases)

---
### 3.3 self.process_traffic_lights()

This function computes the index of the closest traffic light ahead and its state (RED, GREEN, etc).

a) In order to identify the closest traffic light, it employs a static list of all stop_line_positions (a member variable "self.lights") and the index closest to the car using self.get_closest_waypoint_id(). 

Then it iterates over all all traffic lights, for each do:

- compute respective stop-line position

- get index of closest waypoint to the respective stop-line position 

- compute the difference between respective stop-light index and the car's index. 

- if the distance is smaller than all previously found distances, select the respective stop, and adapt best distance found so far.

b) Determine the state of the closest traffic light found using self.get_light_state()

Finally return state and index of the closest light.

It is feasible that information is missing, e.g. because no red lights have been found, or because the classifier/detector is not sure, in these cases return "-1" and/or "TrafficLight.UNKNOWN".



---
### 3.4 self.get_closest_waypoint_id()

Exactly the same as 1.2 above. 

The respective ROS-nodes could share the code between them using a library but then the source code for the respective nodes would no longer be decoupled. 

There is a trade-off to be made between the benefits of code-reuse vs keeping things decoupled. I tend to prefer the latter over the sooner. 

---
### 3.5 self.get_light_state()

A helper function for self.process_traffic_lights(light)

Here is where the ground truth information is used that is not available in the real car. 

Essentially *in the simulator* the class lights has an entry lights.state, in "cheat mode" this is returned. 

See the code snippet below, final line: In normal mode we would have to call the (unfortunately non-existing) 

    self.light_classifier.get_classification(cv_image)

I also prepared some code for debugging and for collecting training data in appropriate numpy arrays. This is unfinished work. In particular I do not think this is the right place for collecting the y-values for training. 

      cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        
        if False: # debugging 
            height,width, channels= cv_image.shape
            rospy.logwarn("height: {0}".format(height))
            rospy.logwarn("width: {0}".format(width))
            rospy.logwarn("channels: {0}".format(channels))

        cheat_mode = True
        if cheat_mode:
            # create training data
            
            if True: # generate learning data X and Y when True
                self.X_list.append(cv_image)
                self.Y_list.append(light.state)
                self.count_samples+= 1
                if self.waypoints:
                    if self.count_samples == len(self.waypoints.waypoints) -10: #10000:
                        X=np.array(self.X_list)
                        Y=np.array(self.Y_list)
                        rospy.logwarn("X.shape(): {0}".format(X.shape))
                        rospy.logwarn("Y.shape(): {0}".format(Y.shape))


            return light.state  ## this ist just for testing, "cheat mode"
            ################# IGNORE ALL BELOW FOR THE TIME BEING
    

        #Get classification
        return self.light_classifier.get_classification(cv_image)  ### we  override this in "cheat mode" for the time being





---
-----------------
## 4 Conclusion, open threads

This was a great but also very challenging and time-consuming project. 

I did not find the time to program a classifier/detector, but relied on ground truth information instead (available only in the simulator).

In any case I learned a lot about how to operate a vehicle using ROS. Very useful!

Let me discuss what is missing and how I could go about collecting data, setting up and train a CNN for detection/classification.

### 4.1 Setting up the Infrastructure
I did not manage to install Tensorflow along with my ROS installation on my VM. This is trivial but it needs to be done and may be time-consuming. 

Alternatively I could move to the Udacity project workspace. There everything should be installed.     

### 4.1 Collecting Training Data

As described in 3.5 I started with collecting images (x-values) and traffic light states (y-values) for training in the code. 

My plan was to write those vectors out in the same format as used for the project on traffic sign classification. 

However, the images are known in the node "TL Detector Node" while the correct classification of the traffic light state is available in another node, namely in "Waypoint Updater Node".  Thus it will not be an easy task to store the "x-stream" and the "y-stream" synchronously. 

I suspect that it would be better to write things out to ROS and somehow extract x-values and y-values from a recording (I suppose from a ROS bag). 

But I do not yet know how to go about it. However, the latter method would seem to be also  applicable for real car measurements, using true video images. 


### 4.1 Detection vs. Classification   

As a minimal though sub-optimal solution, I thought about adapting the classifier for traffic signs from the respective project in Term 1. Slightly  modify the CNN in order to classify for traffic lights rather than traffic signs. 

Even if I had the time to get this to work -- there is a severe drawback to using a classifier (as opposed to using a detector):

- There are several traffic lights in an image, some of them do not even apply for the direction taken by the car. 

- Traffic lights are tiny

- There may be a tendency to overfit, learning the pictures "by heart", rather than semantically parsing for traffic signs.

Suggested solution 1: Classical image processing followed by a classifier.
Use geometry, focal length, positions of Traffic lights in 3d space and the camera to cut out the traffic lights. Once we have such "zoomed in" on the relevant traffic light, a classifier might have a real chance. However this sounds like a lot of work and it would not lend itself to generalisation. 

Suggested solution 2:
Devise a Traffic light detector, that is finding bounding boxes around (releveant) traffic lights plus a classification of for each respective traffic light found. 

I think that solution 2 would be the way forward but it would be even more work, as I could not simply reuse the TensorFlow code from the Traffic Sign Classifier project.

Blocking points/TODOs:

- Toolchain on the VM, alternatively get accustomed to the Udacity workspace
- Find a good way to record training data (from a ROS bag?!)
- When automatic generation of y-values (bounding boxes?!) is not possible: Find a tool for labeling video images. Learn how to operate it. Do it. 
- Devise a CNN using Tensorflow 1.3 to perform traffic light detection
- Train it using the training data and encode it in an appropriate file format.
- Deploy/integrate it to the ROS-Framework

In view of this long list I decided to stop here and submit the project as is, even though in principle I could try to make use of the 4 week extension. I think the project in the present stage satisfies the project rubric. 

However, the above TODOs are all elementary and important skills and I hope to be able to complete the project in the future. 

Thanks to you Udacity people, this was a great course. 









