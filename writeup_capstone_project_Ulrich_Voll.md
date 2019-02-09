# **SDC Capstone Project (ROS)** 




**Ulrich Voll, voll@algorithmic-consultants.com**

-------------------

Notes to the reviewer: 

This is a single submission, simulator only. Stopping at traffic lights in the simulator using ground truth information, no detector/classifier.


Please refer to my github repo [https://github.com/uv10000/CarND-Capstone](https://github.com/uv10000/CarND-Capstone).

The goal of this project according to the [rubric points](https://review.udacity.com/#!/rubrics/1969/view) is "The submitted code must work successfully to navigate Carla around the test track".

This it does but I would have liked to acchieved more ... severe time constraints due to my full time job and my two lovely little kids.

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

#### 0.1 Overview 


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

#### 0.2 Set-Up  

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





#### 1 Code along the lines of the project walkthrough part 1 -> waypoints are published (Simulator) 

Main file: waypoint_updater.py, class WaypointUpdater


### 1.1 \__init__(self)
Added subscribers for /current_pose, /base_waypoints, /traffic_waypoint

Added publisher /final_waypoints (remark: they are not final)

Helper variables (uninitialized, None)

Starting a loop, in order to acchieve a 50 Hz "sampling rate".

### 1.2 loop(self)

Infinite while loop (ok until rospy.is_shutdown==True)

Do nothing if helper variables are not yet initialized. 

### 1.2 get_closes_waypoint_id(self)

Determine id of closest waypoint ahead (similar to the ceil() command). 

Using a KD-tree for efficiency, finding the id of the closest waypoint, as suggested in walk-through. 

Consider current pose, and the vector connecting the closest waypoint and its predecessor in order to find out if the closest waypoint is ahead of or behind the vehicle using elementary geometry (scalar products ...). 

If the closes waypoint it is indeed behind, takt the next waypoint. This resembles a ceil()-operator, rounding to the next largest discrete value. 




#### 2 Code along the lines of the project walkthtrough part 2 -> vehicle follows waypoints (Simulator)

#### 3 Code along the lines of the project walkthtrough part 3 -> vehicle stops correctly at traffic lights (Simulator, ground truth only) 

#### 4 Conclusion, open threads


----------------

#### 1. Your code should compile.

 
Please refer to my githup repo [P8](https://github.com/uv10000/P8), cmake and make should work in the standard way.

I checked with a clean clone, seems to work fine on my local setup. 


-----

#### 2. The PID procedure follows what was taught in the lessons.
See lines 36 ff of PID.cpp
```
 p_error =  cte;
    i_error +=  cte;
    double antiwindup=100.0;  // another anti-windup for limiting I-Integral 
    i_error= fmax(-antiwindup,i_error); i_error=fmin(antiwindup,i_error); 
    // saturate to "antiwindup"
    d_error = cte -prev_cte;
    sum_of_squared_errors = cte*cte + sum_of_squared_errors*0.95; // moving average!
    prev_cte=cte;
```
and line 186 of main.cpp
```
 steer_value = -pid.p_error * pid.Kp  - pid.i_error * pid.Ki - pid.d_error * pid.Kd;
```

Note that I included an anti-windup mechanism for the I error integral and a moving averaging mechanism to prevent the value of the mean squared errors from winding up. 

---------------

#### 3. Describe the effect each of the P, I, D components had in your implementation.

In theory the P part should be for following the road, the D part should prevent overshooting and damping if the P part is chosen aggressively (which may be necessary in order to stay away from curbs ...). And the I part is for stationary accuracy.

However in practice this does not tell the whole story because the overall system consists of the plant (vehicle with limitations, inertia, etc.) plus the controller. 
In simple (linear, time invariant, no saturation or finite actuator velocities, etc.) cases and for simple controllers (P-controller) it is possible to find the right controller parameters algebraically using Laplace transforms. 

Speaking from my personal experience, I find this (i.e. the role of classical linear control theory, to be honest) somewhat overrated in view of many practical applications that are neither linear nor time invariant. All those shiny Laplace transform methods are no longer (strictly) applicable as soon as the slightest non-linearity or time dependeny creeps in.  What is your opinion on this? 

I totally agree with you that parameter tuning by optimization like e.g. by using Twiddle is a far more broadly applicable method, which works well in practice.  

Btw. it is a pity that you removed the  Model Predictive Control module from the course.
Which is - as far as I understand - a method for online optimisation of controller parameters.  

---------------------------

#### 4 Describe how the final hyperparameters were chosen.

Before  I will sketch my incomplete ideas and code fragments for an online version of Twiddle for autotuning below, here comes my "strategy" for manual tuning:

a) Start with I and D parts set to zero

b) Turn up the P value until the system gets instable.

c) Compensate for instability by turning  up the D part, and/or reducing P. Return to b) and iterate

d) Slowly turn up the I part to improve static accuracy, possibly return to b) and reiterate.  

(Plus spend hours of twiddling around by hand in a less than systematic way.)


Here comes the idea for performing twiddle online, that is at-run-time:

In an online setting we cannot restart the system at random during optimisatin, as we did in the quiz.

Idea: 

- State machine with 8 states arranged in a circle. 

- Move from state to state in cylic order.

- In each state certain parameter changes are performed "in the twiddle spirit", details see below. 

- Each time we leave a state the moving-average mean squared error counter is reset. 

- Inbetween states wait for several (e.g. 500) iterations of the simulator for the respective parameter changes to take effect, i.e. until the  moving-average mean squared error is close to it's steady state value. 

- During one round in the circle we collect enough information to perform (discrete) partial derivatives of the cost function, evaluated at the original set of parameter.

- In the final state of the cycle the parameters are updated according to twiddle. 

Remark: Twiddle is a generalization of gradient descent, with variable adaptive step size, isn't it? 

See the following (unfinished) code snippet for further details: 

```
          // this is placed in the main loop i.e. in the body of h.onMessage
          counter=(counter+1)%500;  // wait 500 steps before moving to the next state
          if (counter==0) {  // move to the next state
            double err = pid.TotalError();  // should have converged during 500 steps
            std::cout << "TotalError: " << err  << std::flush << std::endl;
            while (dp[0]+dp[1]+dp[2] >0.00){
              if(myindex ==0) {  // state-machine with 8 states 
                old_err=err; //remember error at (p1,p2,p3)
                p[0] = p[0] + dp[0]; // prepare for next step
                myindex = (myindex +1)%8; // jump to the next state after 500 steps
              }
              else if(myindex ==1) {
                err_plus[0]=err;//compute error at (p1+dp1,p2,p3)
                p[0] = p[0] - dp[0]; // go back
                p[1] = p[1] + dp[1]; // prepare for next step
                myindex = (myindex +1)%8; // jump to the next state after 500 steps
              }
              else if(myindex ==2) {
                err_plus[1]=err; //compute error at (p1,p2+dp2,p3)
                p[1] = p[1] - dp[1]; // go back
                p[2] = p[2] + dp[2]; // prepare for next step
                myindex = (myindex +1)%8; // jump to the next state after 500 steps
              }
              else if(myindex ==3) {
                err_plus[2]=err; //compute error at (p1,p2,p3+dp3)
                p[2] = p[2] - dp[2]; // go back
                p[0] = p[0] - dp[0]; // prepare for next step
                myindex = (myindex +1)%8; // jump to the next state after 500 steps
              }
              else if(myindex ==4) {
                err_minus[0]=err; //compute error at (p1-dp1,p2,p3)
                p[0] = p[0] + dp[0]; // go back
                p[1] = p[1] - dp[1]; // prepare for next step
                myindex = (myindex +1)%8; // jump to the next state after 500 steps
              }
              else if(myindex ==5) {
                err_minus[1]=err; //compute error at (p1,p2-dp2,p3)
                p[1] = p[1] + dp[1]; // go back
                p[2] = p[2] - dp[2]; // prepare for next step
                myindex = (myindex +1)%8; // jump to the next state after 500 steps
              }
              else if(myindex ==6) {
                err_minus[2]=err;  //compute error at (p1,p2,p3-dp3)
                p[2] = p[2] + dp[2]; // go back
                myindex = (myindex +1)%8; // jump to the next state after 500 steps
              }
              else if(myindex ==7) {
                // adapt (p1,p2,p3) and (dp1,dp2,dp3) accordingly
                // three cases:
                // a) right value smaller than middle value -> increase pi, increase dpi
                // b) left value smaller than middle value -> decrease pi, increase dpi
                // c) both values larger than middle value -> keep pi, decrease dpi
                for(int i=0;i<3;i++){
                  if(err_plus[i] < old_err*0.99){
                    p[i]+=dp[i]; dp[i]*=1.1;best_err =err_plus[i];
                  }
                  else if(err_minus[i] < old_err*0.99){
                    p[i]-=dp[i]; dp[i]*=1.1; best_err =err_minus[i];
                  }
                  else{
                    dp[i]*=0.95;
                  }
                  pid.Kp=p[0];pid.Ki=p[1];pid.Kd=p[2];
                }
                myindex = (myindex +1)%8;  // jump to the next state after 500 steps
              }  
            }
            pid.sum_of_squared_errors =0.0; //start afresh, reset error counter
          }
```

------------------
#### 5 The vehicle must successfully drive a lap around the track.


It does, but the driving style is rather jerky. See above.

I found it hard to find a set of parameters that simultaneously leads to a "smooth" driving style while staying close to the middle of the road, in particular away from the curbs. 


------------------
#### 6 Further improvements.

- Make the online optimisation (Twiddle) work. I have run out of time but I am confident that the above idea could work out. 

- alternatively find a way for offline optimization (eg recording an interesting part of the track). I expect this to be a lot of fiddly work though.

- Introduce an element of "looking ahead" of some kind, e.g. by using the CNN from the "Behavioural Cloning" project. Or just by providing an "unfair and artificial ad-hoc-hint" derived from our knowledge of the track, justified by the fact that a human driver will be able to look ahead.

- Model Predictive Control ... 
