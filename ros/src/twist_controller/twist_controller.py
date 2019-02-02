import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self,
        vehicle_mass ,
        fuel_capacity ,
        brake_deadband ,
        decel_limit ,
        accel_limit,
        wheel_radius ,
        wheel_base ,
        steer_ratio ,
        max_lat_accel ,
        max_steer_angle ):
        # TODO: Implement

        self.yaw_controller = YawController(wheel_base,steer_ratio,0.1,max_lat_accel,max_steer_angle)      


        kp = 0.3
        ki = 0.1
        kd = 0.0
        mn = 0.0  #min throttle
        mx = 0.2  #max throttle
        self.throttle_controller= PID(kp,ki,kd,mn,mx)   

        tau= 0.5
        ts = 0.02
        self.vel_lpf= LowPassFilter(tau,ts)

        self.vehicle_mass=vehicle_mass
        self.fuel_capacity=fuel_capacity
        self.brake_deadband=brake_deadband
        self.decel_limit=decel_limit
        self.accel_limit=accel_limit
        self.wheel_radius=wheel_radius

        self.last_time=rospy.get_time()

    def control(self, current_velocity,current_yaw_rate,desired_velocity,desired_yaw_rate,dbw_enabled): #*args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs

        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.0,0.0,0.0
        
        current_velocity = self.vel_lpf.filt(current_velocity)
        #rospy.logwarn("current_yaw_rate: {0}".format(current_yaw_rate))

        steer = self.yaw_controller.get_steering(desired_velocity,desired_yaw_rate, current_velocity)

        vel_error = desired_velocity - current_velocity
        self.last_vel= current_velocity

        current_time = rospy.get_time()
        sample_time= current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error,sample_time)
        brake = 0

        if desired_velocity == 0 and current_velocity < 0.1:
            throttle=0
            brake = 400
        elif throttle < .1 and vel_error <0:
            trottle = 0
            decel = max(vel_error,self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius




        #throttle=0.1 # between 0 and 1 
        #brake=0.0    #percent
        #steer= -2.0/15.0  # deg steering wheel ??? or whatever ...  
        # Return throttle, brake, steer
        #rospy.logwarn("current velocity: {0}".format(linear_vel:))
        return throttle, brake, steer