
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        pass

    def control(self, current_velocity,current_yaw_rate,desired_velocity,desired_yaw_rate,dbw_enabled): #*args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        throttle=0.1 # between 0 and 1 
        brake=0.0    #percent
        steer= -2.0/15.0  # deg steering wheel ??? or whatever ...  
        # Return throttle, brake, steer
        return throttle, brake, steer