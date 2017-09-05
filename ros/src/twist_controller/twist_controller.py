
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Verify this commands
        min = kwargs.get('min_acc', 0.0)
        max = kwargs.get('max_acc', 0.0)
        max_steer = kwargs.get('max_steer_angle', 0.0)

        # create controllers
        self.throttle_pid = pid.PID(kp=1.0, ki=0.02, kd=0.0, min=min, max=max)
        self.steer_pid = pid.PID(kp=0.088, ki=0.003, kd=0.15, min=-max_steer, max=max_steer)

        # create lowpass filters
        self.throttle_filter = lowpass.LowPassFilter(tau=0.10, ts=0.90)
        self.steer_filter = lowpass.LowPassFilter(tau=0.00, ts=1.00)

        # init timestamp
        self.timestamp = rospy.get_time()
        pass

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        latest_timestamp = rospy.get_time()
        duration = latest_timestamp - self.timestamp
        sample_time = duration + 1e-6  # to avoid division by zero
        self.timestamp = latest_timestamp

        if dbw_enabled:
            # initialize values
            brake = 0.
            throttle = 0.
            # calculate new steering angle
            steering_angle = self.steer_pid.step(cte, sample_time)
            steering_angle = self.steer_filter.filt(steering_angle)
            # calculate throttle and brake
            throttle = self.throttle_pid.step(vel_error, sample_time)
            throttle = self.throttle_filter.filt(throttle)
            # convert to the expected format
            if throttle < 0.:
                brake = throttle
                throttle = 0.
            return throttle, brake, steering_angle
        # dbw is not enabled (manual override) so resetting pid's and filters
        self.throttle_pid.reset()
        self.steer_pid.reset()
        self.throttle_filter.last_val = 0.0
        self.steer_filter.last_val = 0.0
        return 0., 0., 0.
