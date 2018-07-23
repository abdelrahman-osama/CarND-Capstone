import rospy                                    # will need access to parameters and system time
from pid import PID                             # PID controller provided
from yaw_controller import YawController        # Yaw controller provided
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MAX_VELOCITY = 5.556 # My car speed, 


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement

        self.decel_limit = kwargs["decel_limit"]
        self.accel_limit = kwargs["accel_limit"]
        # brake_deadband is the interval in which the brake would be ignored
        # the car would just be allowed to slow by itself/coast to a slower speed
        self.brake_deadband = kwargs["brake_deadband"]
        self.vehicle_mass = kwargs["vehicle_mass"]
        self.fuel_capacity = kwargs["fuel_capacity"]
        self.wheel_radius = kwargs["wheel_radius"]
        # bunch of parameters to use for the Yaw (steering) controller
        self.wheel_base = kwargs["wheel_base"]
        self.steer_ratio = kwargs["steer_ratio"]
        self.max_lat_accel = kwargs["max_lat_accel"]
        self.max_steer_angle = kwargs["max_steer_angle"]

        # Initialise speed PID, with tuning parameters
        # Will use this PID for the speed control
        self.pid_accel = PID(11.2, 0.05, 0.3, self.decel_limit, self.accel_limit) #TODO need to be tested
        self.pid_steer = PID(0.8, 0.05, 0.2, -self.max_steer_angle/2, self.max_steer_angle/2) #TODO need to be tested
        # second controller to get throttle signal between 0 and 1
        self.pid_throttle = PID(0.4, 0.05, 0.0, 0.0, 0.75) #TODO make sure you will need this controller

        sampling_rate = 50.0 #50 Hz by default
        self.delta_t = 1/sampling_rate
        self.brake_torque_const = (self.vehicle_mass + self.fuel_capacity \
        * GAS_DENSITY) * self.wheel_radius
        self.past_vel_linear = 0.0
        self.current_accel = 0.0
        self.low_pass_filter_accel = LowPassFilter(0.2, self.delta_t)

        # Create a steering controller
        self.yaw_steer = YawController(wheel_base=self.wheel_base, 
                                    steer_ratio=self.steer_ratio, 
                                    min_speed = 0.0, 
                                    max_lat_accel = self.max_lat_accel,
                                    max_steer_angle = self.max_steer_angle)


        pass

    def control(self, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs

        self.twist_command = kwargs['twist_command']
        self.current_velocity = kwargs['current_velocity']
        self.steering_fb = kwargs['steering_fb']
        throttle, brake, steering = 0.0, 0.0, 0.0


        # Set the Error
        velocity_err = self.twist_command.twist.linear.x - self.current_velocity.twist.linear.x
        # velocity_err = velocity_err/MAX_VELOCITY

        #TODO need modification for submission

        desired_accel = self.pid_accel.step(velocity_err, self.delta_t)

        if desired_accel > 0.0:
            if desired_accel < self.accel_limit:
                throttle = self.pid_throttle.step(desired_accel - self.current_accel, self.delta_t)
            else:
                throttle = self.pid_throttle.step(self.accel_limit - self.current_accel, self.delta_t)
            brake = 0.0
        else:
            throttle = 0.0
            # reset just to be sure
            # self.accel_pid.reset()
            if abs(desired_accel) > self.brake_deadband:
                # don't bother braking unless over the deadband level
                # make sure we do not brake to hard
                if abs(desired_accel) > abs(self.decel_limit):
                    brake = abs(self.decel_limit) * self.brake_torque_const
                else:
                    brake = abs(desired_accel) * self.brake_torque_const

        steering = self.yaw_steer.get_steering(self.twist_command.twist.linear.x,
                                                self.twist_command.twist.angular.z,
                                                self.current_velocity.twist.linear.x)

        steering = self.pid_steer.step(steering - self.steering_fb, self.delta_t)

        # Return throttle, brake, steer
        return throttle, brake, steering
        # return 1., 0., 0.
