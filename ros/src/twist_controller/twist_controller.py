from lowpass import LowPassFilter
from pid import PID
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        # pass
        
        tau = 0.5
        ts = .02
        self.vel_lpf = LowPassFilter(tau, ts)

        kp = 0.3
        ki = 0.1
        kd = 0.
        mn = 0.
        mx = 0.5
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        wheel_base = kwargs['wheel_base']
        steer_ratio = kwargs['steer_ratio']
        max_lat_accel = kwargs['max_lat_accel']
        max_steer_angle = kwargs['max_steer_angle']
        min_speed = 0.1
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        self.vehicle_mass = kwargs['vehicle_mass']
        self.fuel_capacity = kwargs['fuel_capacity']
        self.brake_deadband = kwargs['brake_deadband']
        self.decel_limit = kwargs['decel_limit']
        self.accel_limit = kwargs['accel_limit']
        self.wheel_radius = kwargs['wheel_radius']

        self.last_time = rospy.get_time()


    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        # return 1., 0., 0.
        
        proposed_linear_velocity = args[0]
        proposed_angular_velocity = args[1]
        current_linear_velocity = args[2]
        dbw_status = args[3]

        if not dbw_status:
            self.throttle_controller.reset()
            return 0., 0., 0.

        current_linear_velocity = self.vel_lpf.filt(current_linear_velocity)

        # http://wiki.ros.org/rospy/Overview/Logging
        # rospy.logwarn("Target linear velocity: {0}".format(proposed_linear_velocity))
        # rospy.logwarn("Target angular velocity: {0}".format(proposed_angular_velocity))
        # rospy.logwarn("Current linear velocity: {0}".format(current_linear_velocity))
        # rospy.logwarn("filtered linear velocity: {0}".format(self.vel_lpf.get()))

        steer = self.yaw_controller.get_steering(proposed_linear_velocity, proposed_angular_velocity, current_linear_velocity)

        vel_error = proposed_linear_velocity - current_linear_velocity
        now_time = rospy.get_time()
        sample_time = now_time - self.last_time
        self.last_time = now_time
        throttle = self.throttle_controller.step(vel_error, sample_time)

        #rospy.loginfo("Target_lv: {0} Target_av: {1} Current_lv: {2}  filtered_lv: {3} sample_time: {4}".format(proposed_linear_velocity, 
        #    proposed_angular_velocity, 
        #    current_linear_velocity, 
        #    self.vel_lpf.get(), 
        #    sample_time))

        brake = 0
        if proposed_linear_velocity == 0. and current_linear_velocity < 0.1:
            throttle = 0
            brake = 400
        elif throttle < .1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius #Torque N*m

        # rospy.loginfo("throttle: {0} brake: {1} steer: {2}".format(throttle, brake, steer))
        return throttle, brake, steer
