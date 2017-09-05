#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        """
        1. Grab initial parameters.
        2. Subscribe required nodes for  realtime data.
        3. Create nodes to publish contoller cmd.
        4. Create control cmd
        5. Loop realtime loop
        """

        self.dbw_enabled = True
        self.waypoints = None
        self.velocity = None
        self.pose = None
        self.twist = None

        rospy.init_node('dbw_node')

        #vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        #fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        #brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        #wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # TODO: Create `TwistController` object
        # self.controller = TwistController(<Arguments you wish to provide>)
        controller_args = {'min_acc': decel_limit,
                           'max_acc': accel_limit,
                           'wheel_base': wheel_base,
                           'steer_ratio': steer_ratio,
                           'max_lat_accel': max_lat_accel,
                           'max_steer_angle': max_steer_angle
                          }
        self.controller = Controller(**controller_args)
        self.yaw_controller = YawController(
            wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        # Subscribe to all the topics you need to
        rospy.Subscriber('/vehicle/dbw_enabled', Bool,
                         self.dbw_cb, queue_size=1)
        rospy.Subscriber('/final_waypoints', Lane,
                         self.waypoints_cb, queue_size=1)
        rospy.Subscriber('/current_pose', PoseStamped,
                         self.pose_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped,
                         self.velocity_cb, queue_size=1)
        rospy.Subscriber('/twist_cmd', TwistStamped,
                         self.twist_cb, queue_size=1)


        self.loop()

    def loop(self):

        rate = rospy.Rate(10) # 10Hz
        #rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)
            # if <dbw is enabled>:
            #   self.publish(throttle, brake, steer)
            while not rospy.is_shutdown():
            grab_data = [self.velocity, self.waypoints, self.pose]
            all_available = all([x is not None for x in grab_data])

            if not all_available:
                continue
            # check if too few waypoints and publish a hard break
            if len(self.waypoints) < dbw_helper.POINTS_TO_FIT:
                rospy.logwarn("Number of waypoint received: %s",
                              len(self.waypoints))
                throttle, brake, steer = 0, -5, 0
            else:
                # Read target and current velocities
                cte = dbw_helper.cte(self.pose, self.waypoints)
                target_velocity = self.waypoints[0].twist.twist.linear.x
                current_velocity = self.velocity.linear.x
                vel_error = target_velocity - current_velocity
                # Get predicted throttle, brake, and steering using `twist_controller`
                throttle, brake, steer = self.controller.control(vel_error,
                                                                 cte,
                                                                 self.dbw_enabled)
                # Get predicted steering angle from road curvature
                yaw_steer = self.yaw_controller.get_steering(self.twist.linear.x,
                                                             self.twist.angular.z,
                                                             current_velocity)
                # Apply full deacceleration if target velocity is zero
                brake = -5 if self.twist.linear.x == 0 else brake
            if self.dbw_enabled:
                self.publish(throttle, brake, steer + PREDICTIVE_STEERING * yaw_steer)

            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    def dbw_cb(self, message):
        """From the incoming message extract the dbw_enabled variable """
        self.dbw_enabled = bool(message.data)

    def velocity_cb(self, message):
        """From the incoming message extract the velocity message """
        self.velocity = message.twist

    def pose_cb(self, message):
        """From the incoming message extract the pose message """
        self.pose = message.pose

    def twist_cb(self, message):
        """From the incoming message extract the pose message """
        self.twist = message.twist

    def waypoints_cb(self, message):
        """Update final waypoints array when a new message arrives
        on the corresponding channel
        """
        self.waypoints = message.waypoints

if __name__ == '__main__':
    DBWNode()
