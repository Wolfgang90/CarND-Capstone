#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        map_x = [] # Add base waypoints X array
        map_y = [] # Add base waypoints Y array
        map_wp = 0 #map waypoints base_waypoints

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement nearest waypoint ahead
        # Similar to Path Planning Project function NextWaypoint
        pose_x = msg.pose.position.x
        pose_y = msg.pose.position.y
        nearest_waypoint = self.closest_waypoint(pose_x, pose_y, self.map_x, self.map_y)
        # Look for next waypoint - LOOKAHEAD_WPS (20)
        publish_list = []
        for i in range(LOOKAHEAD_WPS):
            wp = Waypoint()
            wp.pose.pose.position.x = float(self.map_x[nearest_waypoint+i])
            wp.pose.pose.position.y = float(self.map_y[nearest_waypoint+i])
            wp.pose.pose.position.z = float(0.0)
            publush_list.append(wp)
        # publish the Lane() data structure
        lane = Lane()
        lane.header.frame_id = msg.header.frame_id
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = publish_list

        # TODO: need to check if always need publishing on pose_cb, possibly performance issues
        self.final_waypoints_pub.publish(lane)
        pass

    def waypoints_cb(self, waypoints):
        # Create two lists one with X and other with Y waypoints
        new_map_wp = len(waypoints.waypoints)
        new_map_x = []
        new_map_y = []
        for waypoint in waypoints.waypoints[:]:
            new_map_x.append(waypoint.pose.pose.position.x)
            new_map_y.append(waypoint.pose.pose.position.y)
        # Assign to global variables
        self.map_x = new_map_x # x waypoint
        self.map_y = new_map_y # y waypoint
        self.map_wp = new_map_wp # base waypoints
        # TODO: Implement
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    # Find distance between two points
    def dist_two_points(self, x1, y1, x2, y2):
        distance = math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))
        return distance

    # Find nearest waypoint
    def closest_waypoint(self, x, y, maps_x, maps_y):
        closest_wp_dist = 999999.9;
        closest_wp = 0;

        for i in range(self.map_wp):
            map_x = maps_x[i]
            map_y = maps_y[i]
            dist = self.dist_two_points(x, y, map_x, map_y)
            if (dist < closest_wp_dist):
                closest_wp_dist = dist
                closest_wp = i
        return closest_wp

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
