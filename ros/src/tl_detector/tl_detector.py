#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
from traffic_light_config import config
import numpy as np
import tf.transformations
import math

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights helps you acquire an accurate ground truth data source for the traffic light
        classifier, providing the location and current color state of all traffic lights in the
        simulator. This state can be used to generate classified images or subbed into your solution to
        help you work on another single component of the node. This topic won't be available when
        testing your solution in real life so don't rely on it in the final submission.
        '''
        # Not using for testing
        #sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/camera/image_raw', Image, self.image_cb)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        self.tl_tx = TrafficLight()
        self.deb_img = rospy.Publisher('/deb_img',Image, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 8

        self.best_waypoint = 0
        self.last_pos = 0
        self.last_light_pos_wp = []

        self.distance_light = 1000.0
        self.deviation = 0.1
        self.IGNORE_FAR_LIGHT = 100.0

        sub_bagfile = rospy.Subscriber('/image_raw', Image, self.image_cb_bag)

        rospy.spin()

    def image_cb_bag(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        res = self.light_classifier.get_classification(cv_image)
        if (res == 0):
            rospy.loginfo('Rosbag: RED')
        elif (res == 1):
            rospy.loginfo('Rosbag: Yellow')
        elif (res == 2):
            rospy.loginfo('Rosbag: Green')
        else:
            rospy.loginfo('Rosbag: Unknown')
        self.deb_img.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        return res

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    #def traffic_cb(self, msg):
    #    self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        if (state == 0):
            rospy.loginfo('Simul: RED')
        elif (state == 1):
            rospy.loginfo('Simul: Yellow')
        elif (state == 2):
            rospy.loginfo('Simul: Green')
        else:
            rospy.loginfo('Simul: Unknown')


        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        """
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1
        """
        if state == 0:
            self.state = state
            self.state_count = 0
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            if (light_wp >=0):
                self.tl_tx.pose.pose.position.x = self.waypoints.waypoints[self.last_wp].pose.pose.position.x
                self.tl_tx.pose.pose.position.y = self.waypoints.waypoints[self.last_wp].pose.pose.position.y
                self.tl_tx.pose.pose.position.z = self.waypoints.waypoints[self.last_wp].pose.pose.position.z
            self.tl_tx.state = state
            self.upcoming_red_light_pub.publish(self.tl_tx)
        else:
            if self.state_count > STATE_COUNT_THRESHOLD:
                self.state = state
                light_wp = light_wp if state == TrafficLight.RED else -1
                self.last_wp = light_wp
                if (light_wp >=0):
                    self.tl_tx.pose.pose.position.x = self.waypoints.waypoints[self.last_wp].pose.pose.position.x
                    self.tl_tx.pose.pose.position.y = self.waypoints.waypoints[self.last_wp].pose.pose.position.y
                    self.tl_tx.pose.pose.position.z = self.waypoints.waypoints[self.last_wp].pose.pose.position.z
                self.tl_tx.state = self.state
                self.upcoming_red_light_pub.publish(self.tl_tx)
                self.state_count += 1
            else:
                self.state = 0
                light_wp = light_wp if state == TrafficLight.RED else -1
                self.last_wp = light_wp
                if (light_wp >=0):
                    self.tl_tx.pose.pose.position.x = self.waypoints.waypoints[self.last_wp].pose.pose.position.x
                    self.tl_tx.pose.pose.position.y = self.waypoints.waypoints[self.last_wp].pose.pose.position.y
                    self.tl_tx.pose.pose.position.z = self.waypoints.waypoints[self.last_wp].pose.pose.position.z
                self.tl_tx.state = self.state
                self.upcoming_red_light_pub.publish(self.tl_tx)
                self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        best_waypoint = self.best_waypoint
        if self.waypoints is not None:
            waypoints = self.waypoints.waypoints
            min_dist = self.distance(pose.position, waypoints[0].pose.pose.position)
            for i, point in enumerate(waypoints):
                dist = self.distance(pose.position, point.pose.pose.position)
                if dist < min_dist:
                    best_waypoint = i
                    min_dist = dist
            self.best_waypoint = best_waypoint
            return best_waypoint

    def distance(self, p1, p2):
        delta_x = p1.x - p2.x
        delta_y = p1.y - p2.y
        return math.sqrt(delta_x*delta_x + delta_y*delta_y)

    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = config.camera_info.focal_length_x
        fy = config.camera_info.focal_length_y

        image_width = config.camera_info.image_width
        image_height = config.camera_info.image_height

        cord_x = point_in_world[0]
        cord_y = point_in_world[1]

        # get transform between pose of camera and world frame
        trans = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        #TODO Use tranform and rotation to calculate 2D position of light in image
        x = self.pose.pose.orientation.x
        y = self.pose.pose.orientation.y
        z = self.pose.pose.orientation.z
        w = self.pose.pose.orientation.w
        # Determine car heading:
        t3 = +2.0 * (w * z + x*y)
        t4 = +1.0 - 2.0 * (y*y + z*z)
        theta = math.degrees(math.atan2(t3, t4))

        Xcar = (cord_y-self.pose.pose.position.y)*math.sin(math.radians(theta))-(self.pose.pose.position.x-cord_x)*math.cos(math.radians(theta))
        Ycar = (cord_y-self.pose.pose.position.y)*math.cos(math.radians(theta))-(cord_x-self.pose.pose.position.x)*math.sin(math.radians(theta))

        self.distance_to_light = Xcar
        self.deviation_of_light = Ycar
        objectPoints = np.array([[float(Xcar), float(Ycar), 0.0]], dtype=np.float32)
        rvec = (0,0,0)
        tvec = (0,0,0)

        cameraMatrix = np.array([[fx,  0, image_width/2],
                                [ 0, fy, image_height/2],
                                [ 0,  0,  1]])
        distCoeffs = None
        ret, _ = cv2.projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs)

        x = int(ret[0,0,0])
        y = int(ret[0,0,1])

        return (x, y)

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        self.camera_image.encoding = "rgb8"
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        x, y = self.project_to_image_plane(light.pose.pose.position)

        if ((x is None) or (y is None) or (x < 0) or (y<0) or
            (x>config.camera_info.image_width) or (y>config.camera_info.image_height)):
            return TrafficLight.UNKNOWN
        else:
            # 800x600
            left = max(0, config.camera_info.image_width/2 - int(self.deviation_of_light)*10-200) # Vishnerevsky 27.08.2017
            right = min(config.camera_info.image_width/2 - int(self.deviation_of_light)*10+200, config.camera_info.image_width) # Vishnerevsky 27.08.2017

            vertical = int(y - (45.0-self.distance_to_light)*150.0/35.0)
            if vertical<0:
                vertical = 0
            top = min(vertical, config.camera_info.image_height-1)
            bottom = min(300 + vertical, config.camera_info.image_height)
            crop = cv_image[top:bottom, left:right]
            #crop = cv_image

            self.deb_img.publish(self.bridge.cv2_to_imgmsg(crop, "bgr8"))
            #rosrun image_view image_view image:=/deb_img                      #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        #Get classification
        return self.light_classifier.get_classification(crop)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        light = None
        light_positions = config.light_positions
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)

        #TODO find the closest visible traffic light (if one exists)
        light_pos_wp = []
        if self.waypoints is not None:
            wp = self.waypoints
            for i in range(len(light_positions)):
                l_pos = self.get_closest_waypoint_light(wp, light_positions[i])
                light_pos_wp.append(l_pos)
            self.last_light_pos_wp = light_pos_wp
        else:
            light_pos_wp = self.last_light_pos_wp

        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
            if car_position is not None:
                self.last_car_position = car_position

        # valtgun get id of next light
        if (self.last_car_position > max(light_pos_wp)):
             light_num_wp = min(light_pos_wp)
        else:
            light_delta = light_pos_wp[:]
            light_delta[:] = [x - self.last_car_position for x in light_delta]
            light_num_wp = min(i for i in light_delta if i > 0) + self.last_car_position

        light_idx = light_pos_wp.index(light_num_wp)
        light = light_positions[light_idx]

        light_distance = self.distance_light(light, self.waypoints.waypoints[self.last_car_position].pose.pose.position)
        #rospy.logerr('light_distance: ' + str(light_distance))

        if light:
            state = self.get_light_state(light)
            # Original return light_wp, state
            return light_num_wp, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

    def get_closest_waypoint_light(self, wp, l_pos):
        best_waypoint = None
        waypoints = wp.waypoints
        min_dist = self.distance_light(l_pos, waypoints[0].pose.pose.position)
        for i, point in enumerate(waypoints):
            dist = self.distance_light(l_pos, point.pose.pose.position)
            if dist < min_dist:
                best_waypoint = i
                min_dist = dist
        return best_waypoint

    def distance_light(self, l_p, wp):
        delta_x = l_p[0] - wp.x
        delta_y = l_p[1] - wp.y
        return math.sqrt(delta_x*delta_x + delta_y*delta_y)

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
