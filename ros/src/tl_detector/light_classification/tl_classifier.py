# tl_classifier trial 

import numpy
import PIL
import sys
import rospy
import message_filters

from PIL import Image
from cv_bridge import CvBridge
from keras.models import load_model
from keras.preprocessing.image import img_to_array
from styx_msgs.msg import TrafficLight
from styx_msgs.msg import TrafficLightArray
# Need to import Image ??

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        self.RED = 0
        self.GREEN = 1
        self.UNKWNOWN = 2
        self.model = None
        self.RADIUS_MULTIPLIER = 6
        pass

    def predict_light(cropped_roi):
      loaded_model = load_model('light_classifier_model.h5')
      image_array = img_to_array(cropped_roi.resize((64, 64), PIL.Image.ANTIALIAS))
      prediction = loaded_model.predict(image_array[None, :])
      if prediction[0][0] == 1:
        return self.GREEN
      elif prediction[0][1] == 1:
        return self.RED
      else:
        return self.UNKNOWN

    def calculate_bounds(signal):
      xmin = sys.maxint
      xmax = -sys.maxint - 1
      ymin = sys.maxint
      ymax = -sys.maxint - 1
      radius_max = 0

      for signal in signal.Signals:
        x = signal.u
        y = signal.v
        radius_max = max(radius_max, signal.radius)
        xmin = min(xmin, x)
        xmax = max(xmax, x)
        ymin = min(ymin, y)
        ymax = max(ymax, y)

      return int(xmin - RADIUS_MULTIPLIER * radius_max), int(xmax + RADIUS_MULTIPLIER * radius_max), int(
        ymin - RADIUS_MULTIPLIER * radius_max), int(ymax + RADIUS_MULTIPLIER * radius_max)

    def crop_image(image, xmin, xmax, ymin, ymax):
      return image.crop((xmin, ymin, xmax, ymax))

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        if len(signal.Signals) == 0:
            # No signals are visible
            light_detected_publisher.publish(traffic_light(traffic_light=UNKNOWN))
            return

         # Convert the image to PIL
         cv_bridge = CvBridge()
         cv_image = cv_bridge.imgmsg_to_cv2(image, "rgb8")
         image = PIL.Image.fromarray(cv_image)

        # Find the bounds of the signal
        xmin, xmax, ymin, ymax = calculate_bounds(signal)
        # Crop the image for the ROI
        cropped_roi = crop_image(image, xmin, xmax, ymin, ymax)
        #roi_image = rospy.Publisher('roi_image', Image, queue_size=1)
        roi_image.publish(cv_bridge.cv2_to_imgmsg(numpy.array(cropped_roi), "rgb8"))
        # Run the cropped image through the NN
        prediction = predict_light(cropped_roi)

        # Publish the prediction
        light_detected_publisher.publish(traffic_light(traffic_light=prediction))

        return TrafficLight.UNKNOWN
