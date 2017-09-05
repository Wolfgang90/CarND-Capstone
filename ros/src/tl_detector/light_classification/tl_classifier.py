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
import rospkg
import numpy as np
import cv2
import tensorflow as tf
from tensorflow.contrib.layers import flatten

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        self.RED = 0
        self.GREEN = 1
        self.UNKWNOWN = 2
        self.model = None
        self.RADIUS_MULTIPLIER = 6

        self.x = tf.placeholder(tf.float32, (None, 32, 32, 3))
        self.logits = self.LeNet(tf.cast(self.x, tf.float32))
        rospack = rospkg.RosPack()
        self.save_file = str(rospack.get_path('tl_detector')) + '/light_classification/save/model.ckpt'
        self.saver = tf.train.Saver()
        self.init = tf.global_variables_initializer()
        self.session = tf.Session()
        self.saver.restore(self.session, self.save_file)

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

    def LeNet(self, x):    
        # Hyperparameters
        mu = 0
        sigma = 0.1
    
        # Layer 1: Convolutional. Input = 32x32x3. Output = 28x28x6.
        conv1_W = tf.Variable(tf.truncated_normal(shape=(5, 5, 3, 6), mean = mu, stddev = sigma))
        conv1_b = tf.Variable(tf.zeros(6))
        conv1   = tf.nn.conv2d(x, conv1_W, strides=[1, 1, 1, 1], padding='VALID') + conv1_b

        # Activation.
        conv1 = tf.nn.relu(conv1)

        # Pooling. Input = 28x28x6. Output = 14x14x6.
        conv1 = tf.nn.max_pool(conv1, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='VALID')

        # Layer 2: Convolutional. Output = 10x10x16.
        conv2_W = tf.Variable(tf.truncated_normal(shape=(5, 5, 6, 16), mean = mu, stddev = sigma))
        conv2_b = tf.Variable(tf.zeros(16))
        conv2   = tf.nn.conv2d(conv1, conv2_W, strides=[1, 1, 1, 1], padding='VALID') + conv2_b
    
        # Activation.
        conv2 = tf.nn.relu(conv2)

        # Pooling. Input = 10x10x16. Output = 5x5x16.
        conv2 = tf.nn.max_pool(conv2, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='VALID')

        # Flatten. Input = 5x5x16. Output = 400.
        fc0   = flatten(conv2)
    
        # Layer 3: Fully Connected. Input = 400. Output = 50.
        fc1_W = tf.Variable(tf.truncated_normal(shape=(400, 50), mean = mu, stddev = sigma))
        fc1_b = tf.Variable(tf.zeros(50))
        fc1   = tf.matmul(fc0, fc1_W) + fc1_b
    
        # Activation.
        fc1    = tf.nn.relu(fc1)

        # Layer 4: Fully Connected. Input = 50. Output = 25.
        fc2_W  = tf.Variable(tf.truncated_normal(shape=(50, 25), mean = mu, stddev = sigma))
        fc2_b  = tf.Variable(tf.zeros(25))
        fc2    = tf.matmul(fc1, fc2_W) + fc2_b
    
        # Activation.
        fc2    = tf.nn.relu(fc2)

        # Layer 5: Fully Connected. Input = 25. Output = 5.
        fc3_W  = tf.Variable(tf.truncated_normal(shape=(25, 5), mean = mu, stddev = sigma))
        fc3_b  = tf.Variable(tf.zeros(5))
        logits = tf.matmul(fc2, fc3_W) + fc3_b
        return logits

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

        #return TrafficLight.UNKNOWN
        state = TraffficLight.UNKNOWN
        image = image[100:image.shape[0]-350, 0:image.shape[1]]
        res = cv2.resize(image, (32, 32), interpolation = cv2.INTER_CUBIC)

        def Max_Min(Array, ra=0.9, rb=0.1):
            Max = np.max(Array)
            Min = np.min(Array)
            return (ra - rb) * (Array - Min) / (Max - Min) + rb

        ind = self.session.run(np.argmax(self.logits), feed_dict = {self.x: Max_Min(res)})
        if ind == 0:
            state = TrafficLight.RED
        elif ind == 1:
            state = TrafficLight.YELLOW
        elif ind == 2:
            state = TrafficLight.GREEN

        return state
