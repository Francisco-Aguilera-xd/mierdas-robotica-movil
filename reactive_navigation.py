#!/usr/bin/python3
import numpy as np
import cv2
from cv_bridge import CvBridge
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image


class ReactiveNavigation:

    def __init__(self):
        rospy.init_node("reactive_navigation")
        self.rate = rospy.Rate(10)  # Hz
        self.bridge = CvBridge()
        self.vel_linear = 0.0
        self.vel_angular = None
        self.rot_state = rospy.Publisher("/controller/rot/state",
                                         Float64, queue_size=5)
        self.rot_setpoint = rospy.Publisher("/controller/rot/setpoint",
                                            Float64, queue_size=5)
        self.rot_control_topic = rospy.Subscriber("/controller/rot/p/control_effort",
                                                  Float64, self.rot_control_cb)
        self.depth_image = None
        self.aisle_width = 0.0
        self.depth_img_topic = rospy.Subscriber("/camera/depth/image_raw",
                                                Image, self.depth_image_cb)
        self.vel_mux_topic = rospy.Publisher("/yocs_cmd_vel_mux/input/navigation",
                                             Twist, queue_size=5)

    def follow_aisle(self, left_dist, right_dist, maximum_index,
                     min_index=60, max_index=420):
        self.rot_setpoint.publish(0)
        if maximum_index < min_index:  # pierde izquierda
            rospy.loginfo("lost left wall")
            self.rot_state.publish(2*right_dist - self.aisle_width)
        elif max_index < maximum_index:  # pierde derecha
            rospy.loginfo("lost right wall")
            self.rot_state.publish(self.aisle_width - 2*left_dist)
        else:
            rospy.loginfo("find wall")
            self.aisle_width = left_dist + right_dist
            self.rot_state.publish(right_dist - left_dist)

    def process_img(self, height=240, range_=5):
        # 480 x 640
        line = self.depth_image[height][300:-300]
        left_dist = np.mean(line[:range_])
        right_dist = np.mean(line[-range_:])
        maximum_index = np.where(line == np.amax(line))[0][0]
        self.follow_aisle(left_dist, right_dist, maximum_index)

    def show_img(self):
        img2display = self.depth_image * (255.0/self.depth_image.max())
        img2display = img2display.astype(np.uint8)
        img2display = 255 - img2display
        img2display = cv2.applyColorMap(img2display, cv2.COLORMAP_BONE)
        cv2.imshow("Depth Sensor", img2display)
        cv2.waitKey(1)

    def rot_control_cb(self, data: Float64):
        self.vel_angular = data.data
        if abs(data.data) < 0.0001 or np.isnan(data.data):
            self.vel_angular = 0.0
        speed = Twist()
        speed.linear.x = self.vel_linear
        speed.angular.z = self.vel_angular
        self.vel_mux_topic.publish(speed)

    def depth_image_cb(self, data: Image):
        depth_image_raw = self.bridge.imgmsg_to_cv2(data)
        self.depth_image = np.where(np.isnan(depth_image_raw),
                                    0.0, depth_image_raw)
        self.show_img()
        self.process_img()


if __name__ == "__main__":
    react_nav = ReactiveNavigation()
    rospy.spin()
