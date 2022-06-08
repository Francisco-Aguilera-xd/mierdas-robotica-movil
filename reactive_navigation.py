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
        self.vel_linear = 0.2
        self.vel_angular = None

        self.sum_max = 0.14 * 10
        self.sum_margin = 0.2 *10
        self.delta_max = 0.1*10

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
        #self.follow_aisle(left_dist, right_dist, maximum_index)

    def show_img(self):
        img2display = self.depth_image * (255.0/self.depth_image.max())
        img2display = img2display.astype(np.uint8)
        img2display = 255 - img2display
        img2display = cv2.applyColorMap(img2display, cv2.COLORMAP_BONE)
        cv2.imshow("Depth Sensor", img2display)
        cv2.waitKey(1)

    def rot_control_cb(self, data: Float64):
        self.vel_angular = data.data
        if abs(data.data) < 0.1 or np.isnan(data.data):
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

        # Ajuste original:
        #self.process_img()

        # Ajuste por suma absoluta:
        #self.ajuste_por_suma_matrices()
        #self.ajuste_por_suma()

        # Ajuste por promedio
        self.ajuste_por_promedio_matrices()
        self.ajuste_por_promedio()
        

    def ajuste_por_suma(self):
        self.rot_setpoint.publish(0)
        delta = self.der - self.iz

        if self.iz == 0 or self.der == 0:
            self.rot_state.publish(delta)

        else:

            if self.iz > self.sum_max and abs(delta) > self.delta_max:  # pierde izquierda
                rospy.loginfo("lost left wall")

                diferencia_aprox = 0.8 - self.der + 0.8

                self.rot_state.publish(diferencia_aprox)

            elif self.der > self.sum_max and abs(delta) > self.delta_max:  # pierde derecha
                rospy.loginfo("lost right wall")
                diferencia_aprox = 0.8 - self.iz + 0.8

                self.rot_state.publish(diferencia_aprox)
            else:
                rospy.loginfo("find wall")
                self.rot_state.publish(delta)


        pass

    def ajuste_por_suma_matrices(self):
        iz, _, _, mid, _, _, der = np.array_split(self.depth_image, 7, axis=1)
        
        iz = np.delete(iz, np.where(iz >= 10))
        mid = np.delete(mid, np.where(mid >= 10))
        der = np.delete(der, np.where(der >= 10))
        

        print(f"iz: {np.sum(iz)/1e5}")
        print(f"mid: {np.sum(mid)/1e5}")
        print(f"der: {np.sum(der)/1e5}")

        self.iz = np.sum(iz)/1e5
        self.der = np.sum(der)/1e5

        print()

    def ajuste_por_promedio_matrices(self):
        iz, mid, der = np.array_split(self.depth_image, 3, axis=1)
        
        iz = np.mean(iz)
        mid = np.mean(mid)
        der = np.mean(der)
        

        #print(f"iz: {round(iz, 3)}")
        #print(f"mid: {round(mid, 3)}")
        #print(f"der: {round(der, 3)}")

        self.iz = iz
        self.mid = mid
        self.der = der

        print()

    def ajuste_por_promedio(self):
        self.rot_setpoint.publish(0)
        diferencia = self.der - self.iz


        if self.iz <= 0.2:
            print("Muy pegado a la izq")
            self.rot_state.publish(- 10)
        elif self.der <= 0.2:
            print("Muy pegado a la der")
            self.rot_state.publish( + 10)

        else:

            if diferencia >= -2 and self.mid < self.iz:
                # La izquierda se pierde, la medicion de iz no es fiable
                print("Perdí la izq")
                print(f"mid: {round(self.mid, 3)}")
                print(f"der: {round(self.der, 3)}")

                # iz_simulada = 2*self.der - self.mid
                iz_simulada = self.mid - self.der

                self.rot_state.publish(self.der - iz_simulada)
                pass

            elif diferencia >= +2 and self.mid < self.der:
                # La derecha se pierde, la medicion de der no es fiable
                print("Perdí la der")
                print(f"iz: {round(self.iz, 3)}")
                print(f"mid: {round(self.mid, 3)}")

                #der_simulada = 2*self.iz - self.mid
                der_simulada = self.mid - self.iz

                self.rot_state.publish(der_simulada - self.mid)
                
                pass


            else:
                self.rot_state.publish(diferencia)

if __name__ == "__main__":
    react_nav = ReactiveNavigation()
    rospy.spin()
