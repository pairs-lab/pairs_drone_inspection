import rospy
from sensor_msgs.msg import Image
import threading
import time

def img_cb(msg):
    print("Received image!")

rospy.init_node('test_sim')
rospy.Subscriber('/uav1/front_rgbd/infra1/image_raw', Image, img_cb)
time.sleep(5)
