#! /usr/bin/env python

import time
import roslib
import numpy as np
import rospy
import sys
import cv2
import sensor_msgs.point_cloud2
from std_msgs.msg import Empty
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge, CvBridgeError

class DepthCalibration:

    def __init__(self):        
        self.enable_camera_info_relay = False
        self.enable_depth_relay = False
        
        self.camera_info_relay = rospy.Publisher("camera_info_relay", CameraInfo, queue_size=10)
        self.depth_relay = rospy.Publisher("depth_relay", Image, queue_size=10)        
        self.save_calibration_publisher = rospy.Publisher("save_calibration_trigger", Empty, queue_size=10)        
        
        self.camera_info_subscriber = rospy.Subscriber("camera_info", CameraInfo, self.camera_info_cb)
        self.depth_subscriber = rospy.Subscriber("depth_raw_distorted", Image, self.depth_cb)
        self.cloud_subscriber = rospy.Subscriber("calibrated_cloud", PointCloud2, self.cloud_cb)


    def run_calibration(self):
        self.enable_camera_info_relay = True
        
        # wait till camera intrinsics have been published
        time.sleep(1)
        rospy.loginfo("%s: Camera intrinsics relayed", rospy.get_name())
        
        rospy.loginfo("%s: Starting to relay depth", rospy.get_name())
        self.enable_depth_relay = True
        
        # wait till cloud of calibrator has been published, callback unregisters afterwards
        while(self.cloud_subscriber.callback is not None):
            rospy.loginfo("%s: Waiting for point cloud of calibrator to get published", rospy.get_name())
            time.sleep(0.1);
            if(rospy.is_shutdown()):
                return
        
        wait_time = 3
        rospy.loginfo("%s: Calibration started", rospy.get_name())
        time.sleep(wait_time);  # wait some time, should take less than 3 seconds (with 10Hz)
        
        rospy.loginfo("%s: Calibration finished, requesting saving ..", rospy.get_name())
        self.save_calibration_publisher.publish(Empty())
        
    
    def camera_info_cb(self, camera_info_msg):
        if(not self.enable_camera_info_relay):
            return 
        self.camera_info_relay.publish(camera_info_msg)
    
    
    def cloud_cb(self, cloud):
        # just for waiting till stuff gets published
        rospy.loginfo("%s: Point cloud of calibrator is getting published", rospy.get_name()) 
        self.cloud_subscriber.unregister();
    
    
    def depth_cb(self, depth_image_msg):
        if(not self.enable_depth_relay):
            return
        self.depth_relay.publish(depth_image_msg)
            

def main(args):
    rospy.init_node('depth_calibration', anonymous=True)
    DepthCalibration().run_calibration()
    
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
          

if __name__ == '__main__':
    main(sys.argv)
