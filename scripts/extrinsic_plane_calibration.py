#! /usr/bin/env python

import sys
import roslib
import math
import numpy as np
import rospy
import tf
from std_msgs.msg import Float32MultiArray

class ExtrinsicPlaneCalibration:

    def __init__(self):        
#         self.camera_info_relay = rospy.Publisher("camera_info_relay", CameraInfo, queue_size=10)  
        
        self.camera_info_subscriber = rospy.Subscriber("plane_coefficients", Float32MultiArray, self.plane_cb)
        self.camera_transform = "sensor_3d_depth_frame"
        self.tf_broadcaster = tf.TransformBroadcaster()

    def unit_vector(self, vector):
        return vector / np.linalg.norm(vector)


    def angle_between(self, v1, v2):
        v1_u = self.unit_vector(v1)
        v2_u = self.unit_vector(v2)
        angle = np.arccos(np.dot(v1_u, v2_u))
        if np.isnan(angle):
            if (v1_u == v2_u).all():
                return 0.0
            else:
                return np.pi
        return angle

    def plane_cb(self, plane_coefficients):
        
        x = -plane_coefficients.data[0]
        y = -plane_coefficients.data[1]
        z = -plane_coefficients.data[2]
        
        plane_normal = [x, y, z]
        
        angle_x = - self.angle_between([1, 0, 0], plane_normal)
        angle_y = - self.angle_between([0, 1, 0], plane_normal) - (np.pi / 4)
        angle_z = - self.angle_between([0, 0, 1], plane_normal)
        
        rospy.loginfo("Angles: %s %s %s", np.degrees(angle_x), np.degrees(angle_y), np.degrees(angle_z))
        
        self.tf_broadcaster.sendTransform((0, 0, 0),
            tf.transformations.quaternion_from_euler(angle_x, angle_y, angle_z),
            rospy.Time.now(),
            "calibrated_cam",
            self.camera_transform)

def main(args):
    rospy.init_node('extrinsic_plane_calibration', anonymous=True)
    plane_calibration = ExtrinsicPlaneCalibration()   
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
          

if __name__ == '__main__':
    main(sys.argv)