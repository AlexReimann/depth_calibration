#! /usr/bin/env python

import sys
import roslib
import math
import numpy as np
import rospy
import tf
from std_msgs.msg import Float32MultiArray
import xml.etree.ElementTree as ET

class ExtrinsicPlaneCalibration:

    def __init__(self):        
        self.camera_info_subscriber = rospy.Subscriber("plane_coefficients", Float32MultiArray, self.plane_cb)
        self.camera_transform = "camera_rgb_optical_frame"
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        
        self.base_footprint_frame = rospy.get_param('~base_footprint_frame', 'base_footprint')
        self.depth_points_frame = rospy.get_param('~depth_points_frame', 'camera_rgb_optical_frame')
        self.test_publish_frame = rospy.get_param('~test_publish_frame', 'calibrated_depth_sensor_frame')
        
        self.calibrated_urdf_path = rospy.get_param('~calibrated_urdf_path', 'camera_info/xtion_calibrated.urdf.xacro')
        

    def plane_cb(self, plane_coefficients):
        
        camera_axis = (0.0, 1.0, 1.0)  # 45 degree tilted around x axis
        camera_axis = camera_axis / np.linalg.norm(camera_axis)
        
        x = -plane_coefficients.data[0]
        y = -plane_coefficients.data[1]
        z = -plane_coefficients.data[2]
        d = plane_coefficients.data[3]
#         d = 0.9
                
        if(z < 0):
            z *= -1.0
            y *= -1.0
            x *= -1.0
            d *= -1.0
        
        plane_normal = [x, y, z]
        
        v = np.cross(plane_normal, camera_axis)
        s = np.linalg.norm(v)
        c = np.dot(plane_normal, camera_axis)
        v_skew = np.matrix([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        
        R = np.identity(3, dtype=np.float64) + v_skew + v_skew * v_skew * ((1 - c) / (s * s))
        R = np.vstack((R, np.matrix([0, 0, 0])))
        R = np.hstack((R, np.matrix([0, 0, 0, 1]).T))
        
        q = tf.transformations.quaternion_from_matrix(R)
        (temp, rotation_cam_foot) = self.listener.lookupTransform(self.depth_points_frame, self.base_footprint_frame, rospy.Time(0))
        rotation_cam_foot = tf.transformations.quaternion_matrix(rotation_cam_foot)
        
        (trans_foot_cam, temp) = self.listener.lookupTransform(self.base_footprint_frame, self.depth_points_frame, rospy.Time(0))
        
        distance_diff = d - trans_foot_cam[2]
        distance = rotation_cam_foot * np.matrix([0, 0, distance_diff, 1]).T
        
        self.tf_broadcaster.sendTransform(distance[0:3],
            q,
            rospy.Time.now(),
            self.test_publish_frame,
            self.camera_transform)
        
        angle_x, angle_y, angle_z = self.rpy_from_quaternion(q)
        
        rospy.loginfo("Calibration parameters: \n offset: %s \n angles: %s %s %s", distance[0:3].T, np.rad2deg(angle_x), np.rad2deg(angle_y), np.rad2deg(angle_z))
        self.save_calibration(angle_x, angle_y, angle_z, distance[0:3].tolist())
        rospy.loginfo("Calibration saved")


    def rpy_from_quaternion(self, q):
        x = q[0]
        y = q[1]
        z = q[2]
        w = q[3]
                
        sqx = x * x
        sqy = y * y
        sqz = z * z
        sqw = w * w
        pitch = 0
        
        roll = np.arctan2(2 * (y * z + w * x), sqw - sqx - sqy + sqz)
        
        sarg = -2 * (x * z - w * y)
        pitch = np.arcsin(sarg)
        
        if (sarg <= -1.0):
            pitch = -0.5 * np.pi
        
        if(sarg >= 1.0):
            pitch = 0.5 * np.pi
        
        yaw = np.arctan2(2 * (x * y + w * z), sqw + sqx - sqy - sqz)        
        return roll, pitch, yaw
        
        
    def save_calibration(self, angle_x, angle_y, angle_z, distance_vec):
        file_path = self.calibrated_urdf_path
        ET.register_namespace("xacro", "http://www.ros.org/wiki/xacro")
        
        xml_tree = ET.parse(file_path)
        camera_joint = xml_tree.getroot()[0][0]
        
        camera_joint.find("origin").set("xyz", str(distance_vec[0][0]) + " " + str(distance_vec[1][0]) + " " + str(distance_vec[2][0]))
        camera_joint.find("origin").set("rpy", str(angle_x) + " " + str(angle_y) + " " + str(angle_z)) 
        xml_tree.write(file_path, encoding="utf-8", xml_declaration=True)

def main(args):
    rospy.init_node('extrinsic_plane_calibration', anonymous=True)
    plane_calibration = ExtrinsicPlaneCalibration()   
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
          

if __name__ == '__main__':
    main(sys.argv)
