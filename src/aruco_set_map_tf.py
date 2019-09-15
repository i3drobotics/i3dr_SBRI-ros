#!/usr/bin/env python

from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
import tf2_ros
import tf
import tf_conversions
import geometry_msgs.msg
import rospy
import numpy as np


class ArucoSetMap:
    def __init__(self):
        self.setupNode()

    def setupNode(self):
        rospy.init_node("i3dr_aruco_set_map",
                        anonymous=True, disable_signals=True)
        rospy.Service('i3dr_set_map_home', Empty,
                      self.handle_i3dr_set_map_home)

        self.map_scan_tf = None
        self.tf_ready = False

        self.aruco_marker_frame_id = rospy.get_param(
            '~aruco_frame_id', "aruco")
        self.map_frame_id = rospy.get_param(
            '~map_frame_id', "map")
        self.scan_frame_id = rospy.get_param(
            '~scan_frame_id', "scan")

        self.tfListener = tf.TransformListener()
        self.tfTransformer = tf.TransformerROS()
        self.tfBroadcaster = tf.TransformBroadcaster()

    def spin(self, rate_hz):
        rate = rospy.Rate(rate_hz)
        try:
            while not rospy.is_shutdown():
                self.pub_map_scan()
                rate.sleep()
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise

    def pub_map_scan(self):
        if (self.tf_ready):
            scale, shear, rpy, trans, perspective = tf.transformations.decompose_matrix(
                self.map_scan_tf)
            self.tfBroadcaster.sendTransform(trans, tf.transformations.quaternion_from_euler(
                rpy[0], rpy[1], rpy[2]), rospy.Time().now(), self.map_frame_id, self.scan_frame_id)

    def get_aruco_scan_tf(self):
        # get tf from aruco to camera (scan)
        when = rospy.Time.now() - rospy.Duration(5.0)
        self.tfListener.waitForTransform(self.aruco_marker_frame_id, self.scan_frame_id, when, rospy.Duration(5.0))
        tf = self.tfListener.lookupTransform(
            self.aruco_marker_frame_id, self.scan_frame_id, rospy.Time())
        return tf

    def get_map_aruco_tf(self):
        # get tf from map to aruco
        when = rospy.Time.now() - rospy.Duration(5.0)
        self.tfListener.waitForTransform(self.map_frame_id, self.aruco_marker_frame_id, when, rospy.Duration(5.0))
        tf = self.tfListener.lookupTransform(
            self.map_frame_id, self.aruco_marker_frame_id, rospy.Time())
        return tf

    def calc_map_scan_tf(self, aruco_scan_tf, map_aruco_tf):
        T_ma_m = self.tfTransformer.fromTranslationRotation(
            map_aruco_tf[0], map_aruco_tf[1])
        T_as_m = self.tfTransformer.fromTranslationRotation(
            aruco_scan_tf[0], aruco_scan_tf[1])
        T_ms_m = tf.transformations.concatenate_matrices(T_ma_m, T_as_m)
        return T_ms_m

    def handle_i3dr_set_map_home(self, req):
        rospy.loginfo("Request to set map -> scan tf from aruco")
        # get tf from aruco to camera (scan)
        scan_aruco_tf = self.get_aruco_scan_tf()
        # get tf from map to aruco
        map_aruco_tf = self.get_map_aruco_tf()
        # calculate tf from map to scan and update transform publisher tf (scan to map)
        self.map_scan_tf = self.calc_map_scan_tf(scan_aruco_tf, map_aruco_tf)

        rospy.loginfo("Map -> Scan TF calculated and publishing")
        self.tf_ready = True
        return EmptyResponse()


if __name__ == '__main__':
    rt = ArucoSetMap()
    rt.spin(100)
