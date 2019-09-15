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
        rospy.init_node("i3dr_set_tf_parent",
                        anonymous=True, disable_signals=True)

        self.map_to_robot_base_tf = None
        self.tf_ready = False

        self.ur10_base_frame_id = rospy.get_param(
            '~ur10_base_frame_id', "base_link") #base_link / ur10_base_link
        self.end_effector_frame_id = rospy.get_param(
            '~end_effector_frame_id', "tool0") #tool0 / end_effector
        self.map_frame_id = rospy.get_param(
            '~map_frame_id', "map")
        self.camera_frame_id = rospy.get_param(
            '~camera_frame_id', "i3dr_stereo_base_link")
        self.service_update = rospy.get_param(
            '~service_update', False
        )

        if self.service_update:
          rospy.Service('/i3dr_stereo/set_map_home', Empty,
                        self.handle_get_tfs)

        #self.tfBuffer = tf2_ros.Buffer()
        #self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.tfListener = tf.TransformListener()
        self.tfTransformer = tf.TransformerROS()
        self.tfBroadcaster = tf.TransformBroadcaster()

    def spin(self, rate_hz):
        rate = rospy.Rate(rate_hz)
        try:
            while not rospy.is_shutdown():
              if not self.service_update:
                self.get_tfs()
              self.pub_map_scan()
              rate.sleep()
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise

    def pub_map_scan(self):
        if (self.tf_ready):
            tf_msg = geometry_msgs.msg.TransformStamped()
            tf_msg.header.stamp = rospy.Time.now()
            tf_msg.header.frame_id = self.map_frame_id
            tf_msg.child_frame_id = self.ur10_base_frame_id
            tf_msg.transform = self.map_to_robot_base_tf

            scale, shear, rpy, trans, perspective = tf.transformations.decompose_matrix(
                self.map_to_robot_base_tf)
            self.tfBroadcaster.sendTransform(trans, tf.transformations.quaternion_from_euler(
                rpy[0], rpy[1], rpy[2]), rospy.Time().now(), self.ur10_base_frame_id, self.map_frame_id)

    def get_frame_tf(self, frame_a_id, frame_b_id):
        when = rospy.Time.now() - rospy.Duration(5.0)
        self.tfListener.waitForTransform(frame_a_id, frame_b_id, when, rospy.Duration(5.0))
        tfAB = self.tfListener.lookupTransform(frame_a_id, frame_b_id, when)

        # tf = self.tfListener.transformPose(
        #     frame_a_id, frame_b_id)
        return tfAB

    def calc_map_scan_tf(self, frame_group_a_tf, frame_group_b_tf):
        T_ma_m = self.tfTransformer.fromTranslationRotation(
            frame_group_a_tf[0], frame_group_a_tf[1])
        T_as_m = self.tfTransformer.fromTranslationRotation(
            frame_group_b_tf[0], frame_group_b_tf[1])
        T_ms_m = tf.transformations.concatenate_matrices(T_ma_m, T_as_m)
        return T_ms_m

    def handle_get_tfs(self, req):
      self.get_tfs()
      return EmptyResponse()

    def get_tfs(self):
      try:
        group_1_tf = self.get_frame_tf(self.map_frame_id, self.camera_frame_id)
        #print(group_1_tf)
        # get tf from map to aruco
        group_2_tf = self.get_frame_tf(self.end_effector_frame_id, self.ur10_base_frame_id)
        #print(group_2_tf)
        # calculate tf from map to scan and update transform publisher tf (scan to map)
        self.map_to_robot_base_tf = self.calc_map_scan_tf(group_1_tf, group_2_tf)

        rospy.loginfo("Map -> ur10 base link TF calculated and publishing")
        self.tf_ready = True
      except Exception as e: 
        self.tf_ready = False
        print(e)


if __name__ == '__main__':
    rt = ArucoSetMap()
    rt.spin(100)
