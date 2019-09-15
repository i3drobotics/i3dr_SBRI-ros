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

        self.frame_4_id = rospy.get_param(
            '~frame_4_id', "base_link") #base_link / ur10_base_link
        self.frame_3_id = rospy.get_param(
            '~frame_3_id', "tool0") #tool0 / end_effector
        self.frame_1_id = rospy.get_param(
            '~frame_1_id', "map")
        self.frame_2_id = rospy.get_param(
            '~frame_2_id', "i3dr_stereo_base_link")
        self.output_frame_1_id = rospy.get_param(
            '~output_frame_1_id', "map")
        self.output_frame_2_id = rospy.get_param(
            '~output_frame_2_id', "ur10_base_link")
        self.update_mode = rospy.get_param(
            '~update_mode', "continuous"
        )
        self.service_name = rospy.get_param(
            '~service_name', "i3dr_get_scan_home"
        )

        if self.update_mode == "continuous":
            self.service_update = False
        elif self.udpate_mode == "service":
            self.service_update = True

        if self.service_update:
          rospy.Service(self.service_name, Empty,
                        self.handle_get_tfs)

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
            scale, shear, rpy, trans, perspective = tf.transformations.decompose_matrix(
                self.map_to_robot_base_tf)
            self.tfBroadcaster.sendTransform(trans, tf.transformations.quaternion_from_euler(
                rpy[0], rpy[1], rpy[2]), rospy.Time().now(), self.output_frame_1_id, self.output_frame_2_id)

    def get_frame_tf(self, frame_a_id, frame_b_id):
        when = rospy.Time.now() - rospy.Duration(5.0)
        self.tfListener.waitForTransform(frame_a_id, frame_b_id, when, rospy.Duration(5.0))
        tfAB = self.tfListener.lookupTransform(frame_a_id, frame_b_id, when)

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
        # get tf from frame_1 to frame_2
        group_1_tf = self.get_frame_tf(self.frame_1_id, self.frame_2_id)
        # get tf from frame_3 to frame_4
        group_2_tf = self.get_frame_tf(self.frame_3_id, self.frame_4_id)

        # frame_2 = frame_3
        # frame_1 -> frame_4 = (frame_1->frame_2 x frame_3->frame_4)
        self.map_to_robot_base_tf = self.calc_map_scan_tf(group_1_tf, group_2_tf)

        rospy.loginfo("Map -> ur10 base link TF calculated and publishing")
        self.tf_ready = True
      except Exception as e: 
        self.tf_ready = False
        print(e)


if __name__ == '__main__':
    rt = ArucoSetMap()
    rt.spin(100)
