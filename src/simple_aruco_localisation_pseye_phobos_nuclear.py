#!/usr/bin/env python
import rospy
import numpy as np
import tf
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Transform
from fiducial_msgs.msg import FiducialTransformArray
from std_srvs.srv import Empty, EmptyResponse


### Simple Localisation using Aruco markers
class SimpleArucoLocalisation():

  def __init__(self):
    # Node initiation with log level debug by default
    rospy.init_node('simple_aruco_localisation', log_level=rospy.DEBUG)
    # IIND Format    
    rospy.loginfo("--------------------------------------------------")
    rospy.loginfo(" Integrated Innovation for Nuclear Decomissioning ")
    rospy.loginfo("           Simple Aruco Localisation              ")
    rospy.loginfo(" partner: Wood                                    ")
    rospy.loginfo("--------------------------------------------------")

    #> Attributes


    #> Parameters
    frequency = rospy.get_param('~frequency', 50)
    rospy.loginfo("Frequency: %f", frequency)
    rate = rospy.Rate(frequency)
    self.parent_id = rospy.get_param('~parent_id', 'world')
    self.child_id = rospy.get_param('~child_id', 'marker')
    self.camera_optical_frame_id = rospy.get_param('~camera_optical_frame_id','i3dr_stereo_cameraLeft_optical')
    self.marker_frame_id = rospy.get_param('~marker_frame_id','frame_marker_1')
    rospy.loginfo("Frequency: %f", frequency)
    camera_frames = [self.camera_optical_frame_id]
    marker_frames = [self.marker_frame_id]
    
    tfl = tf.TransformListener()
    tft = tf.TransformerROS()
    tfb = tf.TransformBroadcaster()

    ## Loop
    while not rospy.is_shutdown():

      try:
        temps = rospy.Time(0)
        ts,oks = [],[False,False]
        for i in range(len(camera_frames)):
          if tfl.canTransform( camera_frames[i], marker_frames[i], temps): # Should check with fiducial transforms
            (t, q) = tfl.lookupTransform(camera_frames[i], marker_frames[i], temps)
            ts.append(t)
            oks[i] = True

        if not oks[0] and not oks[1]:
          rospy.logerr_throttle(1,"Transformations not available")
          continue
        else:
          if not oks[1]:
            marker_frame = marker_frames[0]
          elif not oks[0]:
            marker_frame = marker_frames[1]
          else:
            # Check which marker is closer and choose that location
            if np.linalg.norm(ts[0]) <= np.linalg.norm(ts[1]):
              marker_frame = marker_frames[0]
            else:
              marker_frame = marker_frames[1]

          rospy.loginfo_throttle(1,marker_frame.split('_')[0])
          (t_w_m, q_w_m) = tfl.lookupTransform("world", marker_frame, temps)
          tfb.sendTransform(t_w_m, tf.transformations.quaternion_from_euler(q_w_m[0],q_w_m[1],q_w_m[2]), temps, self.child_id, self.parent_id)
      
      except tf.Exception as e:
        rospy.logwarn("error while waiting for frames: {0}".format(e))
      
      rate.sleep()



### Main
if __name__ == '__main__':
    try:
        main = SimpleArucoLocalisation()
    except rospy.ROSInterruptException:
        rospy.logwarn("Exception")
        pass