#!/usr/bin/env python
import roslib
roslib.load_manifest('nao_basic')
import rospy
import math
import tf
import geometry_msgs.msg

rospy.init_node('nao_pose')
listener = tf.TransformListener()

def get_nao_pose():
    try:
        (trans_torso    , rot_torso    ) = listener.lookupTransform('/base_link', '/torso'              , rospy.Time(0))
        (trans_neck     , rot_neck     ) = listener.lookupTransform('/base_link', '/HeadYaw_link'       , rospy.Time(0))
        (trans_head     , rot_head     ) = listener.lookupTransform('/base_link', '/CameraTop_frame'    , rospy.Time(0))
        (trans_Lshoulder, rot_Lshoulder) = listener.lookupTransform('/base_link', '/LShoulderPitch_link', rospy.Time(0))
        (trans_Rshoulder, rot_Rshoulder) = listener.lookupTransform('/base_link', '/RShoulderPitch_link', rospy.Time(0))
        (trans_LElbow   , rot_LElbow   ) = listener.lookupTransform('/base_link', '/LElbowYaw_link'     , rospy.Time(0))
        (trans_RElbow   , rot_RElbow   ) = listener.lookupTransform('/base_link', '/RElbowYaw_link'     , rospy.Time(0))
        (trans_Lwrist   , rot_Lwrist   ) = listener.lookupTransform('/base_link', '/LWristYaw_link'     , rospy.Time(0))
        (trans_Rwrist   , rot_Rwrist   ) = listener.lookupTransform('/base_link', '/RWristYaw_link'     , rospy.Time(0))
        (trans_Lgrip    , rot_Lgrip    ) = listener.lookupTransform('/base_link', '/l_gripper'          , rospy.Time(0))
        (trans_Rgrip    , rot_Rgrip    ) = listener.lookupTransform('/base_link', '/r_gripper'          , rospy.Time(0))
        (trans_Lhip     , rot_Lhip     ) = listener.lookupTransform('/base_link', '/LHipYawPitch_link'  , rospy.Time(0))
        (trans_Rhip     , rot_Rhip     ) = listener.lookupTransform('/base_link', '/RHipYawPitch_link'  , rospy.Time(0))
        (trans_Lknee    , rot_Lknee    ) = listener.lookupTransform('/base_link', '/LKneePitch_link'    , rospy.Time(0))
        (trans_Rknee    , rot_Rknee    ) = listener.lookupTransform('/base_link', '/RKneePitch_link'    , rospy.Time(0))
        (trans_Lankle   , rot_Lankle   ) = listener.lookupTransform('/base_link', '/LAnklePitch_link'   , rospy.Time(0))
        (trans_Rankle   , rot_Rankle   ) = listener.lookupTransform('/base_link', '/RAnklePitch_link'   , rospy.Time(0))
        (trans_Lsole    , rot_Lsole    ) = listener.lookupTransform('/base_link', '/l_sole'             , rospy.Time(0))
        (trans_Rsole    , rot_Rsole    ) = listener.lookupTransform('/base_link', '/r_sole'             , rospy.Time(0))
        # return [list(trans_torso), list(trans_neck)]
        return [list(trans_torso), list(trans_neck), list(trans_head), list(trans_Lshoulder), list(trans_Rshoulder),
                list(trans_LElbow), list(trans_RElBow), list(trans_Lwrist), list(trans_Rwrist), list(trans_Lgrip), list(trans_Rgrip),
                list(trans_Lhip), list(trans_Rhip), list(trans_Lknee), list(trans_RKnee), list(trans_Lankle), list(trans_Rankle),
                list(trans_Lsole), list(trans_Rsole)]
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print "exception"





if __name__ == '__main__':
    # rospy.init_node('nao_pose')
    # listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            # (trans,rot) = listener.lookupTransform('/torso', '/HeadYaw_link', rospy.Time(0))
            # print "trans", trans
            # print "rot", rot
            (trans_torso    , rot_torso    ) = listener.lookupTransform('/base_link', '/torso'              , rospy.Time(0))
            (trans_neck     , rot_neck     ) = listener.lookupTransform('/base_link', '/HeadYaw_link'       , rospy.Time(0))
            (trans_head     , rot_head     ) = listener.lookupTransform('/base_link', '/CameraTop_frame'    , rospy.Time(0))
            (trans_Lshoulder, rot_Lshoulder) = listener.lookupTransform('/base_link', '/LShoulderPitch_link', rospy.Time(0))
            (trans_Rshoulder, rot_Rshoulder) = listener.lookupTransform('/base_link', '/RShoulderPitch_link', rospy.Time(0))
            (trans_LElbow   , rot_LElbow   ) = listener.lookupTransform('/base_link', '/LElbowYaw_link'     , rospy.Time(0))
            (trans_RElbow   , rot_RElbow   ) = listener.lookupTransform('/base_link', '/RElbowYaw_link'     , rospy.Time(0))
            (trans_Lwrist   , rot_Lwrist   ) = listener.lookupTransform('/base_link', '/LWristYaw_link'     , rospy.Time(0))
            (trans_Rwrist   , rot_Rwrist   ) = listener.lookupTransform('/base_link', '/RWristYaw_link'     , rospy.Time(0))
            (trans_Lgrip    , rot_Lgrip    ) = listener.lookupTransform('/base_link', '/l_gripper'          , rospy.Time(0))
            (trans_Rgrip    , rot_Rgrip    ) = listener.lookupTransform('/base_link', '/r_gripper'          , rospy.Time(0))
            (trans_Lhip     , rot_Lhip     ) = listener.lookupTransform('/base_link', '/LHipYawPitch_link'  , rospy.Time(0))
            (trans_Rhip     , rot_Rhip     ) = listener.lookupTransform('/base_link', '/RHipYawPitch_link'  , rospy.Time(0))
            (trans_Lknee    , rot_Lknee    ) = listener.lookupTransform('/base_link', '/LKneePitch_link'    , rospy.Time(0))
            (trans_Rknee    , rot_Rknee    ) = listener.lookupTransform('/base_link', '/RKneePitch_link'    , rospy.Time(0))
            (trans_Lankle   , rot_Lankle   ) = listener.lookupTransform('/base_link', '/LAnklePitch_link'   , rospy.Time(0))
            (trans_Rankle   , rot_Rankle   ) = listener.lookupTransform('/base_link', '/RAnklePitch_link'   , rospy.Time(0))
            (trans_Lsole    , rot_Lsole    ) = listener.lookupTransform('/base_link', '/l_sole'             , rospy.Time(0))
            (trans_Rsole    , rot_Rsole    ) = listener.lookupTransform('/base_link', '/r_sole'             , rospy.Time(0))
            print trans_torso
            print trans_neck
            print type(list(trans_torso))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "exception"
            continue
        rate.sleep()
