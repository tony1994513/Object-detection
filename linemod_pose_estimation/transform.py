#!/usr/bin/env python


import roslib
roslib.load_manifest('trac_ik_examples')
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import (Pose ,PoseStamped)
import tf.transformations
import numpy as np

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
import baxter_interface
from std_msgs.msg import (
    Header,
    Empty,
)
import struct
import sys
import copy

class Pose_transform(object):
    def __init__(self, limb="right"):
       
        self._limb_name = limb # string
        self._limb = baxter_interface.Limb(limb)
        self._verbose = False 
        self._gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        
        
    def pose_to_mat(self,pose):
        '''
        Convert a pose message to a 4x4 numpy matrix.
      
        **Args:**
      
            **pose (geometry_msgs.msg.Pose):** Pose rospy message class.
      
        **Returns:**
            mat (numpy.matrix): 4x4 numpy matrix
        '''
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        pos = np.matrix([pose.position.x, pose.position.y, pose.position.z]).T
        mat = np.matrix(tf.transformations.quaternion_matrix(quat))
        mat[0:3, 3] = pos
        return mat
        
        
        
    def mat_to_pose(self,mat, transform = None):
        '''
        Convert a homogeneous matrix to a Pose message, optionally premultiply by a transform.
      
        **Args:**
      
            **mat (numpy.ndarray):** 4x4 array (or matrix) representing a homogenous transform.
      
            *transform (numpy.ndarray):* Optional 4x4 array representing additional transform
      
        **Returns:**
            pose (geometry_msgs.msg.Pose): Pose message representing transform.
        '''
        if transform != None:
            mat = np.dot(transform, mat)
        pose = Pose()
        pose.position.x = mat[0,3]
        pose.position.y = mat[1,3]
        pose.position.z = mat[2,3]
        quat = tf.transformations.quaternion_from_matrix(mat)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return pose
    
    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
    
    def _servo_to_pose(self, pose):
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)
    
    def ik_request(self, pose):
            hdr = Header(stamp=rospy.Time.now(), frame_id='base')
            ikreq = SolvePositionIKRequest()
            ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
            try:
                resp = self._iksvc(ikreq)
            except (rospy.ServiceException, rospy.ROSException), e:
                rospy.logerr("Service call failed: %s" % (e,))
                return False
            # Check if result valid, and type of seed ultimately used to get solution
            # convert rospy's string representation of uint8[]'s to int's
            resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
            limb_joints = {}
            if (resp_seeds[0] != resp.RESULT_INVALID):
                seed_str = {
                            ikreq.SEED_USER: 'User Provided Seed',
                            ikreq.SEED_CURRENT: 'Current Joint Angles',
                            ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                           }.get(resp_seeds[0], 'None')
                if self._verbose:
                    print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                             (seed_str)))
                # Format solution into Limb API-compatible dictionary
                limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
                if self._verbose:
                    print("IK Joint Solution:\n{0}".format(limb_joints))
                    print("------------------")
            else:
                rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
                return False
            return limb_joints
    
    def callback(self,data):
        obj_on_camera_pose = Pose()
        obj_on_camera_pose.position.x  = data.translation.x
        obj_on_camera_pose.position.y  = data.translation.y
        obj_on_camera_pose.position.z  = data.translation.z
        obj_on_camera_pose.orientation.x = data.rotation.x
        obj_on_camera_pose.orientation.y = data.rotation.y
        obj_on_camera_pose.orientation.z = data.rotation.z
        obj_on_camera_pose.orientation.w = data.rotation.w
        obj_on_camera_mat = self.pose_to_mat(obj_on_camera_pose)
        
        camera_on_base_pose = Pose()
         # for static transform about camera and base
    ##############Need to change  s########################3####
        camera_on_base_pose.position.x = 0 
        camera_on_base_pose.position.y = 0
        camera_on_base_pose.position.z = 0
        camera_on_base_pose.orientation.x = 0
        camera_on_base_pose.orientation.y = 0
        camera_on_base_pose.orientation.z = 0
        camera_on_base_pose.orientation.w = 1
        camera_on_base_mat = self.pose_to_mat(camera_on_base_pose) 
    ##############################################################    
        obj_on_base_mat = obj_on_camera_mat * camera_on_base_mat
        obj_on_base_pose = self.mat_to_pose(obj_on_base_mat)
        
        joint_space = self.ik_request(obj_on_base_pose)
        rospy.loginfo("Move to desired position")        
        self._servo_to_pose(joint_space)
      
   
def main():
    rospy.init_node('transform_change', anonymous=True)
    pose_transform = Pose_transform()
    rospy.Subscriber("object_pose", geometry_msgs.msg.Transform, pose_transform.callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass