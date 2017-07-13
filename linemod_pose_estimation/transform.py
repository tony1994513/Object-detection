#!/usr/bin/env python


import roslib
roslib.load_manifest('trac_ik_examples')
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Pose
import tf.transformations
import numpy as np

def pose_to_mat(pose):
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
    
    
    
def mat_to_pose(mat, transform = None):
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

def callback(data):
    obj_on_camera_pose = Pose()
    obj_on_camera_pose.position.x  = data.translation.x
    obj_on_camera_pose.position.y  = data.translation.y
    obj_on_camera_pose.position.z  = data.translation.z
    obj_on_camera_pose.orientation.x = data.rotation.x
    obj_on_camera_pose.orientation.y = data.rotation.y
    obj_on_camera_pose.orientation.z = data.rotation.z
    obj_on_camera_pose.orientation.w = data.rotation.w
    obj_on_camera_mat = pose_to_mat(obj_on_camera_pose)
    
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
    camera_on_base_mat = pose_to_mat(camera_on_base_pose) 
##############################################################    
    obj_on_base_mat = obj_on_camera_mat * camera_on_base_mat
    obj_on_base_pose = mat_to_pose(obj_on_base_mat)
   
    rospy.loginfo("successful publish obj_on_base_pose.position = %d ",obj_on_base_pose.position.x)
    pub = rospy.Publisher('object_in_base', Pose, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.loginfo("successful publish")
        pub.publish(obj_on_base_pose)
        rate.sleep()
    
    
   
def main():
    rospy.init_node('transform_change', anonymous=True)
    rospy.Subscriber("object_pose", geometry_msgs.msg.Transform, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass