#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

# import numpy
# import tf_conversions
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_multiply
# from tf.transformations import *

last_target = Pose()
last_pose = Pose()

scale = 1.0
max_linear_vel = 1.0 * scale
min_linear_vel = 0.2 * scale
max_angular_vel = 0.1 * scale
min_angular_vel = 0.05 * scale

def clamp(n, smallest, largest): return max(smallest, min(n, largest))

def callback_current_pose(data):
  global last_pose
  
  # get the last pose
  last_pose = data.pose

def callback_target(data):
  global last_target

  # update the new target
  last_target = data

def compute_z_vel(base_pose: Pose, target_pose: Pose):
  height_diff = target_pose.position.z - base_pose.position.z
  vel_z = 0

  if height_diff > 0.1:
    vel_z = clamp(height_diff, min_linear_vel, max_linear_vel)
  elif height_diff < -0.1:
    vel_z = clamp(height_diff, -max_linear_vel, -min_linear_vel)

  return vel_z

def compute_linear_vel(base_pose: Pose, target_pose: Pose):
  linear_vel = Vector3()

  linear_vel.z = compute_z_vel(last_pose, last_target)
  linear_vel.x = 0 # to compute
  linear_vel.y = 0 # to compute

  return linear_vel

def compute_relative_rotation(reference_rot: Quaternion, target_rot: Quaternion):

  # invert refence rotation
  reference_rot_arr = [reference_rot.x, reference_rot.y, reference_rot.z, -reference_rot.w]
  target_rot_arr = [target_rot.x, target_rot.y, target_rot.z, target_rot.w]

  # compute
  res_array = quaternion_multiply(target_rot_arr, reference_rot_arr)
  result = Quaternion(x=res_array[0], y=res_array[1], z=res_array[2], w=res_array[3])

  # result = difference_quaternion(target_rot, reference_rot)

  return result

def compute_angular_vel(base_pose: Pose, target_pose: Pose):

  ## compute the Z Axis difference
  target_rotation = quaternion_from_euler(0, 0, 0)
  target_rotation = Quaternion(target_rotation[0], target_rotation[1], target_rotation[2], target_rotation[3])

  base_rot = base_pose.orientation

  relative_rot = compute_relative_rotation(base_rot, target_rotation)
  relative_rot_euler = euler_from_quaternion([relative_rot.x, relative_rot.y, relative_rot.z, relative_rot.w])

  diff_z = relative_rot_euler[2] # z axis

  # rospy.loginfo("[base_rot: %s, target_rotation: %s, relative_rot:%s, relative_rot_euler:%s, diff_z:%s]", base_rot, target_rotation, relative_rot, relative_rot_euler, diff_z)

  ## compute the Z Angular velocity

  diff_z = -diff_z
  vel_z = 0

  if diff_z > 0.1:
    vel_z = clamp(diff_z, min_angular_vel, max_angular_vel)
  elif diff_z < -0.1:
    vel_z = clamp(diff_z, -max_angular_vel, -min_angular_vel)

  angular_vel = Vector3()

  angular_vel.x = 0 # to compute
  angular_vel.y = 0 # to compute
  angular_vel.z = vel_z # to compute

  return angular_vel

def compute_cmd_vel():
  velocity = Twist()

  velocity.linear = compute_linear_vel(last_pose, last_target)
  velocity.angular = compute_angular_vel(last_pose, last_target)

  return velocity

def runner():
  rospy.init_node('drone_controller', anonymous=True)

  rospy.Subscriber("/ground_truth_to_tf/pose", PoseStamped, callback_current_pose)
  rospy.Subscriber("/drone/target", Pose, callback_target)

  cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
  update_rate = rospy.Rate(10)

  while not rospy.is_shutdown():
    # show data
    # rospy.loginfo("[Pose: %s, Target: %s]", last_pose, last_target)

    # compute cmd vel
    cmd = compute_cmd_vel()

    # publish cmd vel
    cmd_vel_pub.publish(cmd)

    # sleep
    update_rate.sleep()

if __name__ == '__main__':
  runner()
