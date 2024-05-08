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

min_limit_height = min_linear_vel

def clamp(n, smallest, largest): return max(smallest, min(n, largest))

def callback_current_pose(data):
  global last_pose
  
  # get the last pose
  last_pose = data.pose

def callback_target(data):
  global last_target

  # update the new target
  last_target = data

def compute_n_vel(base_n: float, target_n: float):
  height_diff = target_n - base_n
  vel_n = 0

  if height_diff > 0.1:
    vel_n = clamp(height_diff, min_linear_vel, max_linear_vel)
  elif height_diff < -0.1:
    vel_n = clamp(height_diff, -max_linear_vel, -min_linear_vel)

  return vel_n

def calcular_diferenca_percentual(n: str, base_n: float, target_n: float) -> float:
    diferenca = target_n - base_n

    if base_n != 0:
        diferenca_percentual = (diferenca / abs(base_n)) * 100
    else:
        diferenca_percentual = 0

    rospy.loginfo("%s: Diferença: %s de %s a %s", n, diferenca, base_n, target_n)
    return diferenca_percentual

def compute_linear_vel(base_pose: Pose, target_pose: Pose) -> Vector3:
    linear_vel = Vector3()
    diferenca = Vector3()

    limiar = 0.05*scale

    # se z tiver de valores próximos ao limiar, move x e y
    # se não,
        # se z > +limiar, move z -> x e y 
        # se z < -limiar, move x e y (até x e y chegar ao abs(limiar)) -> x e y

    diferenca.z = target_pose.position.z - base_pose.position.z
    diferenca.x = target_pose.position.x - base_pose.position.x 
    diferenca.y = target_pose.position.y - base_pose.position.y

    limiar_altura = 0.5
    limiar = 0.1
    #por função calibrar limiares

    # Todos os valores próximos ao limiar
    if abs(diferenca.z)<=limiar_altura and abs(diferenca.x)<=limiar and abs(diferenca.y)<=limiar:
        linear_vel.z = 0
        linear_vel.x = 0
        linear_vel.y = 0
        rospy.loginfo("Caso 1: Todos os valores próximos: %s %s %s", diferenca.z, diferenca.x, diferenca.y)
    # Se não, altura maior que o +limiar
    elif diferenca.z>limiar_altura:
        linear_vel.z = compute_n_vel(base_pose.position.z, target_pose.position.z)
        linear_vel.x = 0
        linear_vel.y = 0
        rospy.loginfo("Caso 2: Altura maior que o limiar: %s %s %s", diferenca.z, diferenca.x, diferenca.y)
    # Se não, altura menor que o -limiar
    elif diferenca.z<-limiar_altura:
       # Se diferença de x e y não atingidos, diferença x ou de y > limiar, calcula x e y
        if abs(diferenca.x)>limiar or abs(diferenca.y)>limiar:
            linear_vel.z = 0
            linear_vel.x = compute_n_vel(base_pose.position.x, target_pose.position.x)
            linear_vel.y = compute_n_vel(base_pose.position.y, target_pose.position.y)
            rospy.loginfo("Caso 3.1: Altura menor que o limiar negativo, x ou y>limiar: %s %s %s", diferenca.z, diferenca.x, diferenca.y)
        # Se x e y atingidos, calcula z.
        else:
            linear_vel.z = compute_n_vel(base_pose.position.z, target_pose.position.z)
            linear_vel.x = 0
            linear_vel.y = 0
            rospy.loginfo("Caso 3.1: Altura menor que o limiar negativo, x ou y<limiar: %s %s %s", diferenca.z, diferenca.x, diferenca.y)
    # Se não, altura entre -limiar e +limiar, calcula x e y.
    elif diferenca.z<limiar_altura and diferenca.z>=0:
        linear_vel.z = 0
        linear_vel.x = compute_n_vel(base_pose.position.x, target_pose.position.x)
        linear_vel.y = compute_n_vel(base_pose.position.y, target_pose.position.y)
        rospy.loginfo("Caso 3: Altura próximo ao limiar: %s %s %s", diferenca.z, diferenca.x, diferenca.y)
       

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
    rospy.loginfo("[Pose: %s, Target: %s]", last_pose, last_target)
    rospy.loginfo("-------------------------------------")

    # compute cmd vel
    cmd = compute_cmd_vel()

    # publish cmd vel
    cmd_vel_pub.publish(cmd)

    # sleep
    update_rate.sleep()

if __name__ == '__main__':
  runner()

