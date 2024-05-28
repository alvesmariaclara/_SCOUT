#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped, Vector3

last_pose = Pose()
scale = 1.0

def callback_current_pose(data):
  global last_pose
  
  # get the last pose
  last_pose = data.pose
  

def update_target(base_pose: Pose, target_pose: Pose) -> bool:
    linear_vel = Vector3()
    diferenca = Vector3()

    limiar = 0.05*scale

    diferenca.z = target_pose.position.z - base_pose.position.z
    diferenca.x = target_pose.position.x - base_pose.position.x 
    diferenca.y = target_pose.position.y - base_pose.position.y

    limiar_altura = 0.3
    limiar = 0.8

    # Todos os valores próximos ao limiar
    if abs(diferenca.z)<=limiar_altura and abs(diferenca.x) <= limiar and abs(diferenca.y)<=limiar:
        return True

    return False

def publish_pose_list(pose_list):
    rospy.init_node("pose_navigator", anonymous=True)

    rospy.Subscriber("/ground_truth_to_tf/pose", PoseStamped, callback_current_pose)

    pub = rospy.Publisher("/drone/target", Pose, queue_size=10)

    rate = rospy.Rate(10) 

    current_index = 0
    while not rospy.is_shutdown():
        # Verifica se ainda há poses na lista
        if current_index >= len(pose_list):
            rospy.loginfo("Todas as poses da lista foram alcançadas.")
            break

        target_pose = pose_list[current_index]
        # Verifica se a pose deve ser atualizada
        if update_target(last_pose, target_pose):
            rospy.loginfo("Atualizando pose...")
            current_index += 1
            continue

        pub.publish(target_pose)
        rate.sleep()

if __name__ == '__main__':
    pose_list = [
        Pose(position=Point(x=1.0, y=2.0, z=2.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)),
        Pose(position=Point(x=2.0, y=3.0, z=2.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)),
        Pose(position=Point(x=3.0, y=4.0, z=1.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=100)),
        last_pose
    ]

    try:
        publish_pose_list(pose_list)
    except rospy.ROSInterruptException:
        pass
