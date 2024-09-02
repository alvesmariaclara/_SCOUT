#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped, Vector3

# Variáveis globais
pose_list = [
]

last_pose = Pose()
scale = 1.0

def callback_current_pose(data):
    global last_pose
    last_pose = data.pose

def callback_new_pose(data):
    # Adiciona a nova pose à lista
    global pose_list
    pose_list.append(data.pose)

def update_target(base_pose: Pose, target_pose: Pose) -> bool:
    diferenca = Vector3()
    limiar = 0.05 * scale

    diferenca.z = target_pose.position.z - base_pose.position.z
    diferenca.x = target_pose.position.x - base_pose.position.x
    diferenca.y = target_pose.position.y - base_pose.position.y

    limiar_altura = 0.5
    limiar = 1.0

    rospy.loginfo("Base pose: %s %s %s", base_pose.position.x, base_pose.position.y, base_pose.position.z,)
    rospy.loginfo("Target: %s %s %s", target_pose.position.x, target_pose.position.y, target_pose.position.z)
    rospy.loginfo("Diferença: %s %s %s", diferenca.x, diferenca.y, diferenca.z)

    #valores próximos ao limiar
    if abs(diferenca.z) <= limiar_altura and abs(diferenca.x) <= limiar and abs(diferenca.y) <= limiar:
        return True

    return False

def publish_pose_list():
    rospy.init_node("pose_navigator", anonymous=True)

    rospy.Subscriber("/ground_truth_to_tf/pose", PoseStamped, callback_current_pose)
    rospy.Subscriber("/pose_topic", PoseStamped, callback_new_pose)

    pub = rospy.Publisher("/drone/target", Pose, queue_size=10)

    rate = rospy.Rate(10) 

    current_index = 0
    while not rospy.is_shutdown():
        #verifica se ainda há poses na lista
        if current_index >= len(pose_list):
            rospy.loginfo("Todas as poses da lista foram alcançadas.")
            continue

        target_pose = pose_list[current_index]
        #verifica se a pose deve ser atualizada
        if update_target(last_pose, target_pose):
            rospy.loginfo("Atualizando pose...")
            current_index += 1
            continue

        pub.publish(target_pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_pose_list()
    except rospy.ROSInterruptException:
        pass
