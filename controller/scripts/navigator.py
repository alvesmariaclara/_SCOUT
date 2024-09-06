#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped, Vector3
from std_msgs.msg import Bool

# Variáveis globais
pose_list = []
publishing = False  # Flag para indicar se estamos publicando a lista de poses

last_pose = Pose()
scale = 1.0

def callback_current_pose(data):
    global last_pose
    last_pose = data.pose

def callback_new_pose(data):
    global pose_list
    # Adiciona a nova pose à lista
    pose_list.append(data.pose)

def callback_publishing_flag(data):
    global publishing
    publishing = data.data  # Atualiza o estado da flag de publicação

def update_target(base_pose: Pose, target_pose: Pose) -> bool:
    diferenca = Vector3()
    limiar = 0.05 * scale

    diferenca.z = target_pose.position.z - base_pose.position.z
    diferenca.x = target_pose.position.x - base_pose.position.x
    diferenca.y = target_pose.position.y - base_pose.position.y

    limiar_altura = 0.5
    limiar = 1.0

    rospy.loginfo("Base pose: %s %s %s", base_pose.position.x, base_pose.position.y, base_pose.position.z)
    rospy.loginfo("Target: %s %s %s", target_pose.position.x, target_pose.position.y, target_pose.position.z)
    rospy.loginfo("Diferença: %s %s %s", diferenca.x, diferenca.y, diferenca.z)

    # Valores próximos ao limiar
    if abs(diferenca.z) <= limiar_altura and abs(diferenca.x) <= limiar and abs(diferenca.y) <= limiar:
        return True

    return False

def publish_pose_list():
    global publishing  # Indica que estamos usando a variável global 'publishing'

    rospy.init_node("pose_navigator", anonymous=True)

    rospy.Subscriber("/ground_truth_to_tf/pose", PoseStamped, callback_current_pose)
    rospy.Subscriber("/pose_topic", PoseStamped, callback_new_pose)
    rospy.Subscriber("/publishing_flag", Bool, callback_publishing_flag)

    pub = rospy.Publisher("/drone/target", Pose, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz

    current_index = 0
    while not rospy.is_shutdown():
        # Verifica se ainda há poses na lista
        if current_index >= len(pose_list):
            rospy.loginfo("Todas as poses da lista foram alcançadas.")
            continue

        if not publishing:
            rospy.loginfo("Aguardando novas poses para atualizar a lista.")
            rate.sleep()
            continue

        target_pose = pose_list[current_index]
        # Verifica se a pose deve ser atualizada
        if update_target(last_pose, target_pose):
            rospy.loginfo("Atualizando pose...")
            current_index += 1
            if current_index >= len(pose_list):
                rospy.loginfo("Todas as poses da lista foram alcançadas.")
                publishing = False  # Para a publicação quando todas as poses foram alcançadas
            continue

        pub.publish(target_pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        # Aguarda para garantir que o ROS esteja completamente inicializado antes de iniciar
        rospy.sleep(1)
        publish_pose_list()
    except rospy.ROSInterruptException:
        pass
