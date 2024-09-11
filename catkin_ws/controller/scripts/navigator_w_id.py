#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped, Vector3
from system.msg import PoseArrayWithID

last_pose = Pose()

pose_list = []  # Lista de poses para navegar
current_id = None  # ID do último conjunto de poses processado

current_index = 0

scale = 1.0

def callback_current_pose(data):
    global last_pose
    # Atualiza a última pose recebida
    last_pose = data.pose

def callback_pose_array(data):
    global pose_list, current_id
    # Verifica se o ID recebido é diferente do ID atual
    if data.id != current_id:
        # Atualiza o ID atual e a lista de poses
        current_id = data.id
        pose_list.extend(data.poses)
        rospy.loginfo("Atualizado com novas poses. ID: %s", current_id)

def update_target(base_pose: Pose, target_pose: Pose) -> bool:
    diferenca = Vector3()

    limiar = 0.05*scale

    diferenca.z = target_pose.position.z - base_pose.position.z
    diferenca.x = target_pose.position.x - base_pose.position.x 
    diferenca.y = target_pose.position.y - base_pose.position.y

    limiar_altura = 0.5
    limiar = 1.0

    #rospy.loginfo("Base pose: %s %s %s", base_pose.position.z, base_pose.position.x, base_pose.position.y)
    rospy.loginfo("Target: %s %s %s", target_pose.position.z, target_pose.position.x, target_pose.position.y)
    #rospy.loginfo("Diferença: %s %s %s", diferenca.z, diferenca.x, diferenca.y)

    # Todos os valores próximos ao limiar
    if abs(diferenca.z)<=limiar_altura and abs(diferenca.x)<=limiar and abs(diferenca.y)<=limiar:
        return True

    return False

def publish_pose_list():
    global pose_list, current_id, current_index  # Declare todas as variáveis globais que você pretende modificar

    rospy.init_node("pose_navigator", anonymous=True)

    # Assina os tópicos necessários
    rospy.Subscriber("/ground_truth_to_tf/pose", PoseStamped, callback_current_pose)
    rospy.Subscriber("pose_array_topic", PoseArrayWithID, callback_pose_array)

    pub = rospy.Publisher("/drone/target", Pose, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Verifica se ainda há poses na lista
        if current_index >= len(pose_list):
            rospy.loginfo("Todas as poses da lista foram alcançadas.")
            continue  # Volta para o início da lista

        if not pose_list:
            # Se a lista de poses estiver vazia, aguarde novas poses
            rospy.loginfo("Aguardando novas poses...")
            rate.sleep()
            continue
        
        target_pose = pose_list[current_index]
        # Verifica se a pose deve ser atualizada
        if update_target(last_pose, target_pose):
            rospy.loginfo("%d", current_index)
            rospy.loginfo("Last pose: (%f, %f, %f)", last_pose.position.x, last_pose.position.y, last_pose.position.z)
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
