#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

# Função callback para o tópico de PoseStamped
def pose_callback(msg):
    rospy.loginfo("Pose recebida: %s", msg)

# Função callback para o tópico de Bool (flag)
def flag_callback(msg):
    if msg.data:
        rospy.loginfo("Flag recebida: True (Início da publicação)")
    else:
        rospy.loginfo("Flag recebida: False (Término da publicação)")

def listener():
    # Inicializa o nó ROS
    rospy.init_node('pose_listener', anonymous=True)

    # Inscreve-se nos tópicos
    rospy.Subscriber('pose_topic', PoseStamped, pose_callback)
    rospy.Subscriber('publishing_flag', Bool, flag_callback)

    # Mantém o nó ativo para escutar as mensagens
    rospy.loginfo("Iniciando o listener. Escutando tópicos...")
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
