#!/usr/bin/env python3

import rospy
import heapq
import math
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from std_msgs.msg import Header
import time

#converter de Pose para coordenadas (x, y, z)
def pose_para_coordenadas(pose):
    return (pose.position.x, pose.position.y, pose.position.z)

#converter de coordenadas (x, y, z) para Pose
def coordenadas_para_pose(x, y, z):
    return Pose(position=Point(x, y, z), orientation=Quaternion(0, 0, 0, 1))

#global de obstáculos (como coordenadas para hash)
obstaculos = [
    pose_para_coordenadas(Pose(position=Point(1, 1, 1), orientation=Quaternion(0, 0, 0, 1))),
    pose_para_coordenadas(Pose(position=Point(2, 2, 2), orientation=Quaternion(0, 0, 0, 1))),
    pose_para_coordenadas(Pose(position=Point(1, 2, 1), orientation=Quaternion(0, 0, 0, 1)))
]

#calcular a heurística (distância euclidiana) entre dois nós (como coordenadas)
def heuristica_euclidiana(no_atual, no_objetivo):
    dx = no_atual[0] - no_objetivo[0]
    dy = no_atual[1] - no_objetivo[1]
    dz = no_atual[2] - no_objetivo[2]
    return math.sqrt(dx**2 + dy**2 + dz**2)

# calcular a heurística (distância de Manhattan) entre dois nós (como coordenadas)
def heuristica_manhattan(no_atual, no_objetivo):
    dx = abs(no_atual[0] - no_objetivo[0])
    dy = abs(no_atual[1] - no_objetivo[1])
    dz = abs(no_atual[2] - no_objetivo[2])
    return dx + dy + dz

#algoritmo A*
def a_estrela(inicio, objetivo, vizinhos_funcao):
    aberta = []
    fechada = set(obstaculos)
    heapq.heappush(aberta, (0, inicio))
    g_custo = {inicio: 0}
    caminho = {inicio: None}

    while aberta:
        _, no_atual = heapq.heappop(aberta)
        if no_atual == objetivo:
            caminho_final = []
            while no_atual:
                caminho_final.append(no_atual)
                no_atual = caminho[no_atual]
            return [coordenadas_para_pose(x, y, z) for (x, y, z) in caminho_final[::-1]]

        fechada.add(no_atual)
        for vizinho in vizinhos_funcao(no_atual):
            if vizinho in fechada:
                continue
            g_vizinho = g_custo[no_atual] + heuristica_euclidiana(no_atual, vizinho)
            if vizinho not in g_custo or g_vizinho < g_custo[vizinho]:
                g_custo[vizinho] = g_vizinho
                f_vizinho = g_vizinho + heuristica_euclidiana(vizinho, objetivo)
                heapq.heappush(aberta, (f_vizinho, vizinho))
                caminho[vizinho] = no_atual

    return None

#obter os vizinhos de um nó (como coordenadas)
def obter_vizinhos(no):
    x, y, z = no
    return [
        (x + 1, y, z), (x - 1, y, z),
        (x, y + 1, z), (x, y - 1, z),
        (x, y, z + 1), (x, y, z - 1)
    ]

#para nós-alvo
def calcular_trajetorias(nos_alvo, vizinhos_funcao):
    trajetorias = []
    lista_continua = []
    tempo_total = 0.0
    tamanho_total = 0

    for i in range(len(nos_alvo) - 1):
        inicio = pose_para_coordenadas(nos_alvo[i])
        objetivo = pose_para_coordenadas(nos_alvo[i + 1])
        start_time = time.time()
        caminho = a_estrela(inicio, objetivo, vizinhos_funcao)
        tempo_processamento = time.time() - start_time

        if caminho:
            trajetorias.append((caminho, len(caminho), tempo_processamento))
            lista_continua.extend(pose_para_coordenadas(p) for p in caminho[1:])
            tamanho_total += len(caminho)
        else:
            print(f"Nenhum caminho encontrado entre {nos_alvo[i]} e {nos_alvo[i + 1]}.")
            trajetorias.append((None, 0, tempo_processamento))

        tempo_total += tempo_processamento

    ##converte a lista contínua de coordenadas para uma lista de Poses
    lista_continua_poses = [coordenadas_para_pose(x, y, z) for (x, y, z) in lista_continua]

    return trajetorias, lista_continua_poses, tempo_total, tamanho_total

#publicar uma lista de poses em um tópico ROS
def publish_pose_list(pose_list):
    pub = rospy.Publisher('pose_topic', PoseStamped, queue_size=10)
    rospy.init_node('pose_publisher', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz
    for pose in pose_list:
        #pub.publish(pose)
        print(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        nos_alvo = [
            coordenadas_para_pose(0, 0, 0),
            coordenadas_para_pose(3, 3, 3),
            coordenadas_para_pose(17, 17, 17),
            coordenadas_para_pose(20, 20, 20)
        ]

        trajetorias, lista_continua, tempo_total, tamanho_total = calcular_trajetorias(nos_alvo, obter_vizinhos)

        publish_pose_list(lista_continua)

        print("Trajetórias encontradas:")
        for i, (trajeto, tamanho, tempo) in enumerate(trajetorias):
            if trajeto:
                print(f"Trajetória {i + 1}: {[pose_para_coordenadas(p) for p in trajeto]}")
                print(f"Tamanho da trajetória: {tamanho} nós")
            else:
                print(f"Trajetória {i + 1}: Nenhum caminho encontrado")
            print(f"Tempo de processamento: {tempo:.4f} segundos\n")

        print(f"Lista contínua de coordenadas: {lista_continua}\n")
        print(f"Tamanho total da trajetória contínua: {len(lista_continua)} nós")
        print(f"Tempo total de processamento: {tempo_total:.4f} segundos\n")

    except rospy.ROSInterruptException:
        pass