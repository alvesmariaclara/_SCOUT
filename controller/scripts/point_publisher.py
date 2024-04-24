#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
import sys

def read_map_points(file_path):
    points = []
    with open(file_path, 'r') as file:
        for line in file:
            values = line.strip().split()
            if len(values) == 7:
                points.append([float(v) for v in values])
    return points

def publish_target(points):
    rospy.init_node('target_publisher', anonymous=True)
    target_pub = rospy.Publisher('/drone/target', Pose, queue_size=10)
    rate = rospy.Rate(1)
    
    for point in points:
        #target_pose é um objeto de mensagem do tipo geometry_msgs/Pose
        target_pose = Pose()
        
        #definindo os valores de posição e orientação
        target_pose.position.x = point[0]
        target_pose.position.y = point[1]
        target_pose.position.z = point[2]
        target_pose.orientation.x = point[3]
        target_pose.orientation.y = point[4]
        target_pose.orientation.z = point[5]
        target_pose.orientation.w = point[6]
        
        #publicando no tópico '/drone/target'
        target_pub.publish(target_pose)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        if len(sys.argv) == 2:
            file_path = sys.argv[1]
            map_points = read_map_points(file_path)
            
            #decolagem_point = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            #pouso_point = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
            #map_points.append(decolagem_point)
            #map_points.append(pouso_point)
            
            publish_target(map_points)
        else:
            print("Usage: rosrun my_publisher_pkg target_publisher.py map_points.txt")
            sys.exit(1)
    except rospy.ROSInterruptException:
        pass
