#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Twist

def read_data():
    with open("data.txt", "r") as file:
        data = file.read().strip().split(',')
        if len(data) != 6:
            return None
        else:
            pose = PoseStamped()
            pose.pose.position.x = float(data[0])
            pose.pose.position.y = float(data[1])
            pose.pose.position.z = float(data[2])
            pose.pose.orientation.x = float(data[3])
            pose.pose.orientation.y = float(data[4])
            pose.pose.orientation.z = float(data[5])
            return pose

def runner():
    rospy.init_node('data_updater', anonymous=True)

    updated_pose_pub = rospy.Publisher("/updated_pose", PoseStamped, queue_size=10)
    update_rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        pose_data = read_data()
        if pose_data is not None:
            updated_pose_pub.publish(pose_data)
            update_rate.sleep()

if __name__ == '__main__':
    try:
        runner()
    except rospy.ROSInterruptException:
        pass

