#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleController:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('turtle_controller', anonymous=True)
        
        # 创建发布者，发布速度指令到/cmd_vel话题
        self.velocity_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # 创建订阅者，订阅海龟当前位置
        self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        
        # 存储当前位置的变量
        self.pose = Pose()
        self.rate = rospy.Rate(10)  # 10Hz
        
        # 目标点坐标
        self.target_x = 4.0
        self.target_y = 4.0

    def update_pose(self, data):
        """回调函数：更新海龟当前位置"""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def calculate_distance(self):
        """计算当前位置到目标点的距离"""
        return math.sqrt(
            (self.target_x - self.pose.x) ** 2 +
            (self.target_y - self.pose.y) ** 2
        )

    def calculate_angle(self):
        """计算朝向目标点需要转动的角度"""
        return math.atan2(
            self.target_y - self.pose.y,
            self.target_x - self.pose.x
        )

    def move_to_goal(self):
        """控制海龟移动到目标点"""
        vel_msg = Twist()
        
        # 设置距离和角度的误差容忍度
        distance_tolerance = 0.1
        angle_tolerance = 0.01
        
        while self.calculate_distance() > distance_tolerance:
            # 计算距离和角度
            distance = self.calculate_distance()
            target_angle = self.calculate_angle()
            angle_diff = target_angle - self.pose.theta
            
            # 角度归一化到[-π, π]
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            # 先调整角度，再前进
            if abs(angle_diff) > angle_tolerance:
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.5 * angle_diff  # 角速度比例控制
            else:
                vel_msg.linear.x = 0.5 * distance  # 线速度比例控制
                vel_msg.angular.z = 0.0
            
            # 发布速度指令
            self.velocity_pub.publish(vel_msg)
            self.rate.sleep()
        
        # 到达目标点后停止
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.velocity_pub.publish(vel_msg)
        rospy.loginfo("已到达目标点!")

if __name__ == '__main__':
    try:
        controller = TurtleController()
        # 等待获取初始位置
        rospy.sleep(1)
        controller.move_to_goal()
    except rospy.ROSInterruptException:
        pass