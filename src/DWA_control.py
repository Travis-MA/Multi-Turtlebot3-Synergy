#!/usr/bin/env python
#coding=utf-8

description = """
Authors: Siqing MA
Date: 2018-9-1 created, 2018-9-16 updated
Turtlebot3 Swarm avoid a few obstacle with Dynamical Window Approach
<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<Performance Edition>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
"""

import tf
import sys
import math
import rospy
import roslib
import numpy as np
import turtlesim.srv
import turtlesim.msg
import geometry_msgs.msg
from sensor_msgs.msg import LaserScan

roslib.load_manifest('multi_turtlebot_opt')

u = np.array([0.2, 0.0]) #初始化速度空间
x = np.array([0.0, 0.0, math.pi / 2.0, 0.2, 0.0]) #初始化位置空间
trajectory = np.array(x) #轨迹
goal = np.array([0,0]) #目标点Pose
obstacle_pose = np.matrix([[0,10000]]) #障碍物空间

class Config(object):
    '''
    参数
    '''
    def __init__(self):
        self.max_linear = 0.22   #[m/s]  #turtlebot最大线速度（确定）
        self.min_speed = 0  # [m/s]  #turtlebot最小线速度
        self.max_accel = 0.2  # [m/ss]  # 最大加速度
        self.max_angular = 2.84 * math.pi / 180.0  #[rad/s]  #turtlebot最大角速度
        self.max_dyawrate = 6.0 * math.pi / 180.0  # [rad/ss]  # 最大角加速度

        self.v_reso = 0.04  # [m/s]，速度迭代分辨率
        self.yawrate_reso = 0.2 * math.pi / 180.0  # [rad/s]，角速度迭代分辨率
        
        self.trigger_distance = 0.8 #[m]  #触发壁障处理阈值
        self.follow_dist = 0.9 #[m]  #跟随距离
        self.dt = 0.1  # [s]  # 采样周期
        self.predict_time = 3.0  # [s]  # 向前预估秒数
        self.robot_radius = 0.1  # [m]  # 机器人半径
        self.scan_range = 180 #[radius] 雷达扫描角度（没什么用不用调）

        self.SIGMA = 1.0
        self.ALPHA = 0.5
        self.BETA = 0.2
        self.GAMMA = 0.3
        
        
def motion(x, u, dt):
    """
    更新位置空间
    :param x: 位置空间（直角坐标）
    :param u: 速度空间（线速度和角速度）
    :param dt: 采样时间
    :return:
    """
    # 简单化表示，使用直线非圆弧
    x[0] += u[0] * math.cos(x[2]) * dt  # x方向位移
    x[1] += u[0] * math.sin(x[2]) * dt  # y
    x[2] += u[1] * dt  # 航向角
    x[3] = u[0]  # 速度v
    x[4] = u[1]  # 角速度w

    return x


def calc_dynamic_window(x, config):
    """
    计算动态窗口（动态窗口表示速度空间中可实现的速度的子集）
    参考论文 'The Dynamic Window Approach to Collision Avoidence' 1997

    :param x:当前位置空间
    :param config:
    :return:两个速度的交集
    """

    # 车辆能够达到的最大最小速度
    vs = [config.min_speed, config.max_linear,
          -config.max_angular, config.max_angular]

    # 一个采样周期能够变化的最大最小速度
    vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_dyawrate * config.dt,
          x[4] + config.max_dyawrate * config.dt]
   
    # 求出两个速度集合的交集
    vr = [max(vs[0], vd[0]), min(vs[1], vd[1]),
          max(vs[2], vd[2]), min(vs[3], vd[3])]
    
    return vr


def calc_trajectory(x_init, v, w, config):
    """
    预测轨迹
    :param x_init:位置空间
    :param v:速度
    :param w:角速度
    :return: 轨迹堆叠向量（采样）
    """
    x = np.array(x_init)
    trajectory = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, w], config.dt)
        trajectory = np.vstack((trajectory, x))
        time += config.dt

    return trajectory


def calc_to_goal_cost(trajectory, goal, config):
    """
    计算Goal cost
    :param trajectory:轨迹搜索空间
    :param goal: 目标点位置
    :return: 轨迹到目标点距离
    """
    #最终位置与目标点的差值
    dx = goal[0] - trajectory[-1, 0] 
    dy = goal[1] - trajectory[-1, 1]
    goal_dis = math.sqrt(dx ** 2 + dy ** 2)
    

    return goal_dis


def calc_obstacle_cost(traj, ob, config):
    """
    计算Dist cost
    :param traj: 轨迹向量
    :param ob: 障碍物向量
    :return: 1/到障碍物的最小距离
    """
    min_r = float("inf")

    for ii in range(0, len(traj[:, 1])):
        for i in range(len(ob[:, 0])):
            ox = ob[i, 0]
            oy = ob[i, 1]
            dx = traj[ii, 0] - ox
            dy = traj[ii, 1] - oy

            r = math.sqrt(dx ** 2 + dy ** 2)
            if r <= config.robot_radius: 
                return float("inf")  

            if min_r >= r:
                min_r = r

    return 1.0 / min_r  # 最小距离越大越好


def calc_final_input(x, u, vr, config, goal, ob):
    """
    计算final cost function，选择最优
    :param x:位置空间
    :param u:速度空间
    :param vr:速度空间交集
    :param config:
    :param goal:目标位置
    :param ob:障碍物
    :return:
    """
    x_init = x[:]
    min_cost = 10000.0
    min_u = u
    best_trajectory = np.array([x])
    heading_sum = 0
    vel_sum = 0
    dist_sum = 0
    heading_array = []
    dist_array = []
    vel_array = []

    for v in np.arange(vr[0], vr[1], config.v_reso):
        for w in np.arange(vr[2], vr[3], config.yawrate_reso):

            trajectory = calc_trajectory(x_init, v, w, config)
            # calc cost
            heading = calc_to_goal_cost(trajectory, goal, config)
            vel = config.max_linear - trajectory[-1, 3]
            dist = calc_obstacle_cost(trajectory, ob, config)

            heading_array.append(heading)
            dist_array.append(dist)
            vel_array.append(vel)

            heading_sum += heading
            vel_sum += vel
            dist_sum += dist
           
            
    times = 0
    for v in np.arange(vr[0], vr[1], config.v_reso):
        for w in np.arange(vr[2], vr[3], config.yawrate_reso):

            heading = heading_array[times]
            dist = dist_array[times]
            vel = vel_array[times]
           
            #对各项进行normalize
            heading_norm = heading/heading_sum
            vel_norm = vel/vel_sum
            dist_norm = dist/dist_sum

            # 评价函数
            final_cost = config.SIGMA * (config.ALPHA * heading_norm + config.BETA * dist_norm + config.GAMMA * vel_norm)
            
            if min_cost >= final_cost:
                min_cost = final_cost
                min_u = [v, w]
                best_trajectory = trajectory

            times = times+1

    return min_u, best_trajectory


def dwa_control(x, u, config, goal, ob):
    """
    调用前面的几个函数，生成最合适的速度空间和轨迹搜索空间
    :param x:
    :param u:
    :param config:
    :param goal:
    :param ob:
    :return:
    """
    # Dynamic Window control

    vr = calc_dynamic_window(x, config)

    u, trajectory = calc_final_input(x, u, vr, config, goal, ob)

    return u, trajectory


def obstacle_space(ranges, config):
    """
    生成障碍物空间
    :param ranges 激光雷达返回的原始数组
    :param config
    """
    
    obstacle_pose_list = [[0,10000]]
    scan_range = config.scan_range
    for i in range(scan_range):
        if i <= scan_range / 2:
            dr = i
        else:
            dr = 360 - scan_range + i

        length = ranges[dr]
        if length < config.trigger_distance: #过滤超长的扫描数据
            
            dr = math.radians(dr) #dr已转弧度
            x = -math.sin(dr) * length
            y = math.cos(dr) * length
            
            obstacle_pose_list.append([x, y])

    ob =  np.matrix(np.array(obstacle_pose_list))
    return ob


def scan_handler(scan):
    """
    读取激光雷达信息并写入障碍物空间
    :param scan topic：scan
    """
    config = Config()
    global obstacle_pose
    ranges = scan.ranges
    obstacle_pose = obstacle_space(ranges, config) #写入障碍物空间


if __name__ == '__main__':
    print(description)

    config = Config()
    rospy.init_node('follower_control')
    tb3_id = rospy.get_param('~tb3_id')
    goal_id = rospy.get_param('~goal_id')
    mode = rospy.get_param('~mode')
    print("tb3_id: " + tb3_id + " goal_id: " + goal_id)
    
    listener = tf.TransformListener()
    turtlebot_vel = rospy.Publisher(tb3_id + '/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
    
    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown():

        try:
            (trans, rot) = listener.lookupTransform('/'+tb3_id, '/'+goal_id , rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        #更新目标点
        goal = np.array([0.1, 10.1])
        goal[0] = -trans[1] #相对前车x坐标
        goal[1] = trans[0] #相对前车y坐标
        distance = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        #print("goal0: " + bytes(goal[0]) + " goal1: " + bytes(goal[1]))

        if(mode == "follow"):
            if(distance > config.follow_dist):
                #更新障碍物点云   
                rospy.Subscriber(tb3_id +'/scan', LaserScan, scan_handler, queue_size=20)
                u, best_trajectory = dwa_control(x, u, config, goal, obstacle_pose)
                
                #把更新的速度信息发送给turtlebot
                cmd = geometry_msgs.msg.Twist()
                cmd.linear.x = u[0]
                cmd.angular.z = u[1] * 180.0/math.pi
                turtlebot_vel.publish(cmd)
            
            else:
                cmd = geometry_msgs.msg.Twist()
                cmd.linear.x = 0
                cmd.angular.z = 0
                turtlebot_vel.publish(cmd)
        if(mode == "disperse"):
            if(distance > config.robot_radius):
                #更新障碍物点云   
                rospy.Subscriber(tb3_id +'/scan', LaserScan, scan_handler, queue_size=20)
                u, best_trajectory = dwa_control(x, u, config, goal, obstacle_pose)
                
                #把更新的速度信息发送给turtlebot
                cmd = geometry_msgs.msg.Twist()
                cmd.linear.x = u[0]
                cmd.angular.z = u[1] * 180.0/math.pi
                turtlebot_vel.publish(cmd)
            
            else:
                cmd = geometry_msgs.msg.Twist()
                cmd.linear.x = 0
                cmd.angular.z = 0
                turtlebot_vel.publish(cmd)
        
        rate.sleep()
