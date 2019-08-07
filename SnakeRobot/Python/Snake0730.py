# -*- coding:utf-8 -*-
# 时间：2019/7/30 11:31
# 作者：Eowyn_Du
# @file: test.py
# @software: PyCharm
# 描述：本文件用于控制V-REP的SnakeRobot.ttt文件中蛇形机器人运动


# 导入所需库文件
from __future__ import division
import numpy as np
import math
import vrep
import controlFunction
import pandas as pd
from snakeClass import ParaAverageValue, ParaSample
import time

# import os
# os.environ["CUDA_VISIBLE_DEVICES"] = "0"

start_time = time.time()
# 用于弧度/角度互转
RAD2DEG = 180/math.pi  # 弧度转角度
DEG2RAD = math.pi/180  # 角度转弧度

# 仿真参数设置
simStep = 0.01         # 定义仿真步长 5ms循环一次
simDiscreteTime = 0     # 定义离散时间 0,1,2,...


# 1. 配置蛇形机器人基本参数

# *几何参数：关节、模块数量/名称
# **关节数量（俯仰、偏航各5个）
jointNum = 8
# **模块数量=关节数量+1
moduleNum = jointNum+1
# **模块名称
baseName = 'ACMR'
vJointName = 'ACMR_vJoint'
hJointName = 'ACMR_hJoint'
moduleName = 'ACMR_body#'


# 2. 定义存储实验中产生的数据：速度、位置、关节角度等变量
# --定义用于存储的表格形式
data = []
# 表头：第一行
data.append(('VX', 'VY', 'VZ', "J0", "J1", "J2", "J3", "J4", "J5", "J6", "J7", "AverageVelX"))

# --速度
# **定义采样平均的基数，采样N次进行平均
N = 10
# **每个模块速度的 N 次采样记录
velSample = ParaSample(moduleNum, N)

# velSampleX, velSampleY, velSampleZ = np.zeros(moduleNum, N)
# **位置的 N 次采样记录
posSample = ParaSample(moduleNum, N)

# posSampleX, posSampleY, posSampleZ = np.zeros(moduleNum, N)

# **速度： N 次采样平均值
velAverageValue = ParaAverageValue(moduleNum)
# velAverageX, velAverageY, velAverageZ = 0

# --关节角度
# __Set：用于控制
vJointSet = np.zeros((jointNum,), dtype=np.float)  # 关节角vJoint设定值
hJointSet = np.zeros((jointNum,), dtype=np.float)  # 关节角hJoint设定值
# __Sensor：用于检测
vJointPositionSensor = np.zeros((jointNum,))
hJointPositionSensor = np.zeros((jointNum,))
modulePositionSensor = np.zeros((jointNum,3), dtype=np.float)

# 3. 定义控制器参数
# --蛇形曲线幅度
A = np.zeros((jointNum,), dtype=np.float)
# --蛇形曲线关节角度相位差
B = np.zeros((jointNum,), dtype=np.float)
# --蛇形曲线左右转弯控制参数
phi = np.zeros((jointNum,), dtype=np.float)
# --蛇形曲线频率——角速度
omega = np.zeros((jointNum,), dtype=np.float)

# 赋初值
A[:] = -math.pi / 4
B[:] = 0  # math.pi / 10
phi[:] = 0.4  # math.pi/6
omega[:] = 1


# 4. 初始化V-REP环境
print('Program started')
# --关闭潜在的连接
vrep.simxFinish(-1)
# 每隔0.2s检测一次，直到连接上V-REP
while True:
    clientID = vrep.simxStart('127.0.0.1',19999,True,True,5000,5)
    if clientID > -1:
        break
    else:
        time.sleep(2)
        print("Failed connecting to remote API server!")
print("Connection success!")

# --配置V-REP模式
# 设置仿真步长，为了保持API端与V-REP端相同步长
vrep.simxSetFloatingParameter(clientID, vrep.sim_floatparam_simulation_time_step, simStep, vrep.simx_opmode_oneshot)
# 然后打开同步模式
vrep.simxSynchronous(clientID, True)
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

# 然后读取Base和Joint的句柄
vJointHandle = np.zeros((jointNum,), dtype=np.int)  # 注意是整型
hJointHandle = np.zeros((jointNum,), dtype=np.int)  # 注意是整型
moduleHandle = np.zeros((moduleNum,), dtype=np.int)  # 注意是整型

for i in range(0, jointNum-1):
    _, returnHandle = vrep.simxGetObjectHandle(clientID, vJointName + str(i), vrep.simx_opmode_blocking)
    vJointHandle[i] = returnHandle
    print("vJointHandle[", i, "]")
    print(vJointHandle[i])

for i in range(0, jointNum-1):
    _, returnHandle = vrep.simxGetObjectHandle(clientID, hJointName + str(i), vrep.simx_opmode_blocking)
    hJointHandle[i] = returnHandle
    print(hJointHandle[i])

for i in range(0, moduleNum-1):
    _, returnHandle = vrep.simxGetObjectHandle(clientID, moduleName + str(i), vrep.simx_opmode_blocking)
    moduleHandle[i] = returnHandle
    print(moduleHandle[i])

_, baseHandle = vrep.simxGetObjectHandle(clientID, baseName, vrep.simx_opmode_blocking)

print("Handles avaliable!")

# 然后首次读取关节的初始值，以streaming的形式
for i in range(0, jointNum-1):
    _, jpos = vrep.simxGetJointPosition(clientID, vJointHandle[i], vrep.simx_opmode_streaming)
    vJointPositionSensor[i] = jpos


for i in range(0, jointNum-1):
    _, jpos = vrep.simxGetJointPosition(clientID, hJointHandle[i], vrep.simx_opmode_streaming)
    hJointPositionSensor[i] = jpos

# Simulation
lastCmdTime = vrep.simxGetLastCmdTime(clientID)  # 记录当前时间
vrep.simxSynchronousTrigger(clientID)  # 让仿真走一步
# 开始仿真
steps = 0
while vrep.simxGetConnectionId(clientID) != -1 and steps <= 2000:
    currCmdTime = vrep.simxGetLastCmdTime(clientID)  # 记录当前时间
    # strCmdTime = currCmdTime # 记录初始时间
    dt = currCmdTime - lastCmdTime # 记录时间间隔，用于控制

    # ---控制部分
    # 读取当前的状态值，之后都用buffer形式读取

    # 读取关节角（传感器返回值）
    for i in range(0, jointNum-1):
        _, jpos = vrep.simxGetJointPosition(clientID, vJointHandle[i], vrep.simx_opmode_buffer)
        vJointPositionSensor[i] = jpos
    for i in range(0, jointNum-1):
        _, jpos = vrep.simxGetJointPosition(clientID, hJointHandle[i], vrep.simx_opmode_buffer)
        hJointPositionSensor[i] = jpos

    # 控制命令需要同时发生，故暂停通信，用于存储所有控制命令一起发送

    vrep.simxPauseCommunication(clientID, True)


# 设置蛇形机器人控制器参数
    if steps % 50 == 0:
        print("steps = ", steps)
        print("velAverageValue.x = ", velAverageValue.x[0])
        print("omega: ", omega)
        if velAverageValue.x[0] >= 1:
            omega = omega - math.pi/12
        else:
            omega = omega + math.pi/12

    simDiscreteTime += dt * simStep
    for i in range(0, jointNum-1):
        # 将控制器参数转化为关节角
        vJointSet[i] = controlFunction.sin_controller(i, A[i], omega[i], B[i], phi[i], simDiscreteTime)
        vrep.c_SetJointTargetPosition(clientID, vJointHandle[i], vJointSet[i], vrep.simx_opmode_oneshot)

    # 读取各个模块的速度及位置
    for i in range(0, moduleNum-1):
        _, jpos = vrep.simxGetObjectPosition(clientID, moduleHandle[i], -1, vrep.simx_opmode_oneshot)
        modulePositionSensor[i,:] = jpos
        # print("modulePosition[",i,"] = ", modulePositionSensor[i])
    _, vel, ang = vrep.simxGetObjectVelocity(clientID, moduleHandle[0], vrep.simx_opmode_oneshot)
    # print("velocity = ", vel)
    data.append((vel[0], vel[1], vel[2], vJointSet[0], vJointSet[1], vJointSet[2],
                 vJointSet[3], vJointSet[4], vJointSet[5], vJointSet[6],
                 vJointSet[7], velAverageValue.x[0]))
    # 当前速度
    vc = vel[0]*vel[0]+vel[1]*vel[1]
    vc = math.sqrt(vc)
    # 求N次采样的平均速度
    # velSample.x = velSample.x*(steps-1)/steps+(1/steps)*vc

    # 求近10次采样平均
    if steps < 10:
        velSample.x[0, steps] = vc
    else:
        for i in range(1, 9):
            velSample.x[0, i-1] = velSample.x[0, i]
        velSample.x[0, 9] = vc

    velAverageValue.x[0] = sum(velSample.x[0, :])/10

    vrep.c_SetJointTargetPosition(clientID, vJointHandle[0], vJointSet[0], vrep.simx_opmode_oneshot)
    vrep.simxPauseCommunication(clientID, False)
    df = pd.DataFrame(data)
    df.to_excel('Velocity.xlsx')

    steps += 1

    # ---控制部分结束
    lastCmdTime = currCmdTime  # 记录当前时间
    vrep.simxSynchronousTrigger(clientID)  # 进行下一步
    vrep.simxGetPingTime(clientID)  # 使得该仿真走完

end_time = time.time()
print("程序运行时间：%.8s s" % (end_time-start_time))
