import numpy as np
import matplotlib.pyplot as plt

# 初始化车的参数
d = 3.5  # 道路标准宽度
W = 1.8  # 汽车宽度
P0 = np.array([0, -d / 2, 1, 1])  # 车辆起点位置，分别代表x,y,vx,vy
Pg = np.array([99, d / 2, 0, 0])  # 目标点位置
Pobs = np.array([[15, 7 / 4, 0, 0],  # 障碍物位置
                 [30, -3 / 2, 0, 0],
                 [45, 3 / 2, 0, 0],
                 [60, -3 / 4, 0, 0],
                 [80, 7 / 4, 0, 0]])
P = np.array([[15, 7 / 4, 0, 0],
              [30, -3 / 2, 0, 0],
              [45, 3 / 2, 0, 0],
              [60, -3 / 4, 0, 0],
              [80, 7 / 4, 0, 0],
              [99, d / 2, 0, 0]])

Eta_att = 5  # 计算引力的增益系数
Eta_rep_ob = 15  # 计算斥力的增益系数
Eta_rep_edge = 50  # 计算边界斥力的增益系数

d0 = 20  # 障碍影响距离
n = len(P)  # 障碍物和目标总计个数
len_step = 0.5  # 步长
Num_iter = 200  # 最大循环迭代次数

Pi = P0
Path = []  # 存储路径
delta = np.zeros((n, 2))  # 存储每一个障碍到车辆的斥力方向向量以及引力方向向量
dist = []  # 存储每一个障碍到车辆的距离以及目标点到车辆距离
unitvector = np.zeros((n, 2))  # 存储每一个障碍到车辆的斥力单位向量以及引力单位向量
F_rep_ob = np.zeros((n - 1, 2))  # 存储每一个障碍到车辆的斥力
F_rep_edge = np.zeros((1, 2))
while ((Pi[0] - Pg[0]) ** 2 + (Pi[1] - Pg[1]) ** 2) ** 0.5 > 1:
    # a = ((Pi[0] - Pg[0]) ** 2 + (Pi[1] - Pg[1]) ** 2) ** 0.5
    Path.append(Pi.tolist())
    # 计算车辆当前位置和障碍物的单位方向向量
    for j in range(len(Pobs)):
        delta[j, :] = Pi[0:2] - P[j, 0:2]
        dist.append(np.linalg.norm(delta[j, :]))
        unitvector[j, :] = [delta[j, 0] / dist[j], delta[j, 1] / dist[j]]
    # 计算车辆当前位置和目标点的单位方向向量
    delta[n - 1, :] = P[n - 1, 0:2] - Pi[0:2]
    dist.append(np.linalg.norm(delta[n - 1, :]))
    unitvector[n - 1, :] = [delta[n - 1, 0] / dist[n - 1], delta[n - 1, 1] / dist[n - 1]]

    # 计算斥力
    for j in range(len(Pobs)):
        if dist[j] >= d0:
            F_rep_ob[j, :] = [0, 0]
        else:
            # 障碍物的斥力1,方向由障碍物指向车辆
            F_rep_ob1_abs = Eta_rep_ob * (1 / dist[j] - 1 / d0) * dist[n - 1] / dist[j] ** 2  # 回看公式设定n=1
            F_rep_ob1 = np.array([F_rep_ob1_abs * unitvector[j, 0], F_rep_ob1_abs * unitvector[j, 1]])
            # 障碍物的斥力2，方向由车辆指向目标点
            F_rep_ob2_abs = 0.5 * Eta_rep_ob * (1 / dist[j] - 1 / d0) ** 2
            F_rep_ob2 = np.array([F_rep_ob2_abs * unitvector[n - 1, 0], F_rep_ob2_abs * unitvector[n - 1, 1]])
            # 改进后的障碍物和斥力计算
            F_rep_ob[j, :] = F_rep_ob1 + F_rep_ob2
    # 增加的边界斥力
    if -d + W / 2 < Pi[1] <= -d / 2:
        # 这个边界斥力只作用在y方向，因此x方向为0
        F_rep_edge = [0, Eta_rep_edge * np.linalg.norm(Pi[2:4]) * np.exp(-d / 2 - Pi[1])]
    elif -d / 2 < Pi[1] <= -W / 2:
        F_rep_edge = [0, 1 / 3 * Eta_rep_edge * Pi[1] ** 2]
    elif W / 2 < Pi[1] <= d / 2:
        F_rep_edge = [0, -1 / 3 * Eta_rep_edge * Pi[1] ** 2]
    elif d / 2 < Pi[1] <= d - W / 2:
        F_rep_edge = [0, Eta_rep_edge * np.linalg.norm(Pi[2:4]) * np.exp(Pi[1] - d / 2)]

    # 计算合力和方向
    F_rep = [np.sum(F_rep_ob[:, 0]) + F_rep_edge[0], np.sum(F_rep_ob[:, 1]) + F_rep_edge[1]]  # 合并边界斥力和障碍舞斥力
    F_att = [Eta_att * dist[n - 1] * unitvector[n - 1, 0], Eta_att * dist[n - 1] * unitvector[n - 1, 1]]  # 引力向量
    F_sum = np.array([F_rep[0] + F_att[0], F_rep[1] + F_att[1]])
    UnitVec_Fsum = 1 / np.linalg.norm(F_sum) * F_sum
    # 计算车的下一步位置
    Pi[0:2] = Pi[0:2] + len_step * UnitVec_Fsum
# 将目标添加到路径中
Path.append(Pg.tolist())

# 画图
x = []  # 路径的x坐标
y = []  # 路径的y坐标
for val in Path:
    x.append(val[0])
    y.append(val[1])

plt.plot(x, y, linewidth=0.6)
plt.axhline(y=0, color='g', linestyle=':',linewidth=0.3)
plt.axis([-5,100,-4,4])
plt.gca().set_aspect('equal')
plt.plot(0, -d / 2, 'ro', markersize=1)
plt.plot(15, 7 / 4, 'ro', markersize=1)
plt.plot(30, -3 / 2, 'ro', markersize=1)
plt.plot(45, 3 / 2, 'ro', markersize=1)
plt.plot(60, -3 / 4, 'ro', markersize=1)
plt.plot(80, 7 / 4, 'ro', markersize=1)
plt.plot(99, d / 2, 'ro', markersize=1)
plt.xlabel('x')
plt.ylabel('y')
plt.show()