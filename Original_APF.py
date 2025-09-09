"""
人工势场寻路算法实现
最基本的人工势场，存在目标点不可达及局部最小值问题
"""
import math
import random
from matplotlib import pyplot as plt
from matplotlib.patches import Circle, Polygon
import time


class Vector2d():
    """
    2维向量, 支持加减, 支持常量乘法(右乘)
    """

    def __init__(self, x, y):
        self.deltaX = x
        self.deltaY = y
        self.length = -1
        self.direction = [0, 0]
        self.vector2d_share()

    def vector2d_share(self):
        if type(self.deltaX) == type(list()) and type(self.deltaY) == type(list()):
            deltaX, deltaY = self.deltaX, self.deltaY
            self.deltaX = deltaY[0] - deltaX[0]
            self.deltaY = deltaY[1] - deltaX[1]
            self.length = math.sqrt(self.deltaX ** 2 + self.deltaY ** 2) * 1.0
            if self.length > 0:
                self.direction = [self.deltaX / self.length, self.deltaY / self.length]
            else:
                self.direction = None
        else:
            self.length = math.sqrt(self.deltaX ** 2 + self.deltaY ** 2) * 1.0
            if self.length > 0:
                self.direction = [self.deltaX / self.length, self.deltaY / self.length]
            else:
                self.direction = None

    def __add__(self, other):
        """
        + 重载
        :param other:
        :return:
        """
        vec = Vector2d(self.deltaX, self.deltaY)
        vec.deltaX += other.deltaX
        vec.deltaY += other.deltaY
        vec.vector2d_share()
        return vec

    def __sub__(self, other):
        vec = Vector2d(self.deltaX, self.deltaY)
        vec.deltaX -= other.deltaX
        vec.deltaY -= other.deltaY
        vec.vector2d_share()
        return vec

    def __mul__(self, other):
        vec = Vector2d(self.deltaX, self.deltaY)
        vec.deltaX *= other
        vec.deltaY *= other
        vec.vector2d_share()
        return vec

    def __truediv__(self, other):
        return self.__mul__(1.0 / other)

    def __repr__(self):
        return 'Vector deltaX:{}, deltaY:{}, length:{}, direction:{}'.format(self.deltaX, self.deltaY, self.length, self.direction)

class APF():
    """
    人工势场寻路
    """

    def __init__(self, start: (), goal: (), obstacles: [], k_att: float, k_rep: float, rr: float,
                 step_size: float, max_iters: int, goal_threshold: float, is_plot=False):
        """
        :param start: 起点
        :param goal: 终点
        :param obstacles: 障碍物列表，每个元素为Vector2d对象
        :param k_att: 引力系数
        :param k_rep: 斥力系数
        :param rr: 斥力作用范围
        :param step_size: 步长
        :param max_iters: 最大迭代次数
        :param goal_threshold: 离目标点小于此值即认为到达目标点
        :param is_plot: 是否绘图
        """
        self.start = Vector2d(start[0], start[1])
        self.current_pos = Vector2d(start[0], start[1])
        self.goal = Vector2d(goal[0], goal[1])
        self.obstacles = [Vector2d(OB[0], OB[1]) for OB in obstacles]
        self.k_att = k_att
        self.k_rep = k_rep
        self.rr = rr  # 斥力作用范围
        self.step_size = step_size
        self.max_iters = max_iters
        self.iters = 0
        self.goal_threashold = goal_threshold
        self.path = list()
        self.is_path_plan_success = False
        self.is_plot = is_plot
        self.delta_t = 0.01
        self.plot_circle = None

    def _get_boat_shape_vertices(self, pos, direction, scale=0.5):
        """
        根据位置和方向计算一个细长的五边形顶点 (从改进的APF代码中借鉴)
        """
        # 定义一个正五边形的本地坐标顶点, 然后进行拉伸
        points = []
        stretch_x = 1.5  # x轴方向拉伸因子 (长度)
        stretch_y = 0.6  # y轴方向收缩因子 (宽度)
        for i in range(5):
            angle = 2 * math.pi * i / 5
            # 在计算出的正五边形顶点基础上进行x,y方向的缩放
            x = scale * math.cos(angle) * stretch_x
            y = scale * math.sin(angle) * stretch_y
            points.append((x, y))

        # 旋转和平移
        cos_dir = math.cos(direction)
        sin_dir = math.sin(direction)

        rotated_translated_points = []
        for x, y in points:
            x_rot = x * cos_dir - y * sin_dir
            y_rot = x * sin_dir + y * cos_dir

            x_final = x_rot + pos.deltaX
            y_final = y_rot + pos.deltaY
            rotated_translated_points.append((x_final, y_final))

        return rotated_translated_points

    def attractive(self):
        """
        引力计算
        :return: 引力
        """
        att = (self.goal - self.current_pos) * self.k_att  # 方向由机器人指向目标点
        return att

    def repulsion(self):
        """
        斥力计算
        :return: 斥力大小
        """
        rep = Vector2d(0, 0)  # 所有障碍物总斥力
        for obstacle in self.obstacles:
            # obstacle = Vector2d(0, 0)
            t_vec = self.current_pos - obstacle
            if (t_vec.length > self.rr):  # 超出障碍物斥力影响范围
                pass
            else:
                rep += Vector2d(t_vec.direction[0], t_vec.direction[1]) * self.k_rep * (
                        1.0 / t_vec.length - 1.0 / self.rr) / (t_vec.length ** 2)  # 方向由障碍物指向机器人
        return rep

    def path_plan(self):
        """
        path plan
        :return:
        """
        # 初始方向
        robot_direction = math.atan2(self.goal.deltaY - self.current_pos.deltaY,
                                     self.goal.deltaX - self.current_pos.deltaX)

        if self.is_plot:
            # 初始化机器人和轨迹的可视化对象
            vertices = self._get_boat_shape_vertices(self.current_pos, robot_direction)
            self.robot_patch = Polygon(vertices, color='blue', alpha=0.8, label="Robot")
            self.plot_circle.add_patch(self.robot_patch)
            self.robot_path_line, = self.plot_circle.plot([], [], '--b', label='Robot Path')

        while self.iters < self.max_iters and (self.current_pos - self.goal).length > self.goal_threashold:
            f_vec = self.attractive() + self.repulsion()

            # 根据合力方向更新机器人朝向
            if f_vec.length > 1e-6:
                robot_direction = math.atan2(f_vec.deltaY, f_vec.deltaX)

            self.current_pos += Vector2d(f_vec.direction[0], f_vec.direction[1]) * self.step_size
            self.iters += 1
            self.path.append([self.current_pos.deltaX, self.current_pos.deltaY])

            if self.is_plot:
                # 更新机器人(Polygon)的位置和方向
                new_vertices = self._get_boat_shape_vertices(self.current_pos, robot_direction)
                self.robot_patch.set_xy(new_vertices)

                # 更新机器人轨迹线
                path_x = [p[0] for p in self.path]
                path_y = [p[1] for p in self.path]
                self.robot_path_line.set_data(path_x, path_y)
                plt.pause(self.delta_t)
        if (self.current_pos - self.goal).length <= self.goal_threashold:
            self.is_path_plan_success = True


if __name__ == '__main__':
    # 相关参数设置
    k_att, k_rep = 1.0, 100.0
    rr = 3
    step_size, max_iters, goal_threashold = .2, 500, .2  # 步长0.5寻路1000次用时4.37s, 步长0.1寻路1000次用时21s
    step_size_ = 2

    # 设置、绘制起点终点
    start, goal = (0, 0), (15, 15)
    is_plot = True
    if is_plot:
        fig = plt.figure(figsize=(8, 8))
        subplot = fig.add_subplot(111)
        subplot.set_xlabel('X / n mile')
        subplot.set_ylabel('Y / n mile')
        subplot.plot(start[0], start[1], '*r', label='Start/Goal')
        subplot.plot(goal[0], goal[1], '*r')
        subplot.set_xlim(-2, 17)
        subplot.set_ylim(-2, 17)
    # 障碍物设置及绘制
    obs = [[1, 4], [2, 4], [3, 3], [4, 1], [6, 7], [10, 6], [11, 12], [14, 14]]
    print('obstacles: {0}'.format(obs))
    for i in range(0):
        obs.append([random.uniform(2, goal[1] - 1), random.uniform(2, goal[1] - 1)])

    if is_plot:
        for OB in obs:
            circle = Circle(xy=(OB[0], OB[1]), radius=rr, alpha=0.3)
            subplot.add_patch(circle)
            subplot.plot(OB[0], OB[1], 'xk')
    # t1 = time.time()
    # for i in range(1000):

    # path plan
    apf = APF(start, goal, obs, k_att, k_rep, rr, step_size, max_iters, goal_threashold, is_plot)
    if is_plot:
        apf.plot_circle = subplot

    apf.path_plan()
    if apf.is_path_plan_success:
        path = apf.path
        path_ = []
        i = int(step_size_ / step_size)
        while i < len(path):
            path_.append(path[i])
            i += int(step_size_ / step_size)

        if not path_ or path_[-1] != path[-1]:
            path_.append(path[-1])

        print('planed path points:{}'.format(path_))
        print('path plan success')
        # 实时绘图后，不再需要最后统一绘制路径
        # if is_plot:
        #     px, py = [K[0] for K in path_], [K[1] for K in path_]  # 路径点x坐标列表, y坐标列表
        #     subplot.plot(px, py, '^k')
    else:
        print('path plan failed')

    # 显示图例和最终绘图窗口
    if is_plot:
        plt.legend()
        plt.show()
    # t2 = time.time()
    # print('寻路1000次所用时间:{}, 寻路1次所用时间:{}'.format(t2-t1, (t2-t1)/1000))
