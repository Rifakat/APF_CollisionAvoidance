"""
人工势场寻路算法实现
改进人工势场，解决不可达问题，仍存在局部最小值问题
"""
from Original_APF import APF, Vector2d
import matplotlib.pyplot as plt
import math
from matplotlib.patches import Circle
import random


def check_vec_angle(v1: Vector2d, v2: Vector2d):
    """
    计算两个向量之间的夹角
    :param v1: 第一个向量,Vector2d类型
    :param v2: 第二个向量,Vector2d类型
    :return: 两个向量的夹角(角度制)
    """    
    v1_v2 = v1.deltaX * v2.deltaX + v1.deltaY * v2.deltaY  # 向量点积公式: a·b = |a||b|cosθ = ax*bx + ay*by
    angle = math.acos(v1_v2 / (v1.length * v2.length)) * 180 / math.pi  # 根据点积计算夹角并转换为角度
    return angle


class DynamicObstacle:
    """
    动态障碍物类
    """
    def __init__(self, start_pos: (), velocity: float, direction: float):
        self.pos = Vector2d(start_pos[0], start_pos[1])
        self.velocity = velocity
        # 将角度转换为弧度
        self.direction = math.radians(direction)
        self.path = [[self.pos.deltaX, self.pos.deltaY]]

    def update_position(self, dt):
        """
        根据时间和速度更新障碍物位置
        :param dt: a small time step
        """
        dx = self.velocity * math.cos(self.direction) * dt
        dy = self.velocity * math.sin(self.direction) * dt
        self.pos.deltaX += dx
        self.pos.deltaY += dy
        self.path.append([self.pos.deltaX, self.pos.deltaY])


class APF_Improved(APF):
    def __init__(self, start: (), goal: (), obstacles: [], dynamic_obstacles: [], k_att: float, k_rep: float, rr: float,
                 step_size: float, max_iters: int, goal_threshold: float, is_plot=False):
        super().__init__(start, goal, obstacles, k_att, k_rep, rr, step_size, max_iters, goal_threshold, is_plot)
        self.dynamic_obstacles = dynamic_obstacles

    def path_plan(self):
        """
        path plan
        :return:
        """
        if self.is_plot:
            self.dynamic_obs_patches = [Circle(xy=(obs.pos.deltaX, obs.pos.deltaY), radius=0.5, color='green') for obs in self.dynamic_obstacles]
            for patch in self.dynamic_obs_patches:
                self.plot_circle.add_patch(patch)

        while self.iters < self.max_iters and (self.current_pos - self.goal).length > self.goal_threashold:
            # 更新动态障碍物位置
            for i, obs in enumerate(self.dynamic_obstacles):
                obs.update_position(self.step_size) # 使用step_size来作为时间步长
                if self.is_plot:
                    self.dynamic_obs_patches[i].center = obs.pos.deltaX, obs.pos.deltaY
            
            # 将动态障碍物加入到总的障碍物列表中进行斥力计算
            # 注意：这里我们创建一个新的列表，而不是直接修改 self.obstacles，以避免在循环中产生意外的副作用
            current_obstacles = self.obstacles + [obs.pos for obs in self.dynamic_obstacles]
            
            # 计算合力时传入当前的障碍物列表
            f_vec = self.attractive() + self.repulsion_dynamic(current_obstacles)

            self.current_pos += Vector2d(f_vec.direction[0], f_vec.direction[1]) * self.step_size
            self.iters += 1
            self.path.append([self.current_pos.deltaX, self.current_pos.deltaY])

            if self.is_plot:
                plt.plot(self.current_pos.deltaX, self.current_pos.deltaY, '.b')
                plt.pause(self.delta_t)
        
        if (self.current_pos - self.goal).length <= self.goal_threashold:
            self.is_path_plan_success = True

    def repulsion_dynamic(self, obstacles):
        """
        针对动态和静态障碍物的斥力计算
        """
        rep = Vector2d(0, 0)
        rob_to_goal = self.goal - self.current_pos
        for obstacle in obstacles:
            obs_to_rob = self.current_pos - obstacle
            if obs_to_rob.length > self.rr:
                pass
            else:
                rep_1 = Vector2d(obs_to_rob.direction[0], obs_to_rob.direction[1]) * self.k_rep * \
                        (1.0 / obs_to_rob.length - 1.0 / self.rr) / (obs_to_rob.length ** 2) * (rob_to_goal.length ** 2)
                rep_2 = Vector2d(rob_to_goal.direction[0], rob_to_goal.direction[1]) * self.k_rep * \
                        ((1.0 / obs_to_rob.length - 1.0 / self.rr) ** 2) * rob_to_goal.length
                rep += (rep_1 + rep_2)
        return rep


if __name__ == '__main__':
    # 相关参数设置
    k_att, k_rep = 2.0, 0.8
    rr = 1.0 # 斥力范围可以适当调整
    step_size, max_iters, goal_threashold = .2, 1000, .2
    step_size_ = 2

    # 设置、绘制起点终点
    start, goal = (0, 0), (16, 16)
    is_plot = True
    
    fig, subplot = None, None
    if is_plot:
        fig = plt.figure(figsize=(7, 7))
        subplot = fig.add_subplot(111)
        subplot.set_xlabel('X-distance: m')
        subplot.set_ylabel('Y-distance: m')
        subplot.plot(start[0], start[1], '*r')
        subplot.plot(goal[0], goal[1], '*r')
        subplot.set_xlim(-2, 18)
        subplot.set_ylim(-2, 18)

    # 静态障碍物设置及绘制
    obs = [[1, 4], [2, 4], [3.5, 3], [6, 1], [6, 7], [10, 6], [11, 12], [15.5, 15.5]]
    
    # 动态障碍物设置
    dynamic_obs_list = [
        DynamicObstacle(start_pos=(5, 5), velocity=0.1, direction=90), # 向上移动
        DynamicObstacle(start_pos=(10, 2), velocity=0.2, direction=180) #向左移动
    ]

    print('Static obstacles: {0}'.format(obs))
    if is_plot:
        for OB in obs:
            circle = Circle(xy=(OB[0], OB[1]), radius=rr, alpha=0.3)
            subplot.add_patch(circle)
            subplot.plot(OB[0], OB[1], 'xk')

    # 路径规划
    apf = APF_Improved(start, goal, obs, dynamic_obs_list, k_att, k_rep, rr, step_size, max_iters, goal_threashold, is_plot)
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
            
        print('Planned path points:{}'.format(path_))
        print('Path plan success')
        if is_plot:
            px, py = [K[0] for K in path_], [K[1] for K in path_]
            subplot.plot(px, py, '^k')
            
            # 绘制动态障碍物的轨迹
            for dob in dynamic_obs_list:
                d_px, d_py = [p[0] for p in dob.path], [p[1] for p in dob.path]
                subplot.plot(d_px, d_py, '--g')

            plt.show()
    else:
        print('Path plan failed')