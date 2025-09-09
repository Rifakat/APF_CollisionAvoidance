"""
人工势场法避碰
改进人工势场，解决不可达问题和陷入局部最小值问题，并引入规则力使得避碰符合规则
"""
from Original_APF import APF, Vector2d
import matplotlib.pyplot as plt
import math
from matplotlib.patches import Circle, Polygon
import random
import os
import tempfile


def normalize_angle(angle):
    """将角度归一化到 [-pi, pi] 区间"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def check_vec_angle(v1: Vector2d, v2: Vector2d):
    v1_v2 = v1.deltaX * v2.deltaX + v1.deltaY * v2.deltaY
    angle = math.acos(v1_v2 / (v1.length * v2.length)) * 180 / math.pi
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
                 step_size: float, max_iters: int, goal_threshold: float, robot_max_speed: float, is_plot=False):
        super().__init__(start, goal, obstacles, k_att, k_rep, rr, step_size, max_iters, goal_threshold, is_plot)
        self.dynamic_obstacles = dynamic_obstacles
        self.robot_max_speed = robot_max_speed  # 机器人最大速度
        
        # --- COLREGs 规则相关参数 ---
        self.k_rule = 3.0  # "规则力"的强度系数
        self.colreg_lookahead_dist = self.rr * 3  # 规则感知的范围，比斥力场更大

        # --- 解决局部最小值问题参数 (修正并增强) ---
        self.is_in_escape_mode = False
        self.stuck_count = 0
        self.stuck_threshold = 10  # 判断为陷阱的迭代次数阈值
        self.escape_pos = None
        self.virtual_force_k = 50.0  # [修改] 增加虚拟力的强度，让逃逸更坚决
        self.escape_direction_vec = None

        # 新增：用于更鲁棒的停滞检测
        self.pos_history = []
        self.pos_history_len = 20         # 监控最近20个历史位置
        self.stagnation_threshold = 0.1   # 20帧内移动距离小于0.1则认为停滞

    def _get_boat_shape_vertices(self, pos, direction, scale=0.5):
        """
        根据位置和方向计算一个细长的五边形顶点
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

    def assess_situation(self, robot_heading, obstacle):
        """评估与单个动态障碍物（他船）的会遇局面"""
        # --- 基本几何关系 ---
        vec_to_obs = obstacle.pos - self.current_pos
        dist = vec_to_obs.length

        # 忽略远处的船只
        if dist > self.colreg_lookahead_dist:
            return "none", 0

        # --- 在世界坐标系下的角度 ---
        angle_to_obs_world = math.atan2(vec_to_obs.deltaY, vec_to_obs.deltaX)
        angle_to_robot_world = math.atan2(-vec_to_obs.deltaY, -vec_to_obs.deltaX)

        # --- 相对角度 ---
        relative_bearing = normalize_angle(angle_to_obs_world - robot_heading)
        bearing_on_obs = normalize_angle(angle_to_robot_world - obstacle.direction)
        heading_diff = normalize_angle(robot_heading - obstacle.direction)

        # --- 规则判断 ---
        overtaking_angle = math.radians(112.5)  # 追越扇区角度

        # 规则 13: 追越 (Overtaking)
        if abs(bearing_on_obs) > overtaking_angle:
            return "overtaking", relative_bearing

        # 规则 14: 对遇 (Head-on)
        if abs(heading_diff) > math.radians(170) and abs(relative_bearing) < math.radians(15):
            return "head_on", relative_bearing

        # 规则 15: 交叉相遇 (Crossing)
        if 0 < relative_bearing < overtaking_angle:
            return "crossing_give_way", relative_bearing  # 他船在右舷，我船为让路船
        if -overtaking_angle < relative_bearing <= 0:
            return "crossing_stand_on", relative_bearing  # 他船在左舷，我船为直航船

        return "none", relative_bearing

    def calculate_rule_force(self, situation, robot_heading, dist_to_obs):
        """根据会遇局面计算规则力"""
        f_rule = Vector2d(0, 0)
        # 规则力的大小与距离成反比
        magnitude = self.k_rule * (self.colreg_lookahead_dist / max(dist_to_obs, 0.1))

        # 计算一个向右转的单位向量
        dx = math.sin(robot_heading)
        dy = -math.cos(robot_heading)
        right_turn_vec = Vector2d(dx, dy)

        if situation == "head_on" or situation == "crossing_give_way":
            # 对遇或交叉相遇（让路），应向右转
            f_rule = right_turn_vec * magnitude
        elif situation == "overtaking":
            # 追越时，应主动避让（此处简化为默认向右）
            f_rule = right_turn_vec * magnitude * 0.5  # 转向力度稍小

        return f_rule

    def path_plan(self):
        """
        path plan
        :return:
        """
        if self.is_plot:
            # 绘制动态障碍物 
            self.dynamic_obs_patches = []
            self.dynamic_obs_rep_field_patches = [] # New list for repulsion fields
            for obs in self.dynamic_obstacles:
                # Boat
                vertices = self._get_boat_shape_vertices(obs.pos, obs.direction)
                boat = Polygon(vertices, color='green', alpha=0.6)
                self.dynamic_obs_patches.append(boat)
                self.plot_circle.add_patch(boat)

                # Repulsion field circle
                rep_field = Circle(xy=(obs.pos.deltaX, obs.pos.deltaY), radius=self.rr, color='m', alpha=0.2) # 'm' for magenta
                self.dynamic_obs_rep_field_patches.append(rep_field)
                self.plot_circle.add_patch(rep_field)

            # 为动态障碍物轨迹创建空的 line 对象，并添加到图表中
            self.dynamic_obs_traj_lines = []
            for i, obs in enumerate(self.dynamic_obstacles):
                label = 'Dynamic Obstacle Path' if i == 0 else ""
                line, = self.plot_circle.plot([], [], '--g', label=label)
                self.dynamic_obs_traj_lines.append(line)
            
            # 为机器人轨迹创建空的 line 对象
            self.robot_path_line, = self.plot_circle.plot([], [], '--b', label='Robot Path')

            # 为机器人自身创建 Polygon 对象
            # 计算初始朝向 (朝向目标点)
            initial_direction = math.atan2(self.goal.deltaY - self.current_pos.deltaY,
                                           self.goal.deltaX - self.current_pos.deltaX)
            vertices = self._get_boat_shape_vertices(self.current_pos, initial_direction)
            self.robot_patch = Polygon(vertices, color='blue', alpha=0.8)
            self.plot_circle.add_patch(self.robot_patch)

        # 初始方向，用于第一帧或速度为0时
        robot_direction = math.atan2(self.goal.deltaY - self.current_pos.deltaY,
                                     self.goal.deltaX - self.current_pos.deltaX)

        # DWA风格帧捕获以保存GIF
        frames = []
        frame_interval = 3  # 每隔多少帧保存一次，减小GIF大小
        frame_counter = 0

        while self.iters < self.max_iters and (self.current_pos - self.goal).length > self.goal_threashold:
            # 更新动态障碍物位置
            for i, obs in enumerate(self.dynamic_obstacles):
                obs.update_position(self.step_size)  # 使用step_size来作为时间步长
                if self.is_plot:
                    # 更新船的位置和方向
                    new_vertices = self._get_boat_shape_vertices(obs.pos, obs.direction)
                    self.dynamic_obs_patches[i].set_xy(new_vertices)

                    # 更新斥力场圆的位置
                    self.dynamic_obs_rep_field_patches[i].center = (obs.pos.deltaX, obs.pos.deltaY)

                    # 实时更新轨迹线的数据
                    path_x = [p[0] for p in obs.path]
                    path_y = [p[1] for p in obs.path]
                    self.dynamic_obs_traj_lines[i].set_data(path_x, path_y)

            # 将动态障碍物加入到总的障碍物列表中进行斥力计算
            # 注意：这里我们创建一个新的列表，而不是直接修改 self.obstacles，以避免在循环中产生意外的副作用
            current_obstacles = self.obstacles + [obs.pos for obs in self.dynamic_obstacles]
            
            # --- [修改] 单独计算斥力，以便用于逃逸判断 ---
            f_rep = self.repulsion_dynamic(current_obstacles)

            # --- 新增: COLREGs 规则判断与规则力计算 ---
            f_rule = Vector2d(0, 0)
            for obs in self.dynamic_obstacles:
                situation, _ = self.assess_situation(robot_direction, obs)
                if situation != "none":
                    dist_to_obs = (obs.pos - self.current_pos).length
                    f_rule_obs = self.calculate_rule_force(situation, robot_direction, dist_to_obs)
                    f_rule += f_rule_obs
                    print(f"Situation with boat at ({obs.pos.deltaX:.1f}, {obs.pos.deltaY:.1f}): {situation}, applying rule force.")
            
            # 计算合力时，融合吸引力、斥力、规则力
            f_vec = self.attractive() + f_rep + f_rule
           

            # --- 解决局部最小值问题 (替换为鲁棒的停滞检测版本) ---
            # 1. 更新并检查位置历史，判断是否停滞
            self.pos_history.append(Vector2d(self.current_pos.deltaX, self.current_pos.deltaY))
            if len(self.pos_history) > self.pos_history_len:
                self.pos_history.pop(0)

            is_stagnant = False
            if len(self.pos_history) == self.pos_history_len:
                dist_traveled = (self.pos_history[-1] - self.pos_history[0]).length
                if dist_traveled < self.stagnation_threshold:
                    is_stagnant = True
            
            # 2. 根据停滞状态更新stuck_count
            if is_stagnant:
                self.stuck_count += 1
            else:
                self.stuck_count = 0
            
            # 3. 如果确认被困，且未在逃逸，则启动逃逸模式
            if self.stuck_count >= self.stuck_threshold and not self.is_in_escape_mode:
                print("检测到停滞(前后振荡), 启动逃逸模式.")
                self.is_in_escape_mode = True
                self.escape_pos = Vector2d(self.current_pos.deltaX, self.current_pos.deltaY)
                # [核心修改] 回归到只在进入时计算一次方向，以"坚持"一个逃逸策略，避免左右摇摆
                self.escape_direction_vec = self._calculate_escape_direction(f_rep)

            # 4. 在逃逸模式下，增加虚拟力
            if self.is_in_escape_mode:
                # [核心修改] 不再每步都计算，而是使用进入时确定的单一方向来施加虚拟力
                # self.escape_direction_vec = self._calculate_escape_direction(f_rep)
                
                virtual_force = self.escape_direction_vec * self.virtual_force_k
                f_vec += virtual_force
                
                # 使用基于距离的逃逸判断
                if (self.current_pos - self.escape_pos).length > self.rr * 2.0:
                    print("已逃离陷阱.")
                    self.is_in_escape_mode = False
                    self.stuck_count = 0
                    self.pos_history = []  # 清空历史，重新评估
                    self.escape_direction_vec = None # 重置逃逸方向

            # --- 控制机器人速度 ---
            # 将力矢量 f_vec 视为期望的速度矢量
            # 如果期望速度超过最大速度，则进行限速
            if f_vec.length > self.robot_max_speed:
                vel_vec = Vector2d(f_vec.direction[0], f_vec.direction[1]) * self.robot_max_speed
            else:
                vel_vec = f_vec

            # 如果速度不为0，则更新方向
            if vel_vec.length > 1e-6:
                robot_direction = math.atan2(vel_vec.deltaY, vel_vec.deltaX)

            # 根据速度矢量和时间步长(step_size)更新位置
            self.current_pos += vel_vec * self.step_size

            self.iters += 1
            self.path.append([self.current_pos.deltaX, self.current_pos.deltaY])

            if self.is_plot:
                # 更新机器人自身(Polygon)的位置和方向
                new_vertices = self._get_boat_shape_vertices(self.current_pos, robot_direction)
                self.robot_patch.set_xy(new_vertices)

                # 实时更新机器人轨迹线的数据
                path_x = [p[0] for p in self.path]
                path_y = [p[1] for p in self.path]
                self.robot_path_line.set_data(path_x, path_y)
                plt.pause(self.delta_t)

                # 抓取帧保存成临时PNG
                plt.draw()
                frame_counter += 1
                if frame_counter % frame_interval == 0:
                    frame_file = os.path.join(tempfile.gettempdir(), f"apf_frame_{frame_counter:05d}.png")
                    try:
                        plt.savefig(frame_file)
                        frames.append(frame_file)
                    except Exception:
                        pass
        
        if (self.current_pos - self.goal).length <= self.goal_threashold:
            self.is_path_plan_success = True

        # 结束后再保存最后一帧，并合成GIF
        if self.is_plot:
            try:
                last_frame_file = os.path.join(tempfile.gettempdir(), f"apf_frame_{frame_counter+1:05d}.png")
                plt.savefig(last_frame_file)
                frames.append(last_frame_file)
            except Exception:
                pass

            if frames:
                print("正在保存GIF动画，请稍候...")
                try:
                    import imageio.v2 as imageio
                    gif_filename = 'APF_避碰.gif'
                    imageio.mimsave(gif_filename, [imageio.imread(f) for f in frames], fps=5)
                    print(f"动画已保存为: {gif_filename}")
                except ImportError:
                    print("保存GIF需要安装imageio库: pip install imageio")
                except Exception as e:
                    print(f"保存GIF时出错: {e}")
                finally:
                    # 清理临时帧文件
                    for f in frames:
                        if os.path.exists(f):
                            try:
                                os.remove(f)
                            except Exception:
                                pass

    def _calculate_escape_direction(self, repulsive_force: Vector2d) -> Vector2d:
        """
        计算一个更智能的、用于逃逸的单位向量方向。
        逃逸方向垂直于斥力方向，并从中选择更朝向目标点的方向。
        """
        to_goal_vec = self.goal - self.current_pos

        # 以"垂直于斥力方向"作为逃逸基准。斥力方向稳定指向远离障碍物的方向。
        if repulsive_force.direction is None or repulsive_force.length == 0:
            # 如果没有斥力，理论上不会被困，但作为安全措施，以"垂直于目标方向"作为备用策略
            base_dir = to_goal_vec.direction
            if base_dir is None: # 如果刚好在目标点，随机给一个方向
                return Vector2d(random.uniform(-1,1), random.uniform(-1,1))
        else:
            # 正常情况：使用斥力方向
            base_dir = repulsive_force.direction


        # 计算两个垂直于基准方向的、可能的逃逸方向
        dx, dy = base_dir
        escape_dir1 = Vector2d(-dy, dx)  # 垂直方向1 (eg. 左转)
        escape_dir2 = Vector2d(dy, -dx)  # 垂直方向2 (eg. 右转)

        # 通过点积判断哪个方向与去往目标的方向更一致 (点积越大，夹角越小)
        dot1 = to_goal_vec.deltaX * escape_dir1.deltaX + to_goal_vec.deltaY * escape_dir1.deltaY
        dot2 = to_goal_vec.deltaX * escape_dir2.deltaX + to_goal_vec.deltaY * escape_dir2.deltaY

        # 返回点积更大的那个方向，即与目标方向夹角更小的方向
        if dot1 > dot2:
            print("Escape direction: Left-biased")
            return escape_dir1
        else:
            print("Escape direction: Right-biased")
            return escape_dir2


    def repulsion_dynamic(self, obstacles):
        """
        针对动态和静态障碍物的斥力计算（恢复改进版本）
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
    rr = 3  # 斥力范围可以适当调整
    step_size, max_iters, goal_threashold = .2, 2000, .2
    robot_max_speed = 0.1  # 设置机器人最大速度
    step_size_ = 1.5

    # 设置、绘制起点终点
    start, goal = (0, 0), (15, 15)
    is_plot = True

    fig, subplot = None, None
    if is_plot:
        fig = plt.figure(figsize=(8, 8))
        subplot = fig.add_subplot(111)
        subplot.set_xlabel('X / n mile')
        subplot.set_ylabel('Y / n mile')
        subplot.plot(start[0], start[1], '*r', label='Start/Goal')
        subplot.plot(goal[0], goal[1], '*r')
        subplot.set_xlim(-2, 18)
        subplot.set_ylim(-2, 18)

    # 静态障碍物设置及绘制
    #obs = [[9, 8],  [8, 1],            #[6, 7], [10, 6], [11, 12],            [14, 14]]
    obs = [[1, 4],  [6, 7], [9, 6], [11, 12], [14, 14]]

    # 动态障碍物设置
    dynamic_obs_list = [
        #DynamicObstacle(start_pos=(9.5,9 ), velocity=0.04, direction=-135),  # 对遇
        #DynamicObstacle(start_pos=(3, 3), velocity=0.04, direction=45), # 追越
        DynamicObstacle(start_pos=(17, 9), velocity=0.07, direction=180)  # 右交叉
    ]

    print('Static obstacles: {0}'.format(obs))
    if is_plot:
        for OB in obs:
            circle = Circle(xy=(OB[0], OB[1]), radius=rr, alpha=0.3)
            subplot.add_patch(circle)
            subplot.plot(OB[0], OB[1], 'xk')

    # 路径规划
    apf = APF_Improved(start, goal, obs, dynamic_obs_list, k_att, k_rep, rr, step_size, max_iters, goal_threashold,
                       robot_max_speed, is_plot)
    if is_plot:
        apf.plot_circle = subplot

    apf.path_plan()

    # --- 修改后的绘图逻辑 ---
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
        # 最终路径已经在动画中绘制为蓝色虚线，这里不再需要重复绘制
        # if is_plot:
        #     px, py = [K[0] for K in path_], [K[1] for K in path_]
        #     subplot.plot(px, py, '^k', label='Robot Path')
    else:
        print('Path plan failed')

    # 轨迹已经在动画循环中实时绘制，这里只需要显示图例和最终的图像即可
    