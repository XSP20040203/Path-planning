function [v_cmd, omega_cmd, predicted_traj] = dwa_local_planner(robot_state, global_path, map, robot_params, dwa_params)
    % 动态窗口算法 (DWA) 局部运动规划模块
    % 输入:
    %   robot_state - 机器人当前状态 [x, y, theta, v_linear, v_angular] (theta 为航向角,弧度)
    %   global_path - 由 A* 规划出的全局路径 (N x 2 矩阵, [x, y] 世界坐标)
    %   map - occupancyMap 对象
    %   robot_params - 机器人参数结构体, e.g.,
    %       robot_params.max_v = 1.0;       % 最大线速度
    %       robot_params.min_v = -0.5;      % 最小线速度 (允许后退)
    %       robot_params.max_omega = 1.5;   % 最大角速度
    %       robot_params.max_accel = 1.0;   % 最大线加速度
    %       robot_params.max_ang_accel = 2.0; % 最大角加速度
    %       robot_params.v_resolution = 0.05; % 线速度采样分辨率
    %       robot_params.omega_resolution = 0.1; % 角速度采样分辨率
    %       robot_params.predict_time = 1.0; % 轨迹预测时间 (s)
    %       robot_params.time_step = 0.1;   % 仿真步长 (s)
    %       robot_params.robot_radius = 0.3; % 机器人半径 (用于碰撞检测)
    %   dwa_params - DWA 参数结构体, e.g.,
    %       dwa_params.weight_goal = 0.5;    % 目标导向评分权重
    %       dwa_params.weight_obstacle = 0.5; % 障碍物评分权重
    %       dwa_params.weight_velocity = 0.1; % 速度评分权重
    %       dwa_params.forward_vel_reward = 0.1; % 奖励前进速度的权重
    %       dwa_params.dist_threshold = 0.5; % 距离目标点小于此阈值时视为到达
    % 输出:
    %   v_cmd, omega_cmd - 计算出的最优线速度和角速度指令
    %   predicted_traj - 最优速度对应的预测轨迹 (M x 2 矩阵)

    % 提取机器人当前状态
    x = robot_state(1);
    y = robot_state(2);
    theta = robot_state(3);
    v_linear = robot_state(4);
    v_angular = robot_state(5);

    % --- 1. 计算动态窗口 (Dynamic Window) ---
    % 计算当前时刻可行的线速度和角速度范围
    % Vr = {v | v_min <= v <= v_max, omega_min <= omega <= omega_max} (机器人硬件限制)
    % Vd = {v | v_cur - accel*dt <= v <= v_cur + accel*dt, omega_cur - ang_accel*dt <= omega <= omega_cur + ang_accel*dt} (动力学限制)
    % V = Vr intersection Vd (动态窗口)

    dt = robot_params.time_step;
    
    % 根据当前速度和加速度计算速度限制窗口
    v_d_min = v_linear - robot_params.max_accel * dt;
    v_d_max = v_linear + robot_params.max_accel * dt;
    omega_d_min = v_angular - robot_params.max_ang_accel * dt;
    omega_d_max = v_angular + robot_params.max_ang_accel * dt;

    % 结合机器人硬件限制，得到动态窗口
    v_window_min = max(robot_params.min_v, v_d_min);
    v_window_max = min(robot_params.max_v, v_d_max);
    omega_window_min = max(-robot_params.max_omega, omega_d_min); % 注意角速度限制通常是 +/- max
    omega_window_max = min(robot_params.max_omega, omega_d_max);

    % 确保窗口有效
    if v_window_min > v_window_max || omega_window_min > omega_window_max
        warning('动态窗口无效，输出零速度。');
        v_cmd = 0;
        omega_cmd = 0;
        predicted_traj = [x y]; % 轨迹是当前点
        return;
    end

    % --- 2. 速度采样 ---
    vs = v_window_min : robot_params.v_resolution : v_window_max;
    omegas = omega_window_min : robot_params.omega_resolution : omega_window_max;

    if isempty(vs) || isempty(omegas)
         warning('动态窗口采样为空，输出零速度。');
        v_cmd = 0;
        omega_cmd = 0;
        predicted_traj = [x y];
        return;
    end

    % --- 3. 轨迹预测与评估 ---
    max_score = -Inf;
    v_cmd = 0; % 默认指令为零速度
    omega_cmd = 0;
    predicted_traj = [x y]; % 默认轨迹为当前点

    % 在全局路径上找到一个局部目标点
    % 简单的策略：选择全局路径上距离当前机器人位置最近的点作为局部目标
    % 更复杂的策略：选择全局路径上前方一定距离的点，或者使用航向角引导
    [~, nearest_idx] = min(vecnorm(global_path - [x y], 2, 2));
    % 选择全局路径上最近点后方的几个点作为参考，或者直接使用最近点
    local_goal_point = global_path(nearest_idx, :);
    % TODO: 可以改进为选择前方固定距离的点，或者考虑机器人的当前航向

    for v = vs
        for omega = omegas
            % 预测轨迹
            traj = predict_trajectory(x, y, theta, v, omega, robot_params.predict_time, dt);

            % 评估轨迹
            % 碰撞检测: 检查轨迹是否与障碍物碰撞
            min_dist_to_obstacle = check_collision(traj, map, robot_params.robot_radius);
            if min_dist_to_obstacle < robot_params.robot_radius
                 % 如果轨迹预测会碰撞，则该速度不可行，跳过
                 continue; 
            end
            
            % 目标导向评分: 轨迹终点距离局部目标点的距离
            % 注意：这里是最小距离，越近越好，评分应与距离负相关
            goal_dist = vecnorm(traj(end,:) - local_goal_point);
            score_goal = 1 / (goal_dist + 0.1); % 加一个小的常数避免除以零

            % 障碍物评分: 轨迹上离最近障碍物的距离 (已经通过 min_dist_to_obstacle 检查)
            % 评分应与最小距离正相关，越远越好。可以用指数衰减函数等
            % simplest: score_obstacle = min_dist_to_obstacle;
            % more robust: penalize based on distance, e.g., inversely proportional to distance
            % Let's use a simple inverse distance for scoring valid trajectories
             score_obstacle = min_dist_to_obstacle; % Penalized if < robot_radius, otherwise higher is better

            % 速度评分: 轨迹末端速度大小 (鼓励高速)
            score_velocity = v; % 简单的使用线速度作为评分，鼓励前进

             % 额外：鼓励朝向目标方向的速度，惩罚后退
             % 如果线速度是负的，给一个负的奖励
             score_forward = 0;
             if v > 0
                 score_forward = v * dwa_params.forward_vel_reward; % 简单的奖励前进速度
             end

            % 计算总评分 (加权和)
            score = dwa_params.weight_goal * score_goal + ...
                    dwa_params.weight_obstacle * score_obstacle + ...
                    dwa_params.weight_velocity * score_velocity + ...
                    score_forward; % 添加前进速度奖励

            % 更新最优速度和轨迹
            if score > max_score
                max_score = score;
                v_cmd = v;
                omega_cmd = omega;
                predicted_traj = traj;
            end
        end
    end

    % 如果没有找到任何有效（无碰撞）的轨迹，则保持零速度
    if max_score == -Inf
        v_cmd = 0;
        omega_cmd = 0;
        predicted_traj = [x y];
    end

    % 检查是否到达全局终点（可选：可以在主仿真循环中检查）
    % dist_to_final_goal = vecnorm([x y] - global_path(end,:));
    % if dist_to_final_goal < dwa_params.dist_threshold
    %     v_cmd = 0;
    %     omega_cmd = 0;
    % end

end

% --- DWA 辅助函数 ---

function traj = predict_trajectory(x0, y0, theta0, v, omega, predict_time, dt)
    % 预测机器人在给定速度下的轨迹
    % 简单的直线+旋转模型 (欧拉积分)

    time_points = 0:dt:predict_time;
    num_steps = length(time_points);
    traj = zeros(num_steps, 2); % 存储 [x, y] 轨迹点
    
    x = x0;
    y = y0;
    theta = theta0;

    for i = 1:num_steps
        % 更新位置和航向
        if abs(omega) < 1e-6 % 近似直线运动
            x = x + v * cos(theta) * dt;
            y = y + v * sin(theta) * dt;
            % theta 不变
        else % 圆弧运动
            x = x - v/omega * sin(theta) + v/omega * sin(theta + omega * dt);
            y = y + v/omega * cos(theta) - v/omega * cos(theta + omega * dt);
            theta = theta + omega * dt;
        end
        
        traj(i, :) = [x, y];
    end
end

function min_dist = check_collision(traj, map, robot_radius)
    % 检查轨迹是否与地图障碍物碰撞
    % 简化处理：检查轨迹上的每个点距离最近障碍物的距离
    
    if isempty(traj)
        min_dist = Inf; % 空轨迹，无碰撞
        return;
    end

    % 将轨迹点从世界坐标转换为地图栅格坐标
    traj_grid = world2grid(map, traj);

    % 获取占据栅格矩阵
    occupancy_matrix = occupancyMatrix(map);
    
    min_dist = Inf;

    for i = 1:size(traj_grid, 1)
        tr = round(traj_grid(i, 1)); % 栅格行 (向下取整或四舍五入取决于地图实现)
        tc = round(traj_grid(i, 2)); % 栅格列

        % 检查点是否在地图范围内
        if tr < 1 || tr > size(occupancy_matrix, 1) || tc < 1 || tc > size(occupancy_matrix, 2)
             min_dist = 0; % 轨迹出界视为碰撞
             return;
        end

        % 如果轨迹点直接落在占据栅格内
        if occupancy_matrix(tr, tc) > 0.5
             min_dist = 0; % 发生碰撞
             return;
        end

        % 计算轨迹点到最近障碍物的距离
        % 这部分需要更高效的实现，比如距离变换图 (Distance Transform)
        % 简化方法：搜索轨迹点周围的栅格
        
        % 获取所有障碍物点的栅格坐标 (一次计算，重复使用)
        persistent obstacle_grids; % 使用 persistent 避免重复计算
        if isempty(obstacle_grids) || ~isequal(size(obstacle_grids), size(occupancy_matrix)) % 重新加载或地图变化时更新
             [obs_rows, obs_cols] = find(occupancy_matrix > 0.5);
             obstacle_grids = [obs_rows, obs_cols];
        end

        if isempty(obstacle_grids) % 没有障碍物
            current_min_dist = Inf;
        else
            % 计算当前轨迹点到所有障碍物点的欧氏距离
            distances_to_obstacles = vecnorm(obstacle_grids - [tr tc], 2, 2);
            current_min_dist = min(distances_to_obstacles) * map.Resolution; % 转换为世界距离
        end
        
        % 更新整个轨迹的最小距离
        min_dist = min(min_dist, current_min_dist);

         % 如果在预测过程中发现距离小于机器人半径，即可判定为碰撞
         if min_dist < robot_radius
             min_dist = 0; % 近似碰撞
             return;
         end
    end

    % 返回轨迹上距离最近障碍物的最小距离
    min_dist = max(0, min_dist - robot_radius); % 考虑机器人半径，距离障碍物边界的距离
    
end

% TODO: 可以添加其他评分函数，例如速度平滑性、航向角评分等