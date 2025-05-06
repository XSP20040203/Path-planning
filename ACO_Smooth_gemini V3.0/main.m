% 加载地图
load('complex_pathfinding_map1.mat', 'map');
class(map)
% 定义起点和终点
startLocation = [22, 1];
endLocation = [28, 28];

% 3. 调用蚁群算法函数进行路径规划
% 可以选择性地传入参数来调整ACO行为和平滑次数
fprintf('\n开始运行蚁群算法...\n');
[smoothed_path_world, smoothed_path_length, best_path_lengths_history] = ant_colony_optimization(map, startLocation, endLocation, ...
    'num_ants', 200, ...          % 蚂蚁数量
    'num_iterations', 200, ...    % 迭代次数
    'alpha', 1, ...            % 信息素重要性因子
    'beta', 25, ...              % 启发信息重要性因子
    'evaporation_rate', 0.3, ... % 信息素蒸发率
    'pheromone_deposit', 12, ...  % 信息素沉积量
    'smooth_iterations', 20);     % 平滑迭代次数

% 4. 显示结果
fprintf('\n蚁群算法运行结束。\n');

if ~isempty(smoothed_path_world)
    fprintf('平滑后找到的最佳路径长度: %.2f 米\n', smoothed_path_length);
    % 可视化已经由函数内部完成
else
    fprintf('未找到到达终点的有效路径。\n');
end

% 绘制历史最佳路径长度曲线已在函数内完成

%% 结束演示