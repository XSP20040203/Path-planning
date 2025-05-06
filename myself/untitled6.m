% 结合 A* 和 DWA 进行导航的仿真示例

clear; close all; clc;

% 加载地图
load('complex_pathfinding_map.mat', 'map');

% 定义起点和终点
startLocation = [1, 1];
endLocation = [28, 28];

% --- 2. 使用 A* 规划全局路径 ---
% 调用之前写的 a_star_search 函数
% 确保 a_star_search.m 文件在当前 MATLAB 路径下
[global_path_world, path_length_astar, ~] = a_star_search(map, startLocation, endLocation, 'allow_diagonal', true, 'heuristic_type', 'euclidean');

if isempty(global_path_world)
    error('A* 未找到全局路径.');
end

disp(['A* 规划路径长度: ', num2str(path_length_astar)]);

% 在 A* 可视化图上绘制全局路径
figure(findobj('Name', 'A* Path Planning')); hold on;
plot(global_path_world(:,1), global_path_world(:,2), 'm--', 'LineWidth', 1.5, 'DisplayName', 'Global Path (A*)');
legend show; % 刷新 legend
