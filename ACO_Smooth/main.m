clc; clear; close all;

% 加载地图
load('complex_pathfinding_map.mat', 'map');
class(map)
% 定义起点和终点
startLocation = [1, 1];
endLocation = [28, 28];

% 运行ACO+Smoothing
[best_path, path_length] = aco_with_smoothing(map, startLocation, endLocation, 'num_ants', 200);

% 可视化结果
figure;
show(map);
hold on;
plot(startLocation(1), startLocation(2), 'go', 'LineWidth',2, 'MarkerSize',10);
plot(endLocation(1), endLocation(2), 'ro', 'LineWidth',2, 'MarkerSize',10);
if ~isempty(best_path)
    plot(best_path(:,1), best_path(:,2), 'b-', 'LineWidth',2);
else
    disp('未找到可行路径');
end
title('ACO路径规划与平滑结果');
legend('起点', '终点', '平滑路径');
