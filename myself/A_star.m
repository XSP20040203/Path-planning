% 加载或创建地图
% 假设地图已经保存在 'complex_pathfinding_map.mat' 文件中
load('complex_pathfinding_map.mat', 'map');

% 定义起点和终点 (世界坐标)
startLocation = [1, 1];  % 起点位置
endLocation = [28, 28];  % 终点位置

% 调用A*算法函数进行路径规划并可视化
[path, path_length] = a_star_visualization(map, startLocation, endLocation);

% 显示最短路径长度
disp(['最短路径长度: ', num2str(path_length)]);

% 可视化最终路径
figure;
show(map);  % 显示占据栅格地图
hold on;
plot(path(:,1), path(:,2), 'r-', 'LineWidth', 2);  % 绘制路径
plot(startLocation(1), startLocation(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);  % 标记起点
plot(endLocation(1), endLocation(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);      % 标记终点
title('A* 规划路径');  % 图标题
xlabel('X [meters]');  % X轴标签
ylabel('Y [meters]');  % Y轴标签
hold off;