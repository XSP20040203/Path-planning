% 加载地图
load('complex_pathfinding_map1.mat', 'map');

% 定义起点和终点
startLocation = [1, 1];
endLocation = [28, 28];

% 调用蚁群算法函数
[best_path_world, best_path_length, best_path_lengths_history] = ant_colony_optimization(map, startLocation, endLocation);

% 可视化结果
figure;
show(map);
hold on;
plot(startLocation(1), startLocation(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(endLocation(1), endLocation(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
if ~isempty(best_path_world)
    plot(best_path_world(:, 1), best_path_world(:, 2), 'r-', 'LineWidth', 2);
end
legend('Start','Goal','Path',Location='northwest')
hold off;
title(sprintf('蚁群算法规划路径 (最佳路径长度: %.2f)', best_path_length));

% 绘制迭代曲线
figure;
plot(1:length(best_path_lengths_history), best_path_lengths_history, 'b-');
title('最佳路径长度随迭代次数的变化');
xlabel('迭代次数');
ylabel('最佳路径长度');
grid on;