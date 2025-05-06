% 加载地图
load('complex_pathfinding_map1.mat', 'map');
class(map)
% 定义起点和终点
startLocation = [1, 1];
endLocation = [28, 28];

[path_world, path_length, ~] = a_star_search(...
    map, startLocation, endLocation, ...
    'heuristic_type', 'euclidean', ...
    'allow_diagonal', true);

if ~isempty(path_world)
    disp(['路径长度: ', num2str(path_length), ' 米']);
    % 绘制最终路径
    figure;
    show(map); hold on;
    plot(startLocation(1), startLocation(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(endLocation(1), endLocation(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    plot(path_world(:,1), path_world(:,2), 'b-', 'LineWidth', 2);
    legend('Start', 'Goal', 'Path',Location='northwest');
else
    disp('未找到可行路径！');
end