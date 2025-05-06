% 加载地图
load('complex_pathfinding_map.mat', 'map');

% 定义起点和终点
startLocation = [1, 1];
endLocation = [28, 28];

% 运行RRT路径规划
[path, found, tree, metrics] = rrt_planner(map, startLocation, endLocation);