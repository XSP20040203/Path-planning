% 加载地图
load('complex_pathfinding_map.mat', 'map');
class(map)
load('gridmap_20x20_scene1.mat','grid_map')
class(grid_map)

% 2. 提取栅格数据并转换为 double
grid_data = map.occupancyMatrix; % 直接访问属性（旧版本可能用 occupancyMatrix）
grid_double = double(grid_data); 

% 3. 显示结果
disp('转换后的数据类型:');
disp(class(grid_double));
disp('唯一值:');
disp(unique(grid_double));

% 4. 可视化对比
subplot(1,2,1);
show(map);
title('原始 binaryOccupancyMap');

subplot(1,2,2);
imshow(grid_double);
title('转换后的 double 矩阵');


