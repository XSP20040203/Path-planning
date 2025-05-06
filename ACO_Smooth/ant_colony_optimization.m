function [best_path_world, best_path_length] = ant_colony_optimization(map, startLocation, endLocation, varargin)
    % 确保在所有代码分支中返回两个参数
    
    % 默认参数设置
    best_path_world = [];
    best_path_length = inf; % 初始化为无穷大
    
    % 检查地图对象合法性
    if ~isa(map, 'occupancyMap') && ~isa(map, 'binaryOccupancyMap')
        error('Invalid map type. Must be occupancyMap or binaryOccupancyMap.');
    end
    
    % 解析参数
    p = inputParser;
    addParameter(p, 'num_ants', 200);
    addParameter(p, 'num_iterations', 200);
    addParameter(p, 'alpha', 1.2);
    addParameter(p, 'beta', 14);
    addParameter(p, 'evaporation_rate', 0.1);
    addParameter(p, 'pheromone_deposit', 6);
    parse(p, varargin{:});
    
    % 转换坐标到栅格
    start_grid = world2grid(map, startLocation);
    end_grid = world2grid(map, endLocation);
    
    % 初始化信息素矩阵
    grid_size = map.GridSize;
    rows = grid_size(1);
    cols = grid_size(2);
    occupancy = occupancyMatrix(map);
    pheromone = ones(rows, cols);
    pheromone(occupancy > 0.5) = 0; % 障碍物的信息素设置为0
    
    % 存储最优路径
    global_best_path = [];
    global_best_length = inf;
    
    % 主循环
    for iter = 1:p.Results.num_iterations
        ant_paths = cell(p.Results.num_ants, 1); % 存储每只蚂蚁的路径
        
        % 每只蚂蚁独立搜索
        for ant = 1:p.Results.num_ants
            path = generate_ant_path(start_grid, end_grid, pheromone, map, p.Results.alpha, p.Results.beta);
            ant_paths{ant} = path;
        end
        
        % 更新信息素
        pheromone = update_pheromone(pheromone, ant_paths, ...
            p.Results.evaporation_rate, p.Results.pheromone_deposit, map);
        
        % 评估当前迭代最优路径
        [current_best_path, current_best_length] = evaluate_paths(ant_paths, map);
        
        % 更新全局最优
        if current_best_length < global_best_length
            global_best_length = current_best_length;
            global_best_path = current_best_path;
        end
    end
    
    % 转换为世界坐标
    if ~isempty(global_best_path)
        best_path_world = grid2world(map, global_best_path);
        best_path_length = global_best_length;
    end
end

function path = generate_ant_path(start_grid, end_grid, pheromone, map, alpha, beta)
    path = start_grid;    % 路径起点
    current_pos = start_grid;
    max_steps = 2 * (map.GridSize(1) + map.GridSize(2)); % 防止陷入死循环
    
    for step = 1:max_steps
        if isequal(current_pos, end_grid)
            break;  % 到达终点
        end
        neighbors = get_valid_neighbors(current_pos, map);
        if isempty(neighbors)
            path = []; % 死路
            break;
        end
        
        % 计算每个相邻点的概率
        probabilities = zeros(length(neighbors), 1);
        for i = 1:size(neighbors,1)
            neighbor = neighbors(i,:);
            pheromone_val = pheromone(neighbor(1), neighbor(2));
            heuristic_val = 1 / (norm(neighbor - end_grid) + 1e-6); % 避免除零
            probabilities(i) = (pheromone_val^alpha) * (heuristic_val^beta);
        end
        probabilities = probabilities / sum(probabilities); % 归一化
        
        % 轮盘赌选择下一步
        next_idx = find(rand() <= cumsum(probabilities), 1, 'first');
        current_pos = neighbors(next_idx, :);
        path = [path; current_pos];
    end
end

function pheromone = update_pheromone(old_pheromone, ant_paths, rho, Q, map)
    pheromone = (1 - rho) * old_pheromone;    % 信息素蒸发
    
    % 遍历所有蚂蚁的路径并沉积信息素
    for k = 1:length(ant_paths)
        path = ant_paths{k};
        if isempty(path)
            continue; 
        end
        
        % 计算路径长度
        path_world = grid2world(map, path);
        path_length = sum(sqrt(sum(diff(path_world).^2, 2))); 
        
        % 沉积信息素
        delta = Q / path_length;
        for i = 1:size(path,1)
            if checkOccupancy(map, path(i,:)) == 0 % 仅允许在自由空间沉积
                pheromone(path(i,1), path(i,2)) = pheromone(path(i,1), path(i,2)) + delta;
            end
        end
    end
end

function [best_path, best_length] = evaluate_paths(paths, map)
    best_length = inf;
    best_path = [];
    
    for k = 1:length(paths)
        path = paths{k};
        if isempty(path)
            continue;
        end
        
        % 计算路径长度
        path_world = grid2world(map, path);
        current_length = sum(sqrt(sum(diff(path_world).^2, 2)));
        
        % 更新最优
        if current_length < best_length
            best_length = current_length;
            best_path = path;
        end
    end
end

function neighbors = get_valid_neighbors(current_pos, map)
    % 获取当前点的有效邻接点（8方向）
    directions = [-1,-1; -1,0; -1,1; 0,-1; 0,1; 1,-1; 1,0; 1,1];
    neighbors = [];
    grid_size = map.GridSize;
    rows = grid_size(1);
    cols = grid_size(2);
    
    for i = 1:size(directions,1)
        next_pos = current_pos + directions(i,:);
        if next_pos(1) >= 1 && next_pos(1) <= rows ...  % 行边界检查
                && next_pos(2) >= 1 && next_pos(2) <= cols ... % 列边界检查
                && checkOccupancy(map, next_pos) == 0 % 非障碍物
            neighbors = [neighbors; next_pos]; 
        end
    end
end
