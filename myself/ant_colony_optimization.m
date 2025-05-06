function [best_path_world, best_path_length, best_path_lengths_history] = ant_colony_optimization(map, startLocation, endLocation, varargin)
    % 蚁群算法路径规划
    % 输入:
    %   map - 占据栅格地图对象
    %   startLocation - 起点在世界坐标系中的位置 [x, y]
    %   endLocation - 终点在世界坐标系中的位置 [x, y]
    %   varargin - 可选参数，包括 num_ants, num_iterations, alpha, beta, evaporation_rate, pheromone_deposit
    % 输出:
    %   best_path_world - 最佳路径在世界坐标系中的点序列
    %   best_path_length - 最佳路径的长度
    %   best_path_lengths_history - 每次迭代的最佳路径长度历史记录

    % 默认参数
    default_num_ants = 200;
    default_num_iterations = 100;
    default_alpha = 1;
    default_beta = 20;
    default_evaporation_rate = 0.1;
    default_pheromone_deposit = 10;

    % 解析可选参数
    p = inputParser;
    addParameter(p, 'num_ants', default_num_ants);
    addParameter(p, 'num_iterations', default_num_iterations);
    addParameter(p, 'alpha', default_alpha);
    addParameter(p, 'beta', default_beta);
    addParameter(p, 'evaporation_rate', default_evaporation_rate);
    addParameter(p, 'pheromone_deposit', default_pheromone_deposit);
    parse(p, varargin{:});

    num_ants = p.Results.num_ants;
    num_iterations = p.Results.num_iterations;
    alpha = p.Results.alpha;
    beta = p.Results.beta;
    evaporation_rate = p.Results.evaporation_rate;
    pheromone_deposit = p.Results.pheromone_deposit;

    % 获取地图的栅格尺寸
    gridSize = map.GridSize;
    rows = gridSize(1);
    cols = gridSize(2);

    % 获取整个地图的占据矩阵
    occupancy_matrix = occupancyMatrix(map);

    % 初始化信息素矩阵
    pheromone = ones(rows, cols); % 初始化所有单元格的信息素浓度为1
    pheromone(occupancy_matrix > 0.5) = 0; % 将占据的单元格信息素设为0

    % 定义启发信息 (使用欧几里得距离的倒数)
    [cols_idx, rows_idx] = meshgrid(1:cols, 1:rows);
    grid_indices_all = [rows_idx(:), cols_idx(:)];
    world_coords_all = grid2world(map, grid_indices_all);
    dist_to_goal_all = sqrt(sum((world_coords_all - endLocation).^2, 2));
    heuristic = reshape(1./(dist_to_goal_all + eps), rows, cols); % 加eps避免除以零

    % 将终点位置的启发信息设为最大值
    goal_grid_coords = world2grid(map, endLocation);
    goal_row = goal_grid_coords(1);
    goal_col = goal_grid_coords(2);
    max_heuristic = max(heuristic(~isinf(heuristic)));
    heuristic(goal_row, goal_col) = max_heuristic;

    % 获取起点和终点的栅格索引
    start_grid_coords = world2grid(map, startLocation);
    start_row = start_grid_coords(1);
    start_col = start_grid_coords(2);
    start_grid = [start_row, start_col];
    goal_grid = [goal_row, goal_col];

    % 存储找到的最佳路径和其长度
    best_path_length = inf;
    best_path_grid = [];

    % 存储每次迭代结束时的最佳路径长度
    best_path_lengths_history = zeros(num_iterations, 1);

    % 可视化设置
    figure('Name', 'ACO Path Planning Visualization');
    subplot(1, 2, 1);
    h_map = show(map);
    hold on;
    h_ants = plot([], [], 'b.', 'MarkerSize', 10); % 蚂蚁位置
    h_best_path = plot([], [], 'r-', 'LineWidth', 2); % 最佳路径
    title('ACO 搜索过程');
    xlabel('X [meters]');
    ylabel('Y [meters]');

    subplot(1, 2, 2);
    h_pheromone = imagesc(pheromone);
    axis equal;
    axis tight;
    colormap('hot');
    title('信息素分布');
    xlabel('Grid Column Index');
    ylabel('Grid Row Index');
    
    % 蚁群算法主循环
    for iter = 1:num_iterations
        % 随机初始化蚂蚁位置在起点
        ant_positions_grid = repmat(start_grid, num_ants, 1);
        current_paths_grid = cell(num_ants, 1);
        for ant = 1:num_ants
            current_paths_grid{ant} = start_grid;
        end

        % 蚂蚁移动
        max_steps = rows * cols * 2;
        for step = 1:max_steps
            ants_finished_this_step = true;
            for ant = 1:num_ants
                current_pos = ant_positions_grid(ant, :);
                if isequal(current_pos, goal_grid)
                    continue;
                end
                ants_finished_this_step = false;
                possible_moves = [];
                dr = [-1, -1, -1, 0, 0, 1, 1, 1];
                dc = [-1, 0, 1, -1, 1, -1, 0, 1];
                for move_idx = 1:length(dr)
                    next_row = current_pos(1) + dr(move_idx);
                    next_col = current_pos(2) + dc(move_idx);
                    if next_row >= 1 && next_row <= rows && next_col >= 1 && next_col <= cols
                        if occupancy_matrix(next_row, next_col) < 0.5
                            if size(current_paths_grid{ant}, 1) > 1 && isequal([next_row, next_col], current_paths_grid{ant}(end-1, :))
                                continue;
                            end
                            possible_moves = [possible_moves; next_row, next_col];
                        end
                    end
                end
                if isempty(possible_moves)
                    continue;
                end
                probabilities = zeros(size(possible_moves, 1), 1);
                total_pheromone_heuristic = 0;
                for move_idx = 1:size(possible_moves, 1)
                    next_grid = possible_moves(move_idx, :);
                    p = pheromone(next_grid(1), next_grid(2))^alpha * heuristic(next_grid(1), next_grid(2))^beta;
                    probabilities(move_idx) = p;
                    total_pheromone_heuristic = total_pheromone_heuristic + p;
                end
                if total_pheromone_heuristic > 0
                    probabilities = probabilities / total_pheromone_heuristic;
                else
                    probabilities = ones(size(possible_moves, 1), 1) / size(possible_moves, 1);
                end
                cumulative_probabilities = cumsum(probabilities);
                rand_num = rand();
                next_move_idx = find(cumulative_probabilities >= rand_num, 1, 'first');
                if isempty(next_move_idx)
                    continue;
                end
                next_pos = possible_moves(next_move_idx, :);
                ant_positions_grid(ant, :) = next_pos;
                current_paths_grid{ant} = [current_paths_grid{ant}; next_pos];
                if isequal(next_pos, goal_grid)
                    path_len = 0;
                    path_coords_grid = current_paths_grid{ant};
                    if size(path_coords_grid, 1) > 1
                        for p_idx = 1:size(path_coords_grid, 1) - 1
                            cell1_world = grid2world(map, path_coords_grid(p_idx, :));
                            cell2_world = grid2world(map, path_coords_grid(p_idx+1, :));
                            path_len = path_len + norm(cell1_world - cell2_world);
                        end
                        if path_len < best_path_length
                            best_path_length = path_len;
                            best_path_grid = path_coords_grid;
                            % 更新最佳路径可视化
                            best_path_world = grid2world(map, best_path_grid);
                            set(h_best_path, 'XData', best_path_world(:, 1), 'YData', best_path_world(:, 2));
                        end
                    end
                    ant_positions_grid(ant, :) = goal_grid;
                end
            end
            all_at_goal = all(ismember(ant_positions_grid, goal_grid, 'rows'));
            if (ants_finished_this_step && step > 1) || all_at_goal
                break;
            end
            % 每10步更新一次蚂蚁位置可视化
            if mod(step, 10) == 0
                active_ants_pos_grid = ant_positions_grid(~ismember(ant_positions_grid, goal_grid, 'rows'), :);
                if ~isempty(active_ants_pos_grid)
                    ant_world_coords = grid2world(map, active_ants_pos_grid);
                    set(h_ants, 'XData', ant_world_coords(:, 1), 'YData', ant_world_coords(:, 2));
                else
                    set(h_ants, 'XData', [], 'YData', []);
                end
                drawnow;
            end
        end

        % 信息素更新
        pheromone = pheromone * (1 - evaporation_rate);
        pheromone(pheromone < 0.01 & occupancy_matrix < 0.5) = 0.01;
        for ant = 1:num_ants
            if isequal(ant_positions_grid(ant, :), goal_grid)
                path_coords_grid = current_paths_grid{ant};
                if size(path_coords_grid, 1) > 1
                    path_len = 0;
                    for p_idx = 1:size(path_coords_grid, 1) - 1
                        cell1_world = grid2world(map, path_coords_grid(p_idx, :));
                        cell2_world = grid2world(map, path_coords_grid(p_idx+1, :));
                        path_len = path_len + norm(cell1_world - cell2_world);
                    end
                    if path_len > 0
                        deposit_amount = pheromone_deposit / path_len;
                        for p_idx = 1:size(path_coords_grid, 1)
                            row = path_coords_grid(p_idx, 1);
                            col = path_coords_grid(p_idx, 2);
                            if occupancy_matrix(row, col) < 0.5
                                pheromone(row, col) = pheromone(row, col) + deposit_amount;
                            end
                        end
                    end
                end
            end
        end

        % 更新信息素可视化
        set(h_pheromone, 'CData', pheromone);
        drawnow;

        % 记录当前迭代的最佳路径长度
        best_path_lengths_history(iter) = best_path_length;
    end

    % 将最佳路径从栅格坐标转换为世界坐标
    if ~isempty(best_path_grid)
        best_path_world = grid2world(map, best_path_grid);
    else
        best_path_world = [];
    end

    % 绘制迭代曲线
    figure;
    plot(1:num_iterations, best_path_lengths_history, 'b-');
    title('最佳路径长度随迭代次数的变化');
    xlabel('迭代次数');
    ylabel('最佳路径长度');
    grid on;
end