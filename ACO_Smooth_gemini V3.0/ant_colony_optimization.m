function [smoothed_path_world, smoothed_path_length, best_path_lengths_history] = ant_colony_optimization(map, startLocation, endLocation, varargin)
    % 蚁群算法路径规划 (带路径平滑，确保防碰撞，碰撞时重新优化)
    % 输入:
    %    map - 占据栅格地图对象
    %    startLocation - 起点在世界坐标系中的位置 [x, y]
    %    endLocation - 终点在世界坐标系中的位置 [x, y]
    %    varargin - 可选参数，包括 num_ants, num_iterations, alpha, beta, evaporation_rate, pheromone_deposit, smooth_iterations, sample_resolution
    % 输出:
    %    smoothed_path_world - 平滑后的最佳路径在世界坐标系中的点序列
    %    smoothed_path_length - 平滑后的最佳路径的长度
    %    best_path_lengths_history - 每次迭代的最佳原始路径长度历史记录

    % 默认参数
    default_num_ants = 200;
    default_num_iterations = 200;
    default_alpha = 1.2;
    default_beta = 14;
    default_evaporation_rate = 0.1;
    default_pheromone_deposit = 6;
    default_smooth_iterations = 10; % 默认平滑迭代次数
    default_sample_resolution = map.Resolution / 10; % 更精细的采样分辨率

    % 解析可选参数
    p = inputParser;
    addParameter(p, 'num_ants', default_num_ants, @isnumeric);
    addParameter(p, 'num_iterations', default_num_iterations, @isnumeric);
    addParameter(p, 'alpha', default_alpha, @isnumeric);
    addParameter(p, 'beta', default_beta, @isnumeric);
    addParameter(p, 'evaporation_rate', default_evaporation_rate, @isnumeric);
    addParameter(p, 'pheromone_deposit', default_pheromone_deposit, @isnumeric);
    addParameter(p, 'smooth_iterations', default_smooth_iterations, @isnumeric);
    addParameter(p, 'sample_resolution', default_sample_resolution, @isnumeric);
    parse(p, varargin{:});

    % 参数赋值
    num_ants = p.Results.num_ants;
    num_iterations = p.Results.num_iterations;
    alpha = p.Results.alpha;
    beta = p.Results.beta;
    evaporation_rate = p.Results.evaporation_rate;
    pheromone_deposit = p.Results.pheromone_deposit;
    smooth_iterations = p.Results.smooth_iterations;
    sample_resolution = p.Results.sample_resolution;

    % 输入验证
    validateattributes(startLocation, {'numeric'}, {'numel', 2}, 'startLocation');
    validateattributes(endLocation, {'numeric'}, {'numel', 2}, 'endLocation');
    if ~isValidMapPosition(map, startLocation) || ~isValidMapPosition(map, endLocation)
        error('起点或终点位置超出地图范围或位于障碍物中。');
    end

    % 地图参数初始化
    gridSize = map.GridSize;
    rows = gridSize(1);
    cols = gridSize(2);
    occupancy_matrix = occupancyMatrix(map);

    % 初始化信息素矩阵
    pheromone = ones(rows, cols);
    pheromone(occupancy_matrix >= 0.5) = 0;

    % 启发信息矩阵
    [cols_idx, rows_idx] = meshgrid(1:cols, 1:rows);
    grid_indices_all = [rows_idx(:), cols_idx(:)];
    world_coords_all = grid2world(map, grid_indices_all);
    dist_to_goal_all = sqrt(sum((world_coords_all - endLocation).^2, 2));
    heuristic = reshape(1./(dist_to_goal_all + eps), rows, cols);

    % 转换坐标到栅格
    start_grid = world2grid(map, startLocation);
    goal_grid = world2grid(map, endLocation);

    % 最佳路径初始化
    best_path_length_raw = inf;
    best_path_grid_raw = [];
    best_path_lengths_history = zeros(num_iterations, 1);
    smoothed_path_lengths_history = zeros(num_iterations, 1); % 记录平滑路径长度历史

    % 可视化初始化
    fig1 = figure('Name', 'DFA 路径规划可视化');
    h_map = show(map);
    hold on;
    plot(startLocation(1), startLocation(2), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8);
    plot(endLocation(1), endLocation(2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
    h_best_raw_path = plot(NaN, NaN, 'r--', 'LineWidth', 1.5);
    h_smoothed_path = plot(NaN, NaN, 'g-', 'LineWidth', 2);
    title('ACO 搜索过程与路径');
    legend('起点', '终点', 'ACO 原始路径', '平滑路径', 'Location', 'northwest');
    axis equal;

    fig2 = figure('Name', '信息素分布');
    h_pheromone = imagesc(pheromone);
    colormap('hot');
    colorbar;
    title('信息素分布');
    axis tight;

    % 蚁群算法主循环
    for iter = 1:num_iterations
        % 蚂蚁移动初始化
        ant_positions_grid = repmat(start_grid, num_ants, 1);
        current_paths_grid = arrayfun(@(~) start_grid, 1:num_ants, 'UniformOutput', false)';

        % 蚂蚁移动过程
        for step = 1:(rows*cols*2)
            all_ants_at_goal = true;
            for ant = 1:num_ants
                current_pos = ant_positions_grid(ant, :);
                if isequal(current_pos, goal_grid)
                    continue;
                end
                all_ants_at_goal = false;

                % 生成可能移动方向 (8方向)
                [dr, dc] = meshgrid(-1:1, -1:1);
                dr = dr(:); dc = dc(:);
                possible_moves = [current_pos(1)+dr, current_pos(2)+dc];
                valid_moves = possible_moves(all(possible_moves >= 1 & possible_moves <= [rows, cols], 2), :);
                valid_moves = valid_moves(occupancy_matrix(sub2ind([rows, cols], valid_moves(:,1), valid_moves(:,2))) < 0.5, :);

                % 概率计算
                if isempty(valid_moves)
                    continue;
                end
                pheromone_values = pheromone(sub2ind([rows, cols], valid_moves(:,1), valid_moves(:,2)));
                heuristic_values = heuristic(sub2ind([rows, cols], valid_moves(:,1), valid_moves(:,2)));
                probabilities = (pheromone_values.^alpha) .* (heuristic_values.^beta);
                probabilities = probabilities / sum(probabilities);

                % 轮盘赌选择
                selected_idx = find(cumsum(probabilities) >= rand(), 1);
                if isempty(selected_idx)
                    continue;
                end
                new_pos = valid_moves(selected_idx, :);
                ant_positions_grid(ant, :) = new_pos;
                current_paths_grid{ant} = [current_paths_grid{ant}; new_pos];
            end
            if all_ants_at_goal
                break;
            end
        end

        % 信息素蒸发与沉积
        pheromone = pheromone * (1 - evaporation_rate);
        pheromone(occupancy_matrix >= 0.5) = 0;
        for ant = 1:num_ants
            path = current_paths_grid{ant};
            if ~isequal(path(end,:), goal_grid)
                continue;
            end
            path_len = sum(vecnorm(diff(grid2world(map, path)), 2, 2));
            if path_len > 0
                deposit = pheromone_deposit / path_len;
                valid_cells = sub2ind(size(pheromone), path(:,1), path(:,2));
                pheromone(valid_cells) = pheromone(valid_cells) + deposit;
            end
        end
        pheromone = max(pheromone, 0.001); % 维持最小信息素

        % 更新最佳路径
        valid_paths = current_paths_grid(ismember(ant_positions_grid, goal_grid, 'rows'));
        if ~isempty(valid_paths)
            path_lengths = cellfun(@(p) sum(vecnorm(diff(grid2world(map,p)),2,2)), valid_paths);
            [current_best, idx] = min(path_lengths);
            if current_best < best_path_length_raw
                best_path_length_raw = current_best;
                best_path_grid_raw = valid_paths{idx};
            end
        end
        best_path_lengths_history(iter) = best_path_length_raw;
        fprintf('迭代 %d/%d, 当前最佳原始路径长度: %.2f\n', iter, num_iterations, best_path_length_raw);

        % 更新可视化
        set(h_pheromone, 'CData', pheromone);
        figure(fig2);
        title(sprintf('信息素分布 (迭代 %d)', iter));
        if ~isempty(best_path_grid_raw)
            best_path_world_raw = grid2world(map, best_path_grid_raw);
            set(h_best_raw_path, 'XData', best_path_world_raw(:,1), 'YData', best_path_world_raw(:,2));
        end
        figure(fig1);
        title(sprintf('ACO 搜索过程与路径 (迭代 %d)', iter));

        % 路径平滑与重新优化
        if ~isempty(best_path_grid_raw)
            max_attempts = 3; % 最多尝试3次重新平滑
            attempt = 1;
            current_smooth_iterations = smooth_iterations;
            current_sample_resolution = sample_resolution;
            collision_free = false;

            while attempt <= max_attempts && ~collision_free
                smoothed_path = smoothPath(map, best_path_grid_raw, current_smooth_iterations, current_sample_resolution);
                if ~isequal(smoothed_path(end,:), goal_grid)
                    smoothed_path(end+1,:) = goal_grid;
                end
                smoothed_path_world = grid2world(map, smoothed_path);
                smoothed_path_length = sum(vecnorm(diff(smoothed_path_world),2,2));

                % 验证平滑路径是否防碰撞
                if isPathCollisionFree(smoothed_path_world, map, current_sample_resolution/2)
                    collision_free = true;
                    smoothed_path_lengths_history(iter) = smoothed_path_length;
                    set(h_smoothed_path, 'XData', smoothed_path_world(:,1), 'YData', smoothed_path_world(:,2));
                else
                    fprintf('警告: 迭代 %d, 尝试 %d, 平滑路径发生碰撞，调整参数重新优化\n', iter, attempt);
                    % 调整平滑参数：减少迭代次数，增加采样密度
                    current_smooth_iterations = max(2, floor(current_smooth_iterations * 0.5));
                    current_sample_resolution = current_sample_resolution / 1.5;
                    attempt = attempt + 1;
                end
            end

            % 如果所有尝试失败，标记为无效
            if ~collision_free
                fprintf('错误: 迭代 %d, 多次尝试后仍无法生成无碰撞平滑路径\n', iter);
                smoothed_path_world = grid2world(map, best_path_grid_raw); % 回退到原始路径
                smoothed_path_length = best_path_length_raw;
                smoothed_path_lengths_history(iter) = inf;
                set(h_smoothed_path, 'XData', smoothed_path_world(:,1), 'YData', smoothed_path_world(:,2));
            end
        else
            smoothed_path_world = [];
            smoothed_path_length = inf;
            smoothed_path_lengths_history(iter) = inf;
        end

        drawnow;

        % 保存可视化 (仅在特定迭代)
        if iter == 1 || mod(iter, 10) == 0
            saveas(fig1, sprintf('ACO_search_iter%d.png', iter));
            saveas(fig2, sprintf('Pheromone_dist_iter%d.png', iter));
        end
    end

    % 最终可视化
    hold off;

    % 绘制优化过程（原始路径长度）
    figure('Name', '优化过程表现');
    plot(best_path_lengths_history, 'LineWidth', 2, 'Color', 'b');
    title('最佳原始路径长度随迭代变化');
    xlabel('迭代次数');
    ylabel('路径长度');
    grid on;
    saveas(gcf, 'Best_path_length_history.png');

    % 绘制平滑路径长度优化过程
    figure('Name', '平滑路径长度优化过程');
    valid_indices = smoothed_path_lengths_history ~= inf; % 过滤掉无效值
    plot(find(valid_indices), smoothed_path_lengths_history(valid_indices), 'LineWidth', 2, 'Color', 'g');
    title('平滑路径长度随迭代变化');
    xlabel('迭代次数');
    ylabel('平滑路径长度');
    grid on;
    saveas(gcf, 'Smoothed_path_length_history.png');

    % 辅助函数：验证地图位置
    function valid = isValidMapPosition(map, pos)
        grid_pos = world2grid(map, pos);
        valid = all(grid_pos >= 1) && all(grid_pos <= map.GridSize) && ...
                checkOccupancy(map, pos) < 0.5;
    end

    % 辅助函数：路径平滑
    function smoothed_path = smoothPath(map, path, smooth_iterations, sample_resolution)
        smoothed_path = path;
        for k = 1:smooth_iterations
            new_path = smoothed_path(1,:);
            i = 1;
            while i < size(smoothed_path,1)
                j = size(smoothed_path,1);
                while j > i+1
                    p1 = grid2world(map, smoothed_path(i,:));
                    p2 = grid2world(map, smoothed_path(j,:));
                    if ~isLineOccupied(p1, p2, map, sample_resolution)
                        new_path = [new_path; smoothed_path(j,:)];
                        i = j;
                        break;
                    end
                    j = j - 1;
                end
                if j == i+1
                    new_path = [new_path; smoothed_path(i+1,:)];
                    i = i + 1;
                end
            end
            smoothed_path = new_path;
        end
    end

    % 辅助函数：检查路径占用
    function occupied = isLineOccupied(p1, p2, map, sample_resolution)
        dist = norm(p2 - p1);
        num_samples = max(2, ceil(dist/sample_resolution)+1);
        points = [linspace(p1(1), p2(1), num_samples)', linspace(p1(2), p2(2), num_samples)'];
        occupied = any(checkOccupancy(map, points) > 0.5);
    end

    % 辅助函数：验证整个路径是否无碰撞
    function collision_free = isPathCollisionFree(path_world, map, sample_resolution)
        collision_free = true;
        for i = 1:size(path_world,1)-1
            p1 = path_world(i,:);
            p2 = path_world(i+1,:);
            if isLineOccupied(p1, p2, map, sample_resolution)
                collision_free = false;
                break;
            end
        end
    end
end