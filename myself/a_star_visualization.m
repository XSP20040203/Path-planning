function [path, path_length] = a_star_visualization(map, startLocation, endLocation)
    % A*路径规划（带可视化）
    % 输入:
    %   map - 占据栅格地图对象
    %   startLocation - 起点在世界坐标系中的位置 [x, y]
    %   endLocation - 终点在世界坐标系中的位置 [x, y]
    % 输出:
    %   path - 最短路径在世界坐标系中的点序列
    %   path_length - 最短路径的长度

    % 获取地图的栅格尺寸
    gridSize = map.GridSize;
    rows = gridSize(1);
    cols = gridSize(2);

    % 获取整个地图的占据矩阵
    occupancy_matrix = occupancyMatrix(map);

    % 获取起点和终点的栅格索引
    start_grid = world2grid(map, startLocation);
    goal_grid = world2grid(map, endLocation);

    % 初始化开放列表和封闭列表
    open_list = [];
    closed_list = false(rows, cols);

    % 初始化节点信息
    g = inf(rows, cols);
    h = inf(rows, cols);
    f = inf(rows, cols);
    parent = zeros(rows, cols, 2);

    % 设置起点
    g(start_grid(1), start_grid(2)) = 0;
    h(start_grid(1), start_grid(2)) = norm(startLocation - endLocation);
    f(start_grid(1), start_grid(2)) = g(start_grid(1), start_grid(2)) + h(start_grid(1), start_grid(2));
    open_list = [open_list; start_grid];

    % 可视化设置
    figure('Name', 'A* Path Planning Visualization');
    h_map = show(map);
    hold on;
    h_open = plot([], [], 'go', 'MarkerSize', 5); % 开放列表（绿色）
    h_closed = plot([], [], 'ro', 'MarkerSize', 5); % 封闭列表（红色）
    h_path = plot([], [], 'b-', 'LineWidth', 2); % 当前路径（蓝色）
    title('A* 搜索过程');
    xlabel('X [meters]');
    ylabel('Y [meters]');

    while ~isempty(open_list)
        % 找到f值最小的节点
        [~, idx] = min(f(sub2ind([rows, cols], open_list(:,1), open_list(:,2))));
        current = open_list(idx, :);

        % 如果当前节点是终点，停止搜索
        if isequal(current, goal_grid)
            break;
        end

        % 将当前节点移到封闭列表
        open_list(idx, :) = [];
        closed_list(current(1), current(2)) = true;

        % 扩展邻居节点
        neighbors = get_neighbors(current, rows, cols, occupancy_matrix);
        for i = 1:size(neighbors, 1)
            neighbor = neighbors(i, :);
            if closed_list(neighbor(1), neighbor(2))
                continue;
            end

            tentative_g = g(current(1), current(2)) + norm(grid2world(map, current) - grid2world(map, neighbor));

            if ~ismember(neighbor, open_list, 'rows') || tentative_g < g(neighbor(1), neighbor(2))
                parent(neighbor(1), neighbor(2), :) = current;
                g(neighbor(1), neighbor(2)) = tentative_g;
                h(neighbor(1), neighbor(2)) = norm(grid2world(map, neighbor) - endLocation);
                f(neighbor(1), neighbor(2)) = g(neighbor(1), neighbor(2)) + h(neighbor(1), neighbor(2));
                if ~ismember(neighbor, open_list, 'rows')
                    open_list = [open_list; neighbor];
                end
            end
        end

        % 更新可视化
        update_visualization(h_open, h_closed, h_path, open_list, closed_list, parent, start_grid, goal_grid, map);
        drawnow;
    end

    % 回溯路径
    path_grid = [];
    current = goal_grid;
    while ~isequal(current, start_grid)
        path_grid = [current; path_grid];
        current = squeeze(parent(current(1), current(2), :))';
    end
    path_grid = [start_grid; path_grid];

    % 转换为世界坐标
    path = grid2world(map, path_grid);

    % 计算路径长度
    path_length = 0;
    for i = 1:size(path, 1)-1
        path_length = path_length + norm(path(i, :) - path(i+1, :));
    end

    % 显示最终路径
    set(h_path, 'XData', path(:,1), 'YData', path(:,2));
    drawnow;
end

function neighbors = get_neighbors(current, rows, cols, occupancy_matrix)
    % 获取当前节点的邻居节点（8个方向）
    dr = [-1, -1, -1, 0, 0, 1, 1, 1];
    dc = [-1, 0, 1, -1, 1, -1, 0, 1];
    neighbors = [];
    for i = 1:length(dr)
        nr = current(1) + dr(i);
        nc = current(2) + dc(i);
        if nr >= 1 && nr <= rows && nc >= 1 && nc <= cols && occupancy_matrix(nr, nc) < 0.5
            neighbors = [neighbors; nr, nc];
        end
    end
end

function update_visualization(h_open, h_closed, h_path, open_list, closed_list, parent, start_grid, goal_grid, map)
    % 更新开放列表可视化
    if ~isempty(open_list)
        open_world = grid2world(map, open_list);
        set(h_open, 'XData', open_world(:,1), 'YData', open_world(:,2));
    else
        set(h_open, 'XData', [], 'YData', []);
    end

    % 更新封闭列表可视化
    [closed_rows, closed_cols] = find(closed_list);
    if ~isempty(closed_rows)
        closed_grid = [closed_rows, closed_cols];
        closed_world = grid2world(map, closed_grid);
        set(h_closed, 'XData', closed_world(:,1), 'YData', closed_world(:,2));
    else
        set(h_closed, 'XData', [], 'YData', []);
    end

    % 更新当前路径可视化（从起点到当前节点）
    current_path_grid = [];
    current = goal_grid;
    while ~isequal(current, start_grid)
        current_path_grid = [current; current_path_grid];
        if all(parent(current(1), current(2), :) == 0)
            break;
        end
        current = squeeze(parent(current(1), current(2), :))';
    end
    current_path_grid = [start_grid; current_path_grid];
    if size(current_path_grid, 1) > 1
        current_path_world = grid2world(map, current_path_grid);
        set(h_path, 'XData', current_path_world(:,1), 'YData', current_path_world(:,2));
    else
        set(h_path, 'XData', [], 'YData', []);
    end
end