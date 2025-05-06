function [path_world, path_length, search_history] = a_star_search(map, startLocation, endLocation, varargin)
    % A* 算法路径规划
    % 输入:
    %   map - 占据栅格地图对象
    %   startLocation - 起点在世界坐标系中的位置 [x, y]
    %   endLocation - 终点在世界坐标系中的位置 [x, y]
    %   varargin - 可选参数：'heuristic_type', 'allow_diagonal'
    % 输出:
    %   path_world - 路径在世界坐标系中的点序列
    %   path_length - 路径的长度
    %   search_history - 保留字段（兼容接口）

    % --------------------- 参数解析 ---------------------
    p = inputParser;
    addParameter(p, 'heuristic_type', 'euclidean', @ischar);
    addParameter(p, 'allow_diagonal', true, @islogical);
    parse(p, varargin{:});
    
    heuristic_type = p.Results.heuristic_type;
    allow_diagonal = p.Results.allow_diagonal;

    % --------------------- 地图信息 ---------------------
    gridSize = map.GridSize;
    rows = gridSize(1);
    cols = gridSize(2);
    occupancy_matrix = occupancyMatrix(map); % 获取占据栅格矩阵

    % 转换坐标到栅格
    start_grid = world2grid(map, startLocation);
    goal_grid = world2grid(map, endLocation);
    start_row = start_grid(1); start_col = start_grid(2);
    goal_row = goal_grid(1); goal_col = goal_grid(2);

    % --------------------- 初始化数据结构 ---------------------
    [g, h, f] = deal(Inf(rows, cols));       % 代价矩阵
    parent = cell(rows, cols);               % 父节点指针
    inOpen = false(rows, cols);              % 开放列表标记
    inClosed = false(rows, cols);            % 关闭列表标记
    
    % 初始化起点
    g(start_row, start_col) = 0;
    h(start_row, start_col) = heuristic(start_row, start_col, goal_row, goal_col, heuristic_type);
    f(start_row, start_col) = g(start_row, start_col) + h(start_row, start_col);
    parent{start_row, start_col} = [NaN, NaN]; % 起点无父节点
    openList = [start_row, start_col, f(start_row, start_col)]; % 开放列表初始化

    % --------------------- 可视化初始化 ---------------------
    figure('Name', 'A* Path Planning');
    show(map); hold on;
    h_start = plot(startLocation(1), startLocation(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    h_goal = plot(endLocation(1), endLocation(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    h_open = plot([], [], 'g.', 'MarkerSize', 8);
    h_closed = plot([], [], 'y.', 'MarkerSize', 8);
    h_current = plot([], [], 'r*', 'MarkerSize', 12);
    h_path = plot([], [], 'b-', 'LineWidth', 2);
    legend([h_start, h_goal, h_open, h_closed, h_current, h_path],...
        {'Start','Goal','Open','Closed','Current','Path'});

    path_found = false;
    path_grid = []; % 关键修复：初始化 path_grid 为空数组

    % --------------------- A*主循环 ---------------------
    while ~isempty(openList)
        % 获取当前节点（开放列表中f值最小的节点）
        [~, idx] = min(openList(:,3));
        current = openList(idx,:);
        openList(idx,:) = []; % 从开放列表移除
        current_row = current(1); current_col = current(2);
        
        % 跳过已处理的节点
        if inClosed(current_row, current_col), continue; end
        inClosed(current_row, current_col) = true; % 标记为已关闭
        
        % 检查是否到达目标点
        if isequal([current_row, current_col], [goal_row, goal_col])
            path_grid = backtrack(parent, goal_row, goal_col, start_row, start_col);
            path_found = true;
            break; % 提前退出循环
        end
        
        % --------------------- 生成邻居节点 ---------------------
        if allow_diagonal
            moves = [-1 -1; -1 0; -1 1; 0 -1; 0 1; 1 -1; 1 0; 1 1]; % 8邻域
        else
            moves = [-1 0; 1 0; 0 -1; 0 1]; % 4邻域
        end
        
        for k = 1:size(moves,1)
            nr = current_row + moves(k,1); % 邻居行坐标
            nc = current_col + moves(k,2); % 邻居列坐标
            
            % 验证邻居是否有效（在栅格范围内且未被占据）
            if nr < 1 || nr > rows || nc < 1 || nc > cols, continue; end
            if occupancy_matrix(nr, nc) > 0.5, continue; end % 占据栅格跳过
            
            % 计算移动代价（对角线移动为√2，直线移动为1）
            move_cost = norm(moves(k,:));
            tentative_g = g(current_row, current_col) + move_cost;
            
            % 如果新路径更优，则更新邻居信息
            if tentative_g < g(nr, nc) || ~inOpen(nr, nc)
                parent{nr, nc} = [current_row, current_col];
                g(nr, nc) = tentative_g;
                h(nr, nc) = heuristic(nr, nc, goal_row, goal_col, heuristic_type);
                f(nr, nc) = g(nr, nc) + h(nr, nc);
                
                % 将邻居加入开放列表
                if ~inOpen(nr, nc)
                    openList = [openList; nr nc f(nr, nc)];
                    inOpen(nr, nc) = true;
                end
            end
        end
        
        % --------------------- 更新可视化 ---------------------
        [open_nodes, closed_nodes] = deal(openList(:,1:2), find(inClosed));
        [closed_rows, closed_cols] = ind2sub([rows, cols], closed_nodes);
        
        % 关键修复：根据是否找到路径传递数据
        if path_found
            update_visualization(map, open_nodes, [closed_rows, closed_cols],...
                [current_row, current_col], path_found, path_grid);
        else
            update_visualization(map, open_nodes, [closed_rows, closed_cols],...
                [current_row, current_col], path_found, []);
        end
        drawnow limitrate; % 限制刷新率以提高性能
    end
    
    % --------------------- 结果处理 ---------------------
    if path_found
        path_world = grid2world(map, path_grid); % 转换为世界坐标
        path_length = sum(vecnorm(diff(path_world), 2)); % 计算路径长度
    else
        path_world = [];
        path_length = Inf;
    end
    search_history = []; % 保留字段（兼容性）
end

% --------------------- 辅助函数 ---------------------
function update_visualization(map, open, closed, current, path_found, path)
    % 更新可视化状态
    open_world = grid2world(map, open);    % 开放列表世界坐标
    closed_world = grid2world(map, closed); % 关闭列表世界坐标
    current_world = grid2world(map, current); % 当前节点世界坐标
    
    % 更新图形对象
    set(findobj('Type','Line','Marker','.'), 'XData', open_world(:,1), 'YData', open_world(:,2));
    set(findobj('Type','Line','Marker','.','Color','y'), 'XData', closed_world(:,1), 'YData', closed_world(:,2));
    set(findobj('Type','Line','Marker','*'), 'XData', current_world(1), 'YData', current_world(2));
    
    % 仅在找到路径时更新路径显示
    if path_found && ~isempty(path)
        path_world = grid2world(map, path);
        set(findobj('Type','Line','Color','b'), 'XData', path_world(:,1), 'YData', path_world(:,2));
    end
end

function h = heuristic(row, col, gr, gc, type)
    % 计算启发式函数值
    dx = abs(col - gc); % 列差（对应x轴）
    dy = abs(row - gr); % 行差（对应y轴）
    switch type
        case 'euclidean'
            h = sqrt(dx^2 + dy^2); % 欧氏距离
        case 'manhattan'
            h = dx + dy; % 曼哈顿距离
        case 'diagonal'
            h = max(dx, dy) + (sqrt(2)-1)*min(dx, dy); % 对角线优化距离
        otherwise
            error('未知启发式类型: %s', type);
    end
end

function path = backtrack(parent, gr, gc, sr, sc)
    % 从目标点回溯路径
    path = [];
    cr = gr; cc = gc; % 当前节点从目标开始
    while ~(cr == sr && cc == sc) % 回溯到起点时停止
        path = [cr cc; path]; % 头部插入保持顺序
        pr = parent{cr, cc}(1);
        pc = parent{cr, cc}(2);
        if isempty(pr) % 父节点不存在时中断（防止死循环）
            path = [];
            return;
        end
        cr = pr; cc = pc;
    end
    path = [sr sc; path]; % 添加起点
end