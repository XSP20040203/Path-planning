function smooth_path = smooth_path_bspline(raw_path, map)
    % 输入检查
    if isempty(raw_path)
        smooth_path = [];
        return;
    end
    
    % 插值增加路径点密度
    interp_factor = 5;
    t_original = linspace(0, 1, size(raw_path,1))';
    t_interp = linspace(0, 1, interp_factor * size(raw_path,1))';
    dense_path_x = interp1(t_original, raw_path(:,1), t_interp, 'pchip');
    dense_path_y = interp1(t_original, raw_path(:,2), t_interp, 'pchip');
    dense_path = [dense_path_x, dense_path_y];
    
    % B样条平滑
    n_control_points = min(20, floor(size(dense_path,1)/2));
    knots = linspace(0, 1, n_control_points);
    sp = spaps(knots, dense_path', 0.01); % 调整0.01控制平滑度
    smooth_path_temp = fnval(sp, linspace(0,1,100))';
    
    % 碰撞检测与修正
    if check_collision(smooth_path_temp, map)
        % 简单修正示例：若碰撞，二次平滑
        sp = spaps(knots, dense_path', 0.1); % 增强平滑度
        smooth_path = fnval(sp, linspace(0,1,100))';
    else
        smooth_path = smooth_path_temp;
    end
end

function collision = check_collision(path, map)
    % 路径点碰撞检测
    collision = false;
    for i = 1:size(path,1)
        grid_coords = world2grid(map, path(i,:));
        if checkOccupancy(map, grid_coords) == 1
            collision = true;
            break;
        end
    end
end
