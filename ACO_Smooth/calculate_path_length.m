function length = calculate_path_length(path)
    % 计算路径长度（欧氏距离累加）
    if isempty(path) || size(path,1) < 2
        length = 0;
        return;
    end
    diff_coords = diff(path); % 相邻点坐标差
    segment_lengths = sqrt(sum(diff_coords.^2, 2));
    length = sum(segment_lengths);
end
