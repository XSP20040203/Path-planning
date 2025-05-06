function [best_path_world, best_path_length] = aco_with_smoothing(map, startLocation, endLocation, varargin)
    % 主函数：带路径平滑的蚁群算法
    % 确保在所有分支中返回两个输出参数
    
    % 默认参数
    default_num_ants = 50;
    default_num_iterations = 100;
    
    % 解析输入参数
    p = inputParser;
    addParameter(p, 'num_ants', default_num_ants);
    addParameter(p, 'num_iterations', default_num_iterations);
    parse(p, varargin{:});
    
    % 调用蚁群算法获取原始路径
    [raw_path_world, raw_length] = ant_colony_optimization(map, startLocation, endLocation, ...
        'num_ants', p.Results.num_ants, ...
        'num_iterations', p.Results.num_iterations);
    
    % 路径平滑处理（若存在有效路径）
    if ~isempty(raw_path_world)
        smooth_path = smooth_path_bspline(raw_path_world, map);
        best_path_world = smooth_path;
        best_path_length = calculate_path_length(smooth_path);
    else
        best_path_world = [];
        best_path_length = inf;
    end
end
