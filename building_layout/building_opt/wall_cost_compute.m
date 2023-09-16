% 输入1：初值 (应该是：2 x 2)
% 输入2：RA地图
% 输出：得到的值
function value = wall_cost_compute(init_value, detect_ra_result)
    init_v1 = init_value(1,:);
    init_v2 = init_value(2,:);
    % 取得数据
    
    cost_inside_clutter = 0;
    res1 = sum(abs(detect_ra_result - init_v1), 2);
    res2 = sum(abs(detect_ra_result - init_v2), 2);
    % 如果有值小于阈值 说明在聚类内部
    if isempty(find(res1 < 0.1))
        % 取得最小的值
        cost_inside_clutter = cost_inside_clutter + min(res1);
    end
    % 
    if isempty(find(res2 < 0.1))
        % 取得最小的值
        cost_inside_clutter = cost_inside_clutter + min(res2);
    end
    
    % 计算init_v1 和 init_v2的距离
    dis = abs(init_v2(1) - init_v1(1));
    % 计算init_v1 和 init_v2的角度
    angle = abs(init_v2(2) - init_v1(2));
    % 计算init_v1 和 init_v2的周边的点数目
    respond_v1 = length(find(abs(init_v1(1) - detect_ra_result(:,1)) < 0.5));
    respond_v2 = length(find(abs(init_v2(1) - detect_ra_result(:,1)) < 0.5));
    respond    = respond_v1 + respond_v2;
    
    value = -dis * 10.0 - angle * 50.0 - respond * 10.0 + cost_inside_clutter * 1000.0;
end

