% wallDistEstimate: 墙壁距离估计
% 输入：range_profile 距离像
% 输入：drone_height 无人机高度 (平均后)
% 输入：range_index 距离单元
% 输出：无人机到墙壁的距离
function dist = wallDistEstimate(range_profile, drone_height, range_index)
    test_cell_num = 16;   % 测试单元
    gain          = 0.05; % 增益值
    L             = 4;    % 排序最低接受单元
    H             = 12;   % 排序最高接受单元
    ROI           = [];   % 感兴趣的距离
    VOI           = [];   % 感兴趣的距离单元所对应的值
    select_index  = 5;    % 选取的距离单元
    for tt = 1:size(range_profile, 2)
        result_oscfar = OSCFAR_RangeProfile(range_profile(:, tt), ... 
            test_cell_num, gain, L, H);      % 有序CFAR结果
        [value, index] = sort(abs(result_oscfar)); % 对数据进行排序 只选择几个最高幅度的点
        % 选择距离单元
        index = index(end-select_index: end) + test_cell_num / 2;
        value = value(end-select_index: end);
        % 计算无人机到墙壁的直线距离
        prior = range_index(index).^2 - drone_height^2;
        value = value(find(prior > 0));
        ranges = sqrt(prior(find(prior > 0)));
        % 记录距离像感兴趣的单元
        ROI = [ROI ranges];
        VOI = [VOI value'];
    end

    dist = 0;
    threshold_VOI = mean(VOI);
    threshold_ROI = mean(ROI);
    great_ROI = find(ROI > threshold_ROI * 3.5);
    minor_ROI = find(ROI < drone_height * 0.5);
    noise_VOI = find(VOI < threshold_VOI * 0.25);
    remove_ROI = unique(sort([great_ROI minor_ROI noise_VOI]));
    ROI(remove_ROI) = [];
    % 除去异常值
    
    % GMM 滤波
    g_mu = drone_height;
    g_var = 1.0;
    g_w = 0.5;
    w_mu = mean(ROI);
    w_var = 1.0;
    w_w = 0.5;

    for ii = 1:10
        if g_var < 1e-4 || w_var < 1e-4
            break
        end
        [g_mu_n, g_var_n, g_w_n, w_mu_n, w_var_n, w_w_n] = GMM_estimate_wallDis(ROI, g_mu, g_var, g_w, w_mu, w_var, w_w);
        if abs(g_mu_n - g_mu) < 1e-3 || abs(g_mu_n - g_mu) < 1e-3
            break
        end
        g_mu = g_mu_n; g_var = g_var_n; g_w = g_w_n;
        w_mu = w_mu_n; w_var = w_var_n; w_w = w_w_n;
        % 更新数值
    end
    dist = w_mu;
end

