% observe函数通过观测出x-means的聚类
% 并由此继续提供合理的结果
% 首先观测函数需要做到：
% 1. 连续的距离单元、相差不大的角度群体 肯定是同一个聚类群体
% 2. 速度单元也可以作为一个滤除指标，如果针对一个随机0均值晃动的无人机，
%   该目标只有一边的速度单元存在检测结果，明显不是墙体目标
% 3. 角度单元以及距离单元相差过大的聚类单元，很明显不是墙体目标
% 4. 分布不连续的目标单元很明显也不是墙体目标
% 5. 进行线性组合 找到 墙壁分布最远的位置
% 输入：1. X-means 聚类结果
% 输入：2. X-means 聚类中心
% 输入：3. range_threshold 距离聚类阈值
% 输入：4. angle_threshold 角度聚类阈值
% 输入：5. select_velocity 限制了的速度
% 输出：1. X1 墙体起始坐标
% 输出：2. X2 墙体结束坐标
% 最后利用GMM高斯混合聚类 估计出墙壁的大小
function [X1, X2] = observe(X_cluster, X_mean, ...
    X1_past, X2_past, ...
    range_threshold, ...
    angle_threshold, select_velocity, ...
    select_range, select_angle)
    if nargin == 4, 
        % 规定：距离相同阈值是2.0m 角度是 pi / 6 rad
        % 规定：距离相同阈值是0.5m 角度是 pi / 4 rad
        range_threshold = [2.0 0.5];
        angle_threshold = [pi / 6 pi / 4];
        % 规定：
        select_velocity = 0.35;
        % 规定：
        select_angle = pi / 18;
        select_range = 0.5;
    end
    X1 = [];
    X2 = [];
    % 遍历数据，确定数据是无法进一步切割分类了
    new_X_cluster = [];
    new_X_mean    = [];
    new_X_index   = 1;
    % 数据还能继续分子类的话
    for ii = 1:size(X_mean, 1)
        data_scatter = X_cluster(find(X_cluster(:,4)==ii), 1:3);
        detect_results = data_scatter(:,1:3);        % 滤除噪声后的数据 
        [center, data_new] = xmeans(detect_results, 20);
        for jj = 1:size(center, 1)
            new_X_mean    = [new_X_mean; center(jj,:)];
            data_new1     = data_new(find(data_new(:, 4) == jj), 1:3);
            data_new1_num = size(data_new1, 1);
            new_X_cluster = [new_X_cluster; ...
                [data_new1 ones(data_new1_num, 1) * new_X_index]];
            new_X_index = new_X_index + 1;
        end
    end
    X_cluster = new_X_cluster;
    X_mean    = new_X_mean;
    
    % 遍历 X_mean 聚类中心
    % 如果相对角度以及相对距离小于一定阈值 那么就是相同的聚类
    % 取出每个聚类中心，不要速度中心
    remain_index          = [];                         % 记录对应的index
    X_center_without_velo = X_mean(:,[1,3]);            % 去除index
    Flag_center_record    = zeros(1, size(X_mean, 1));  % 记录被融合的数据
    new_center        = [];                             % 新的聚类中心
    new_cluster       = [];                             % 新的聚类数据
    new_cluster_index = 0;                              % 标记对应的数据
    % 数据 1： 距离   数据 2： 角度
    for ii = 1:size(X_center_without_velo, 1)
        if Flag_center_record(ii), continue, end
        origin_center = X_center_without_velo(ii, :);
        Flag_center_record(ii) = 1;                     % 置1 
        fused_index   = [ii];
        for iii = 1:size(X_center_without_velo, 1)
            % 检查数据
            if ii == iii, continue, end
            if Flag_center_record(iii), continue, end
            select_center = X_center_without_velo(iii, :);
            % 被选取的数据
            diff          = abs(origin_center - select_center); % 算出差值
            angle_diff    = abs(diff(2));
            range_diff    = abs(diff(1));
            % 判断是否是同一类数据
            % 情况1 角度偏差不大 但是距离相差相对较远
            if angle_diff < angle_threshold(1) && range_diff < range_threshold(1)
                fused_index = [fused_index iii];
                Flag_center_record(iii) = 1;             % 置1 
            end
            % 情况2 角度偏差较大 但是距离极为接近
            if angle_diff < angle_threshold(2) && range_diff < range_threshold(2)
                fused_index = [fused_index iii];
                Flag_center_record(iii) = 1;             % 置1 
            end
        end
        new_cluster_index = new_cluster_index + 1; 
        % 融合数据
        if ~isempty(fused_index)
            index = [];
            for tt = 1:length(fused_index)
               index_t = find(X_cluster(:, 4) == fused_index(tt));
               index = [index; index_t];
            end
            % 取出聚类数据
            temp_cluster = X_cluster(index, 1:3);
            new_cluster  = [new_cluster; [temp_cluster new_cluster_index ...
                * ones(length(index), 1)]];
            remain_index = [remain_index new_cluster_index];
            new_center   = [new_center; mean(temp_cluster, 1)];
        end
    end
    % 融合同一聚类群体
    new_cluster_var     = [];                           % 新的数据方差
    mean_cluster_var    = [];                           % 平均的数据方差  
    delta_cluster_range = [];                           % 每个聚类的距离差值
    delta_cluster_angle = [];                           % 每个聚类的角度差值
    for ii = 1:size(new_center, 1)
        center        = new_center(ii, :);
        t_cluster     = new_cluster(find(new_cluster(:, 4) == ii), 1:3);
        range_cluster = t_cluster(:, 1);                      % 聚类的距离
        angle_cluster = t_cluster(:, 3);                      % 聚类的角度
        t_r_var       = sum(abs(t_cluster(:,1) - center(1))); % 计算距离方差
        t_v_var       = sum(abs(t_cluster(:,2) - center(2))); % 计算速度方差
        t_a_var       = sum(abs(t_cluster(:,3) - center(3))); % 计算角度方差
        m_r_var       = t_r_var / size(new_cluster, 1);
        m_v_var       = t_v_var / size(new_cluster, 1);
        m_a_var       = t_a_var / size(new_cluster, 1);
        new_cluster_var = [new_cluster_var; t_r_var t_v_var t_a_var];
        mean_cluster_var = [mean_cluster_var; m_r_var m_v_var m_a_var];
        delta_cluster_range = [delta_cluster_range max(range_cluster) - min(range_cluster)];
        delta_cluster_angle = [delta_cluster_angle max(angle_cluster) - min(angle_cluster)];
    end
   
    % 可以把错误聚类结果给纠正回来
    if 0
       figure(10021)
        for ii = 1:size(new_center, 1)
            data_scatter = new_cluster(find(new_cluster(:,4)==ii), 1:3);
            scatter3(data_scatter(:,1),data_scatter(:,2),data_scatter(:,3), 5)
            hold on
        end
        hold off
        title("Corrected cluster")
        xlabel("Range (meter)")
        ylabel("Velocity (m/s)")
        zlabel("Angle (rad)")
    end 
    
    % 剔除速度分布明显不均的数据
    for ii = size(new_center, 1):-1:1
        velo = new_center(ii, 2);
        if select_velocity < abs(velo),
            remain_index(ii)        = [];
            new_center(ii, :)       = [];
            new_cluster_var(ii, :)  = [];
            mean_cluster_var(ii, :) = [];
            delta_cluster_range(ii) = [];
            delta_cluster_angle(ii) = [];
            new_cluster(find(new_cluster(:, 4) == ii),:) = [];
        end
    end
    
    % 剔除距离、角度分布明显较小的数据
    for ii = length(delta_cluster_range):-1:1
        if delta_cluster_range(ii) >= select_range && ...
                delta_cluster_angle(ii) >= select_angle
            continue
        else
            remain_index(ii)        = [];
            new_center(ii, :)       = [];
            new_cluster_var(ii, :)  = [];
            mean_cluster_var(ii, :) = [];
            delta_cluster_range(ii) = [];
            delta_cluster_angle(ii) = [];
            new_cluster(find(new_cluster(:, 4) == ii),:) = [];
        end
    end
    
    if isempty(new_cluster), return, end;
    
    % 根据计算阈值
    % 最后一个措施：
    % 只取得唯一一个墙体
    % 为了达到这个目的：
    % 找到符合墙体数据分布的第一条数据
    % 也就是最近的数据，其他的数据，均不考虑
    [value, index] = min(new_center(:,1));
    index          = remain_index(index);
    % 找到距离值最小、最大的单元
    ra_units = new_cluster(find(new_cluster(:, 4) == index), [1, 3]);
    ra_units = unique(ra_units, 'rows');
    
    [ra_units_dbscan, ~]       = DBSCAN(ra_units, 0.05, 5);
    [ra_units_fin, DBSCAN_num] = DBSCAN(ra_units(:,1:2), 0.75, 10);
    
    if 0
       figure(10022)
        for ii = 1:size(new_center, 1)
            data_scatter = new_cluster(find(new_cluster(:,4)==index(ii)), 1:3);
            scatter3(data_scatter(:,1),data_scatter(:,2),data_scatter(:,3), 5)
            hold on
        end
        hold off
        title("观测后滤波聚类结果")
        xlabel("距离 (meter)")
        ylabel("速度 (m/s)")
        zlabel("角度 (rad)")
        legend
        
        % 为什么要用该参数？（考虑到角度以及微小的聚集结果）
        figure(10024)
        scatter(ra_units_fin(:,1), ra_units_fin(:,2))
        
        title("Corrected cluster")
        xlabel("Range (meter)")
        ylabel("Angle (rad)")
    end
    
    [X1, X2, ~, ~] = wall_parameter_estimate(ra_units_fin, DBSCAN_num);
    
%     % 扫描每个单元
%     % 取出最小的数个单元 以及 最大的数个单元
%     unit_num = 5;
%     % 找到距离值最小最大的单元
%     % 角度-距离投影
%     % 距离角度单元阈值
%     % 最小的单元
%     range_threshold = 5; % 连续的单元
%     min_range_index = [];
%     min_range_value = [];
%     last_range_     = 0;
%     for rr = 1:size(ra_units, 1)
%         range_ = ra_units(rr, 1);
%         if last_range_ == range_
%             continue;
%         end
%         rindex = find(ra_units(:,1) == range_);
%         if length(rindex) >= range_threshold
%             min_range_index = [min_range_index rr];
%             min_range_value = [min_range_value range_];
%         end
%         if length(min_range_index) >= unit_num
%             break
%         end
%         last_range_ = range_;
%     end
%     % 找到最大的单元 该单元将会被当作是墙体结束的部分
%     max_range_index = [];
%     max_range_value = [];
%     last_range_     = 0;
%     for rr = size(ra_units, 1):-1:1
%         range_ = ra_units(rr, 1);
%         if last_range_ == range_
%             continue;
%         end
%         rindex = find(ra_units(:,1) == range_);
%         if length(rindex) >= range_threshold, 
%             max_range_index = [max_range_index rr];
%             max_range_value = [max_range_value range_];
%         end
%         if length(max_range_index) >= unit_num
%             break
%         end
%         last_range_ = range_;
%     end
%     if isempty(max_range_index) && isempty(min_range_index)
%         return;
%     end
%     
%     % 求得
%     X1_1 = []; X2_1 = [];
%     X1_2 = []; X2_2 = [];
%     X1_mean = mean(X1_past, 1);
%     X2_mean = mean(X2_past, 1);
%     if ~isempty(X1_past)
%         X1_max  = max(X1_past(:,2));
%         X1_min  = min(X1_past(:,2));
%     else
%         X1_max  = [];
%         X1_min  = [];
%     end
%     if ~isempty(X2_past)
%         X2_max  = max(X2_past(:,2));
%         X2_min  = min(X2_past(:,2));
%     else
%         X2_max  = [];
%         X2_min  = [];
%     end
%     
%     % 选择角度结果
%     for rr = 1:length(min_range_value)
%         % 检测角度结果，并进行线性组合
%         min_angle_results = (ra_units(find(ra_units(:,1) == min_range_value(rr)), 2));
%         min_min_range    = min(min_angle_results);
%         % 如果大于了平均数据的指定方差，默认是噪声数据
%         if ~isempty(X1_mean)
%             if abs(X1_mean(1) - min_range_value(rr)) > 0.3 % 相当于 var 是 0.25
%                 break
%             end
%             if abs(X1_min - min_min_range) > pi / 18
%                 continue;
%             end
%         end
%         % 该数据距离第一个数据太远了 也不被当作是真实值
%         if ~isempty(X1_1)
%             if abs(min(X1_1(:, 1)) - min_range_value(rr)) > 0.5
%                 break % 无参考性的噪声
%             end
%         end
%         X1_1 = [X1_1; min_range_value(rr), min_min_range];
%     end
%     
%     for rr = 1:length(max_range_value)
%         max_angle_results = (ra_units(find(ra_units(:,1) == max_range_value(rr)), 2));
%         max_max_range    = max(max_angle_results);
%         %
%         if ~isempty(X2_mean)
%             if abs(X2_mean(1) - max_range_value(rr)) > 0.3 % 相当于 var 是 0.09
%                 break
%             end
%             if abs(X2_max - max_max_range) > pi / 18
%                 continue;
%             end
%         end
%         % 查看数据
%         if ~isempty(X2_1)
%             if abs(max(X2_1(:, 1)) - max_range_value(rr)) > 0.5
%                 break % 无参考性的噪声
%             end
%         end
%         X2_1 = [X2_1; max_range_value(rr), max_max_range];
%     end
%     
%     for rr = 1:length(min_range_value)
%         % 检测角度结果，并进行线性组合
%         min_angle_results = (ra_units(find(ra_units(:,1) == min_range_value(rr)), 2));
%         max_min_range    = max(min_angle_results);
%         %
%         if ~isempty(X1_mean)
%             if abs(X1_mean(1) - min_range_value(rr)) > 0.3 % 相当于 var 是 0.25
%                 break
%             end
%             if abs(X1_max - max_min_range) > pi / 18
%                 continue;
%             end
%         end
%         %
%         if ~isempty(X1_2)
%             if abs(min(X1_2(:, 1)) - min_range_value(rr)) > 0.5
%                 break % 无参考性的噪声
%             end
%         end
%         X1_2 = [X1_2; min_range_value(rr), max_min_range];
%     end
%     
%     for rr = 1:length(max_range_value)
%         max_angle_results = (ra_units(find(ra_units(:,1) == max_range_value(rr)), 2));
%         min_max_range    = min(max_angle_results);
%         %
%         if ~isempty(X2_mean)
%             if abs(X2_mean(1) - max_range_value(rr)) > 0.3 % 相当于 var 是 0.25
%                 break
%             end
%             if abs(X2_min - min_max_range) > pi / 18
%                 continue;
%             end
%         end
%         % 查看数据
%         if ~isempty(X2_2)
%             if abs(max(X2_2(:, 1)) - max_range_value(rr)) > 0.5
%                 break % 无参考性的噪声
%             end
%         end
%         X2_2 = [X2_2; max_range_value(rr), min_max_range];
%     end
%     
%     % 检查X1 或者 X2 是否是空集合 如果是空集合 不返回数据
%     % 空数据无法进行角度比较
%     if isempty(X1_1) || isempty(X1_2) || isempty(X2_1) || isempty(X2_2)
%         X1 = [];
%         X2 = [];
%         return;
%     end
%     
%     if abs(mean(X2_1(:, 2)) - mean(X1_1(:, 2))) < abs(mean(X2_2(:, 2)) - mean(X1_2(:, 2))) % 计算斜率
%         X1 = X1_2; X2 = X2_2;
%     else
%         X1 = X1_1; X2 = X2_1;
%     end
end

