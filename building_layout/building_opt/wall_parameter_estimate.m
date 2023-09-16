% 我觉得聚类方法不太能一直聚出来很良好的结果
% 候选结果 初始起始候选结果 A_1 包含噪声
% 从所有的结果中：优化出：最大的角度响应值theta, 以及最大的距离响应值range
% 优化目标：从感兴趣的观测结果中：
% 得到最大的响应值
% 输入：
% 1. 观测到的数据 observed_data （要用这个计算质心）（这个数据是 (N x 3)） M类别 N多少份数据
% 输出：
% 1. 感兴趣的墙体起始点        %    #######
% 2. 感兴趣的墙体结束点        %  #######
% 3. 最大响应的墙体角度        %   ####
% 4. 最大响应的墙体延申距离    %    ##

function [wall_start, wall_end, max_angle, max_range] = ...
    wall_parameter_estimate(observed_data, observed_num)
    % 估计参数
    wall_start = []; wall_end = []; max_angle = []; max_range = [];
    for ii = 1:observed_num
        index = find(observed_data(:,3) == ii);
        select_data = observed_data(index, 1:2);
        wall_cost = @(init_v)wall_cost_compute(init_v, select_data);
        [Vopt, Cost] = fminsearch(wall_cost, ...
            [select_data(1,:); select_data(end,:)]);
        wallStart = Vopt(1,:); wallEnd = Vopt(2,:);
        maxAngle = wallEnd(1, 2) - wallStart(1, 2);
        maxRange = wallEnd(1, 1) - wallStart(1, 1);
        wall_start = [wall_start; wallStart];
        wall_end   = [wall_end; wallEnd];
        max_angle  = [max_angle; maxAngle];
        max_range  = [max_range; maxRange]; 
    end
    
end