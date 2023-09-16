% observe����ͨ���۲��x-means�ľ���
% ���ɴ˼����ṩ����Ľ��
% ���ȹ۲⺯����Ҫ������
% 1. �����ľ��뵥Ԫ������ĽǶ�Ⱥ�� �϶���ͬһ������Ⱥ��
% 2. �ٶȵ�ԪҲ������Ϊһ���˳�ָ�꣬������һ�����0��ֵ�ζ������˻���
%   ��Ŀ��ֻ��һ�ߵ��ٶȵ�Ԫ���ڼ���������Բ���ǽ��Ŀ��
% 3. �Ƕȵ�Ԫ�Լ����뵥Ԫ������ľ��൥Ԫ�������Բ���ǽ��Ŀ��
% 4. �ֲ���������Ŀ�굥Ԫ������Ҳ����ǽ��Ŀ��
% 5. ����������� �ҵ� ǽ�ڷֲ���Զ��λ��
% ���룺1. X-means ������
% ���룺2. X-means ��������
% ���룺3. range_threshold ���������ֵ
% ���룺4. angle_threshold �ǶȾ�����ֵ
% ���룺5. select_velocity �����˵��ٶ�
% �����1. X1 ǽ����ʼ����
% �����2. X2 ǽ���������
% �������GMM��˹��Ͼ��� ���Ƴ�ǽ�ڵĴ�С
function [X1, X2] = observe(X_cluster, X_mean, ...
    X1_past, X2_past, ...
    range_threshold, ...
    angle_threshold, select_velocity, ...
    select_range, select_angle)
    if nargin == 4, 
        % �涨��������ͬ��ֵ��2.0m �Ƕ��� pi / 6 rad
        % �涨��������ͬ��ֵ��0.5m �Ƕ��� pi / 4 rad
        range_threshold = [2.0 0.5];
        angle_threshold = [pi / 6 pi / 4];
        % �涨��
        select_velocity = 0.35;
        % �涨��
        select_angle = pi / 18;
        select_range = 0.5;
    end
    X1 = [];
    X2 = [];
    % �������ݣ�ȷ���������޷���һ���и������
    new_X_cluster = [];
    new_X_mean    = [];
    new_X_index   = 1;
    % ���ݻ��ܼ���������Ļ�
    for ii = 1:size(X_mean, 1)
        data_scatter = X_cluster(find(X_cluster(:,4)==ii), 1:3);
        detect_results = data_scatter(:,1:3);        % �˳������������ 
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
    
    % ���� X_mean ��������
    % �����ԽǶ��Լ���Ծ���С��һ����ֵ ��ô������ͬ�ľ���
    % ȡ��ÿ���������ģ���Ҫ�ٶ�����
    remain_index          = [];                         % ��¼��Ӧ��index
    X_center_without_velo = X_mean(:,[1,3]);            % ȥ��index
    Flag_center_record    = zeros(1, size(X_mean, 1));  % ��¼���ںϵ�����
    new_center        = [];                             % �µľ�������
    new_cluster       = [];                             % �µľ�������
    new_cluster_index = 0;                              % ��Ƕ�Ӧ������
    % ���� 1�� ����   ���� 2�� �Ƕ�
    for ii = 1:size(X_center_without_velo, 1)
        if Flag_center_record(ii), continue, end
        origin_center = X_center_without_velo(ii, :);
        Flag_center_record(ii) = 1;                     % ��1 
        fused_index   = [ii];
        for iii = 1:size(X_center_without_velo, 1)
            % �������
            if ii == iii, continue, end
            if Flag_center_record(iii), continue, end
            select_center = X_center_without_velo(iii, :);
            % ��ѡȡ������
            diff          = abs(origin_center - select_center); % �����ֵ
            angle_diff    = abs(diff(2));
            range_diff    = abs(diff(1));
            % �ж��Ƿ���ͬһ������
            % ���1 �Ƕ�ƫ��� ���Ǿ��������Խ�Զ
            if angle_diff < angle_threshold(1) && range_diff < range_threshold(1)
                fused_index = [fused_index iii];
                Flag_center_record(iii) = 1;             % ��1 
            end
            % ���2 �Ƕ�ƫ��ϴ� ���Ǿ��뼫Ϊ�ӽ�
            if angle_diff < angle_threshold(2) && range_diff < range_threshold(2)
                fused_index = [fused_index iii];
                Flag_center_record(iii) = 1;             % ��1 
            end
        end
        new_cluster_index = new_cluster_index + 1; 
        % �ں�����
        if ~isempty(fused_index)
            index = [];
            for tt = 1:length(fused_index)
               index_t = find(X_cluster(:, 4) == fused_index(tt));
               index = [index; index_t];
            end
            % ȡ����������
            temp_cluster = X_cluster(index, 1:3);
            new_cluster  = [new_cluster; [temp_cluster new_cluster_index ...
                * ones(length(index), 1)]];
            remain_index = [remain_index new_cluster_index];
            new_center   = [new_center; mean(temp_cluster, 1)];
        end
    end
    % �ں�ͬһ����Ⱥ��
    new_cluster_var     = [];                           % �µ����ݷ���
    mean_cluster_var    = [];                           % ƽ�������ݷ���  
    delta_cluster_range = [];                           % ÿ������ľ����ֵ
    delta_cluster_angle = [];                           % ÿ������ĽǶȲ�ֵ
    for ii = 1:size(new_center, 1)
        center        = new_center(ii, :);
        t_cluster     = new_cluster(find(new_cluster(:, 4) == ii), 1:3);
        range_cluster = t_cluster(:, 1);                      % ����ľ���
        angle_cluster = t_cluster(:, 3);                      % ����ĽǶ�
        t_r_var       = sum(abs(t_cluster(:,1) - center(1))); % ������뷽��
        t_v_var       = sum(abs(t_cluster(:,2) - center(2))); % �����ٶȷ���
        t_a_var       = sum(abs(t_cluster(:,3) - center(3))); % ����Ƕȷ���
        m_r_var       = t_r_var / size(new_cluster, 1);
        m_v_var       = t_v_var / size(new_cluster, 1);
        m_a_var       = t_a_var / size(new_cluster, 1);
        new_cluster_var = [new_cluster_var; t_r_var t_v_var t_a_var];
        mean_cluster_var = [mean_cluster_var; m_r_var m_v_var m_a_var];
        delta_cluster_range = [delta_cluster_range max(range_cluster) - min(range_cluster)];
        delta_cluster_angle = [delta_cluster_angle max(angle_cluster) - min(angle_cluster)];
    end
   
    % ���԰Ѵ������������������
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
    
    % �޳��ٶȷֲ����Բ���������
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
    
    % �޳����롢�Ƕȷֲ����Խ�С������
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
    
    % ���ݼ�����ֵ
    % ���һ����ʩ��
    % ֻȡ��Ψһһ��ǽ��
    % Ϊ�˴ﵽ���Ŀ�ģ�
    % �ҵ�����ǽ�����ݷֲ��ĵ�һ������
    % Ҳ������������ݣ����������ݣ���������
    [value, index] = min(new_center(:,1));
    index          = remain_index(index);
    % �ҵ�����ֵ��С�����ĵ�Ԫ
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
        title("�۲���˲�������")
        xlabel("���� (meter)")
        ylabel("�ٶ� (m/s)")
        zlabel("�Ƕ� (rad)")
        legend
        
        % ΪʲôҪ�øò����������ǵ��Ƕ��Լ�΢С�ľۼ������
        figure(10024)
        scatter(ra_units_fin(:,1), ra_units_fin(:,2))
        
        title("Corrected cluster")
        xlabel("Range (meter)")
        ylabel("Angle (rad)")
    end
    
    [X1, X2, ~, ~] = wall_parameter_estimate(ra_units_fin, DBSCAN_num);
    
%     % ɨ��ÿ����Ԫ
%     % ȡ����С��������Ԫ �Լ� ����������Ԫ
%     unit_num = 5;
%     % �ҵ�����ֵ��С���ĵ�Ԫ
%     % �Ƕ�-����ͶӰ
%     % ����Ƕȵ�Ԫ��ֵ
%     % ��С�ĵ�Ԫ
%     range_threshold = 5; % �����ĵ�Ԫ
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
%     % �ҵ����ĵ�Ԫ �õ�Ԫ���ᱻ������ǽ������Ĳ���
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
%     % ���
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
%     % ѡ��ǶȽ��
%     for rr = 1:length(min_range_value)
%         % ���ǶȽ�����������������
%         min_angle_results = (ra_units(find(ra_units(:,1) == min_range_value(rr)), 2));
%         min_min_range    = min(min_angle_results);
%         % ���������ƽ�����ݵ�ָ�����Ĭ������������
%         if ~isempty(X1_mean)
%             if abs(X1_mean(1) - min_range_value(rr)) > 0.3 % �൱�� var �� 0.25
%                 break
%             end
%             if abs(X1_min - min_min_range) > pi / 18
%                 continue;
%             end
%         end
%         % �����ݾ����һ������̫Զ�� Ҳ������������ʵֵ
%         if ~isempty(X1_1)
%             if abs(min(X1_1(:, 1)) - min_range_value(rr)) > 0.5
%                 break % �޲ο��Ե�����
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
%             if abs(X2_mean(1) - max_range_value(rr)) > 0.3 % �൱�� var �� 0.09
%                 break
%             end
%             if abs(X2_max - max_max_range) > pi / 18
%                 continue;
%             end
%         end
%         % �鿴����
%         if ~isempty(X2_1)
%             if abs(max(X2_1(:, 1)) - max_range_value(rr)) > 0.5
%                 break % �޲ο��Ե�����
%             end
%         end
%         X2_1 = [X2_1; max_range_value(rr), max_max_range];
%     end
%     
%     for rr = 1:length(min_range_value)
%         % ���ǶȽ�����������������
%         min_angle_results = (ra_units(find(ra_units(:,1) == min_range_value(rr)), 2));
%         max_min_range    = max(min_angle_results);
%         %
%         if ~isempty(X1_mean)
%             if abs(X1_mean(1) - min_range_value(rr)) > 0.3 % �൱�� var �� 0.25
%                 break
%             end
%             if abs(X1_max - max_min_range) > pi / 18
%                 continue;
%             end
%         end
%         %
%         if ~isempty(X1_2)
%             if abs(min(X1_2(:, 1)) - min_range_value(rr)) > 0.5
%                 break % �޲ο��Ե�����
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
%             if abs(X2_mean(1) - max_range_value(rr)) > 0.3 % �൱�� var �� 0.25
%                 break
%             end
%             if abs(X2_min - min_max_range) > pi / 18
%                 continue;
%             end
%         end
%         % �鿴����
%         if ~isempty(X2_2)
%             if abs(max(X2_2(:, 1)) - max_range_value(rr)) > 0.5
%                 break % �޲ο��Ե�����
%             end
%         end
%         X2_2 = [X2_2; max_range_value(rr), min_max_range];
%     end
%     
%     % ���X1 ���� X2 �Ƿ��ǿռ��� ����ǿռ��� ����������
%     % �������޷����нǶȱȽ�
%     if isempty(X1_1) || isempty(X1_2) || isempty(X2_1) || isempty(X2_2)
%         X1 = [];
%         X2 = [];
%         return;
%     end
%     
%     if abs(mean(X2_1(:, 2)) - mean(X1_1(:, 2))) < abs(mean(X2_2(:, 2)) - mean(X1_2(:, 2))) % ����б��
%         X1 = X1_2; X2 = X2_2;
%     else
%         X1 = X1_1; X2 = X2_1;
%     end
end

