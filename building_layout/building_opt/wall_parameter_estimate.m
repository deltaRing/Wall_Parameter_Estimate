% �Ҿ��þ��෽����̫��һֱ�۳��������õĽ��
% ��ѡ��� ��ʼ��ʼ��ѡ��� A_1 ��������
% �����еĽ���У��Ż��������ĽǶ���Ӧֵtheta, �Լ����ľ�����Ӧֵrange
% �Ż�Ŀ�꣺�Ӹ���Ȥ�Ĺ۲����У�
% �õ�������Ӧֵ
% ���룺
% 1. �۲⵽������ observed_data ��Ҫ������������ģ������������ (N x 3)�� M��� N���ٷ�����
% �����
% 1. ����Ȥ��ǽ����ʼ��        %    #######
% 2. ����Ȥ��ǽ�������        %  #######
% 3. �����Ӧ��ǽ��Ƕ�        %   ####
% 4. �����Ӧ��ǽ���������    %    ##

function [wall_start, wall_end, max_angle, max_range] = ...
    wall_parameter_estimate(observed_data, observed_num)
    % ���Ʋ���
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