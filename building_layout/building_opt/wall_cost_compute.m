% ����1����ֵ (Ӧ���ǣ�2 x 2)
% ����2��RA��ͼ
% ������õ���ֵ
function value = wall_cost_compute(init_value, detect_ra_result)
    init_v1 = init_value(1,:);
    init_v2 = init_value(2,:);
    % ȡ������
    
    cost_inside_clutter = 0;
    res1 = sum(abs(detect_ra_result - init_v1), 2);
    res2 = sum(abs(detect_ra_result - init_v2), 2);
    % �����ֵС����ֵ ˵���ھ����ڲ�
    if isempty(find(res1 < 0.1))
        % ȡ����С��ֵ
        cost_inside_clutter = cost_inside_clutter + min(res1);
    end
    % 
    if isempty(find(res2 < 0.1))
        % ȡ����С��ֵ
        cost_inside_clutter = cost_inside_clutter + min(res2);
    end
    
    % ����init_v1 �� init_v2�ľ���
    dis = abs(init_v2(1) - init_v1(1));
    % ����init_v1 �� init_v2�ĽǶ�
    angle = abs(init_v2(2) - init_v1(2));
    % ����init_v1 �� init_v2���ܱߵĵ���Ŀ
    respond_v1 = length(find(abs(init_v1(1) - detect_ra_result(:,1)) < 0.5));
    respond_v2 = length(find(abs(init_v2(1) - detect_ra_result(:,1)) < 0.5));
    respond    = respond_v1 + respond_v2;
    
    value = -dis * 10.0 - angle * 50.0 - respond * 10.0 + cost_inside_clutter * 1000.0;
end

