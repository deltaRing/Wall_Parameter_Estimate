% 2-D CACFAR
% ���룺RD_map�������������ͼ
% ���룺R���ο���Ԫ��С
% ���룺P��������Ԫ��С
% ���룺gain�������С
% �����detect_result������2D-CACFAR��ļ��ͼ
% �����detect_points������2D-CACFAR��ĸ���Ȥ�� (rr, vv, value)
function [detect_result, detect_points] = TDCACFAR(RD_map, R, P, gain)
    if nargin == 1
        R = 5;      % Ĭ��
        P = 3;      % Ĭ��
        gain = 1.0; % Ĭ��
    end
    
    all_gate = R + P;
    protect_gate = P;
    
    detect_result = [];
    detect_points = [];
    for rr = 1:size(RD_map, 1)         % ������
        for vv = 1:size(RD_map, 2)     % �ٶ���
            test_rr_str = rr - all_gate;
            test_vv_str = vv - all_gate; % ���Ե�Ԫ��ʼ���
            test_rr_end = rr + all_gate;
            test_vv_end = vv + all_gate; % ���Ե�Ԫ�������
            
            % ������Χ
            if test_rr_str <= 0 || test_vv_str <= 0
                detect_result(rr, vv) = 0.0;
                continue
            end
            if test_rr_end > size(RD_map, 1) || test_vv_end >= size(RD_map, 2)
                detect_result(rr, vv) = 0.0;
                continue;
            end
            % ������Χ
            
            % ������Ԫ
            pro_rr_str = rr - protect_gate;
            pro_vv_str = vv - protect_gate;
            pro_rr_end = rr + protect_gate;
            pro_vv_end = vv + protect_gate;
            % ������Ԫ
            
            select_units = RD_map(test_rr_str:test_rr_end, ...
                test_vv_str:test_vv_end); % 
            select_units(pro_rr_str:pro_rr_end, ...
                pro_vv_str:pro_vv_end) = 0;
            threshold = sum(sum(select_units)) / (all_gate * all_gate - ...
                protect_gate * protect_gate); % �����ܺ� ƽ��ֵ
            if abs(RD_map(rr, vv)) > gain * abs(threshold)
                detect_result(rr, vv) = abs(RD_map(rr, vv));
                detect_points = [detect_points; rr vv abs(RD_map(rr, vv))];
            else
                detect_result(rr, vv) = 0.0;
            end
            
        end
    end

end