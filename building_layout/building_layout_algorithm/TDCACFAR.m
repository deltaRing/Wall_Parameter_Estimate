% 2-D CACFAR
% 输入：RD_map：距离多普勒谱图
% 输入：R：参考单元大小
% 输入：P：保护单元大小
% 输入：gain：增益大小
% 输出：detect_result：经过2D-CACFAR后的检测图
% 输出：detect_points：经过2D-CACFAR后的感兴趣点 (rr, vv, value)
function [detect_result, detect_points] = TDCACFAR(RD_map, R, P, gain)
    if nargin == 1
        R = 5;      % 默认
        P = 3;      % 默认
        gain = 1.0; % 默认
    end
    
    all_gate = R + P;
    protect_gate = P;
    
    detect_result = [];
    detect_points = [];
    for rr = 1:size(RD_map, 1)         % 距离向
        for vv = 1:size(RD_map, 2)     % 速度向
            test_rr_str = rr - all_gate;
            test_vv_str = vv - all_gate; % 测试单元起始标号
            test_rr_end = rr + all_gate;
            test_vv_end = vv + all_gate; % 测试单元结束标号
            
            % 超过范围
            if test_rr_str <= 0 || test_vv_str <= 0
                detect_result(rr, vv) = 0.0;
                continue
            end
            if test_rr_end > size(RD_map, 1) || test_vv_end >= size(RD_map, 2)
                detect_result(rr, vv) = 0.0;
                continue;
            end
            % 超过范围
            
            % 保护单元
            pro_rr_str = rr - protect_gate;
            pro_vv_str = vv - protect_gate;
            pro_rr_end = rr + protect_gate;
            pro_vv_end = vv + protect_gate;
            % 保护单元
            
            select_units = RD_map(test_rr_str:test_rr_end, ...
                test_vv_str:test_vv_end); % 
            select_units(pro_rr_str:pro_rr_end, ...
                pro_vv_str:pro_vv_end) = 0;
            threshold = sum(sum(select_units)) / (all_gate * all_gate - ...
                protect_gate * protect_gate); % 计算总和 平均值
            if abs(RD_map(rr, vv)) > gain * abs(threshold)
                detect_result(rr, vv) = abs(RD_map(rr, vv));
                detect_points = [detect_points; rr vv abs(RD_map(rr, vv))];
            else
                detect_result(rr, vv) = 0.0;
            end
            
        end
    end

end