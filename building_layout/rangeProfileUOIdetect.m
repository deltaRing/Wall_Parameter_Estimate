% 直接用距离像来提取感兴趣的单元
% 输入：rangeProfile: 距离像
% 输入：proportion: 比例
% 输入：scale: 滤除比例
% 输出：ROI: 感兴趣的距离单元
function ROI = rangeProfileUOIdetect(rangeProfile, proportion, scale)
    if nargin == 1
        proportion = 0.45; % 超过 45% 比例的距离单元就算是感兴趣的距离单元
        scale      = 0.95; % 删除低于平均幅值0.65倍的
    end
    
    mean_value_rangeProfile = mean(mean(rangeProfile));         % 平均幅值
    value_threshold         = scale * mean_value_rangeProfile;  % 检测阈值
    rangeProfile(find(rangeProfile <= value_threshold)) = 0;
    
    ROI = [];                        % 初始化距离单元
    
    size2  = size(rangeProfile, 2);  % 时间轴
    size_I = proportion * size2;     % 最小可接受时间范围
    for rr = 1:size(rangeProfile, 1)
        [~, value] = find(rangeProfile(rr, :) > 0);
        num = continuousRangeIndexDetect(value);
        if num > size_I
            ROI = [ROI rr];
        end
    end
end

% 检测是否是连续距离单元
function num = continuousRangeIndexDetect(index)
    diff_index = index(2:end) - index(1:end-1);
    [index, value] = find(diff_index <= 5);     % 如果差距小于5，认为是连续的单元
    num = length(value);                        % 长度就是
end
