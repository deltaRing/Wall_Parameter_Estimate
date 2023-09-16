% 把距离像重塑成图像
% 输入：range_profile：距离像
% 输入：SIZE：重映射图像大小
% 输入：scale：滤波比例
% 输出：重映射图像 IMG
function img = remap_profile_to_img(range_profile, size, scale)
    if nargin == 1
        size = [480, 640]; % 默认提供 640 x 480 距离像
        scale = 0.5;       % 滤波比例  w  x  h
    end
    
    mean_value_rangeProfile = mean(mean(range_profile));         % 平均幅值
    value_threshold         = scale * mean_value_rangeProfile;  % 检测阈值
    range_profile(find(range_profile <= value_threshold)) = 0;
    
    range_profile = abs(range_profile) / max(max(abs(range_profile))) * 255;
    range_profile = fix(range_profile);
    % 重映射
    img = imresize(range_profile, size);
end

