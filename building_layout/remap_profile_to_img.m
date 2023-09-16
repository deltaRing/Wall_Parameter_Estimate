% �Ѿ��������ܳ�ͼ��
% ���룺range_profile��������
% ���룺SIZE����ӳ��ͼ���С
% ���룺scale���˲�����
% �������ӳ��ͼ�� IMG
function img = remap_profile_to_img(range_profile, size, scale)
    if nargin == 1
        size = [480, 640]; % Ĭ���ṩ 640 x 480 ������
        scale = 0.5;       % �˲�����  w  x  h
    end
    
    mean_value_rangeProfile = mean(mean(range_profile));         % ƽ����ֵ
    value_threshold         = scale * mean_value_rangeProfile;  % �����ֵ
    range_profile(find(range_profile <= value_threshold)) = 0;
    
    range_profile = abs(range_profile) / max(max(abs(range_profile))) * 255;
    range_profile = fix(range_profile);
    % ��ӳ��
    img = imresize(range_profile, size);
end

