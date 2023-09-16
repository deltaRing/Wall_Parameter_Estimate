% ֱ���þ���������ȡ����Ȥ�ĵ�Ԫ
% ���룺rangeProfile: ������
% ���룺proportion: ����
% ���룺scale: �˳�����
% �����ROI: ����Ȥ�ľ��뵥Ԫ
function ROI = rangeProfileUOIdetect(rangeProfile, proportion, scale)
    if nargin == 1
        proportion = 0.45; % ���� 45% �����ľ��뵥Ԫ�����Ǹ���Ȥ�ľ��뵥Ԫ
        scale      = 0.95; % ɾ������ƽ����ֵ0.65����
    end
    
    mean_value_rangeProfile = mean(mean(rangeProfile));         % ƽ����ֵ
    value_threshold         = scale * mean_value_rangeProfile;  % �����ֵ
    rangeProfile(find(rangeProfile <= value_threshold)) = 0;
    
    ROI = [];                        % ��ʼ�����뵥Ԫ
    
    size2  = size(rangeProfile, 2);  % ʱ����
    size_I = proportion * size2;     % ��С�ɽ���ʱ�䷶Χ
    for rr = 1:size(rangeProfile, 1)
        [~, value] = find(rangeProfile(rr, :) > 0);
        num = continuousRangeIndexDetect(value);
        if num > size_I
            ROI = [ROI rr];
        end
    end
end

% ����Ƿ����������뵥Ԫ
function num = continuousRangeIndexDetect(index)
    diff_index = index(2:end) - index(1:end-1);
    [index, value] = find(diff_index <= 5);     % ������С��5����Ϊ�������ĵ�Ԫ
    num = length(value);                        % ���Ⱦ���
end
