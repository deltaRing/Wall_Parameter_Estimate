%% ǽ�ڶྶУ���㷨
% ���룺�״�ƫ�ýǶȣ�ǽ�������ʼλ�ã�ǽ����Խ���λ�ã�ԭʼ��̽����
% �����У�����̽��λ��
function [correct_loc, mirror_loc] = wallMultipathCorrect(emitterAngle, ...
    reflectWallStartLoc, ...  % 1 x 2
    reflectWallEndLoc, ...    % 1 x 2
    origin_detect_res) % N x 2

    correct_loc = [];
    
if length(reflectWallStartLoc) ~= 2
    disp("ǽ�ڵ���ʼλ�ã���Ҫ1x2������")
    return
end

if length(reflectWallEndLoc) ~= 2
    disp("ǽ�ڵĽ���λ�ã���Ҫ1x2������")
    return
end

% У�����״���ת����
RotationMatrixEmitter = [cos(emitterAngle) -sin(emitterAngle);
    sin(emitterAngle) cos(emitterAngle)];

RelatedWallLoc   = reflectWallEndLoc - reflectWallStartLoc;                % ǽ�����λ��
wallAngle  = atan2(RelatedWallLoc(2), RelatedWallLoc(1));    % ǽ�ڵ���ԽǶ�
cDetectRes = origin_detect_res * RotationMatrixEmitter;    % ��Ŀ��λ�ø������� �״�����ϵתΪ��������ϵ
mirror_loc = cDetectRes;

% У����ǽ�ڵ����λ��
RotationMatrixWall = [cos(wallAngle) -sin(wallAngle);
    sin(wallAngle) cos(wallAngle)];
cDetectRes = cDetectRes * RotationMatrixWall.';
reflectWallEndLoc = reflectWallEndLoc * RotationMatrixWall.';

% ���жϾ����Ƿ���������˻���ǽ�ڵľ���
% ��������ڣ������Բ��Ƿ��Ӿ�ĵ�
% Ӧ������
[~, cDetectRes] = wallDistanceCalc(reflectWallStartLoc, reflectWallEndLoc, cDetectRes);    % ����㼣��ǽ�ڵľ���
cDetectRes = cDetectRes * RotationMatrixWall;                                % ��������
% ������
% ����Щ�㼣���н�������
correct_loc = cDetectRes;
end

% �����ǽ�ڵľ��룬���˳������Ϸ��Ӿ�����ĵ㼣
% ����: ǽ�ڵ���ʼλ�á�Ŀ���λ��Ⱥ (�״�λ�ã�0��0��
% ������˲����Ŀ�� �������Ŀ��λ��
function [new_tarLoc, wall_correct] = wallDistanceCalc(wallLocStart, ... % 1 x 2
    wallLocEnd, ...                                                       % 1 x 2
    tarLoc)                                                               % N x 2
% ����ǽ���Ǳ���0
relatedWallLoc = wallLocEnd - wallLocStart;
if abs(relatedWallLoc(1)) < 1e-7
    % ��y��ƽ��
    x_loc = wallLocStart(1);
    if x_loc > 0
        index = find(tarLoc(:,1) < x_loc);
        tarLoc(index,:) = []; % ���ε�
        wall_correct = [x_loc - abs(tarLoc(:,1) - x_loc) tarLoc(:,2)]; % ����������Ŀ��λ�ã�Ӧ�óʾ���Գƣ�
    elseif x_loc < 0
        index = find(tarLoc(:,1) > x_loc);
        tarLoc(index,:) = []; % ���ε�
        wall_correct = [x_loc + abs(x_loc - tarLoc(:,1)) tarLoc(:,2)]; % ����������Ŀ��λ�ã�Ӧ�óʾ���Գƣ�
    end
elseif abs(relatedWallLoc(2)) < 1e-7
    % ��x��ƽ��
    y_loc = wallLocStart(2);
    if y_loc > 0
        index = find(tarLoc(:,2) < y_loc);
        tarLoc(index,:) = []; % ���ε�
        wall_correct = [tarLoc(:,1) y_loc - abs(tarLoc(:,2) - y_loc)]; % ����������Ŀ��λ�ã�Ӧ�óʾ���Գƣ�
    elseif y_loc < 0
        index = find(tarLoc(:,2) > y_loc);
        tarLoc(index,:) = []; % ���ε�
        wall_correct = [tarLoc(:,1) y_loc + abs(tarLoc(:,2) - y_loc)]; % ����������Ŀ��λ�ã�Ӧ�óʾ���Գƣ�
    end
end

new_tarLoc = tarLoc;
end