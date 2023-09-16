%% 墙壁多径校正算法
% 输入：雷达偏置角度，墙壁相对起始位置，墙壁相对结束位置，原始的探测结果
% 输出：校正后的探测位置
function [correct_loc, mirror_loc] = wallMultipathCorrect(emitterAngle, ...
    reflectWallStartLoc, ...  % 1 x 2
    reflectWallEndLoc, ...    % 1 x 2
    origin_detect_res) % N x 2

    correct_loc = [];
    
if length(reflectWallStartLoc) ~= 2
    disp("墙壁的起始位置：需要1x2的向量")
    return
end

if length(reflectWallEndLoc) ~= 2
    disp("墙壁的结束位置：需要1x2的向量")
    return
end

% 校正的雷达旋转矩阵
RotationMatrixEmitter = [cos(emitterAngle) -sin(emitterAngle);
    sin(emitterAngle) cos(emitterAngle)];

RelatedWallLoc   = reflectWallEndLoc - reflectWallStartLoc;                % 墙壁相对位置
wallAngle  = atan2(RelatedWallLoc(2), RelatedWallLoc(1));    % 墙壁的相对角度
cDetectRes = origin_detect_res * RotationMatrixEmitter;    % 把目标位置给矫正好 雷达坐标系转为世界坐标系
mirror_loc = cDetectRes;

% 校正的墙壁的相对位置
RotationMatrixWall = [cos(wallAngle) -sin(wallAngle);
    sin(wallAngle) cos(wallAngle)];
cDetectRes = cDetectRes * RotationMatrixWall.';
reflectWallEndLoc = reflectWallEndLoc * RotationMatrixWall.';

% 先判断距离是否大于了无人机到墙壁的距离
% 如果不大于，很明显不是非视距的点
% 应该消除
[~, cDetectRes] = wallDistanceCalc(reflectWallStartLoc, reflectWallEndLoc, cDetectRes);    % 计算点迹到墙壁的距离
cDetectRes = cDetectRes * RotationMatrixWall;                                % 矫正回来
% 消除后：
% 对这些点迹进行矫正处理：
correct_loc = cDetectRes;
end

% 计算出墙壁的距离，并滤除不符合非视距区域的点迹
% 输入: 墙壁的起始位置、目标的位置群 (雷达位置：0，0）
% 输出：滤波后的目标 矫正后的目标位置
function [new_tarLoc, wall_correct] = wallDistanceCalc(wallLocStart, ... % 1 x 2
    wallLocEnd, ...                                                       % 1 x 2
    tarLoc)                                                               % N x 2
% 计算墙壁那边是0
relatedWallLoc = wallLocEnd - wallLocStart;
if abs(relatedWallLoc(1)) < 1e-7
    % 与y轴平行
    x_loc = wallLocStart(1);
    if x_loc > 0
        index = find(tarLoc(:,1) < x_loc);
        tarLoc(index,:) = []; % 屏蔽掉
        wall_correct = [x_loc - abs(tarLoc(:,1) - x_loc) tarLoc(:,2)]; % 计算矫正后的目标位置（应该呈镜像对称）
    elseif x_loc < 0
        index = find(tarLoc(:,1) > x_loc);
        tarLoc(index,:) = []; % 屏蔽掉
        wall_correct = [x_loc + abs(x_loc - tarLoc(:,1)) tarLoc(:,2)]; % 计算矫正后的目标位置（应该呈镜像对称）
    end
elseif abs(relatedWallLoc(2)) < 1e-7
    % 与x轴平行
    y_loc = wallLocStart(2);
    if y_loc > 0
        index = find(tarLoc(:,2) < y_loc);
        tarLoc(index,:) = []; % 屏蔽掉
        wall_correct = [tarLoc(:,1) y_loc - abs(tarLoc(:,2) - y_loc)]; % 计算矫正后的目标位置（应该呈镜像对称）
    elseif y_loc < 0
        index = find(tarLoc(:,2) > y_loc);
        tarLoc(index,:) = []; % 屏蔽掉
        wall_correct = [tarLoc(:,1) y_loc + abs(tarLoc(:,2) - y_loc)]; % 计算矫正后的目标位置（应该呈镜像对称）
    end
end

new_tarLoc = tarLoc;
end