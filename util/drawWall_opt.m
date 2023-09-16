% ����ǽ�� Final
% ���룺1 ǽ����ʼλ��
% ���룺2 ǽ�����λ��
% ���룺3 �˳��뾶
% ���룺4 �����뾶
function drawWall_opt(wallStart, wallEnd, filterRange, associateRange)
if nargin == 2 
    filterRange = 0.5;
    associateRange = 0.1;
end
x_axis = -5:0.01:5;
y_axis = 0:0.01:5;
% ӳ�䵽ͼ����
% ���޳��쳣ֵ

startDBSCAN = DBSCAN(wallStart, filterRange, 3);
endDBSCAN   = DBSCAN(wallEnd,   filterRange, 3);

meanStart = mean(startDBSCAN(:,1:2), 1);
meanEnd   = mean(endDBSCAN(:,1:2),   1);
new_wall_start = wallStart(find(sum(abs(wallStart - meanStart), 2) < filterRange), :);
new_wall_end   = wallEnd(find(sum(abs(wallEnd - meanEnd), 2) < filterRange), :);
dis = meanEnd - meanStart;
angle = atan2(dis(2), dis(1));
% �������еĽǶ�
% ��ʼ��Image
image = zeros(length(y_axis), length(x_axis));
for xx = 1:length(x_axis)
    for yy = 1:length(y_axis)
        point                   = [x_axis(xx), y_axis(yy)];
        wallStartAssociateIndex = find(sum(abs(point - new_wall_start), 2) < associateRange);
        wallEndAssociateIndex   = find(sum(abs(point - new_wall_end), 2) < associateRange);
        if ~isempty(wallStartAssociateIndex) || ~isempty(wallEndAssociateIndex)
            image(yy, xx) = 4; 
            continue;
        end
        % �Ƕ�
        dis_start = (point - meanStart);
        dis_end   = (meanEnd - point);
        angle_start = atan2(dis_start(2), dis_start(1));
        angle_end   = atan2(dis_end(2), dis_end(1));
        % ��ֵ
        if sum(abs(dis_start)) > sum(abs(dis)), continue; end
        if sum(abs(dis_end))   > sum(abs(dis)), continue; end
        if abs(angle_start - angle) > pi / 18 || abs(angle_end   - angle) > pi / 18, continue; end
        image(yy, xx) = 5; 
    end
end

xxindex = find(abs(meanStart(1) - x_axis) < 0.01);
xyindex = find(abs(meanStart(2) - y_axis) < 0.01);
yxindex = find(abs(meanEnd(1) - x_axis) < 0.01);
yyindex = find(abs(meanEnd(2) - y_axis) < 0.01);
image(xyindex, xxindex) = 3;
image(yyindex, yxindex) = 3;

figure(10026)
imagesc(x_axis, y_axis, image)
xlabel("X axis (meter)")
ylabel("Y axis (meter)")
set(gca,'YDir','normal')

ang = -(meanStart - meanEnd);
norm(meanEnd / cos( atan2(ang(2), ang(1))))
norm(meanStart / cos(atan2(ang(2), ang(1))))
abs(meanStart(1)) * tan(atan2(ang(2), ang(1))) + meanStart(2)
norm(meanStart)
norm(meanEnd)
end