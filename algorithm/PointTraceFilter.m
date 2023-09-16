% �㼣�˲��� �����㷨�������˳��״������ӵ��Լ��޹��ɵ������㣩
% ���룺֡id ��̽���� ȷ��̽���� ��ʱ̽�����������Խ���� �����˲���ֵ ��Ӵ���
% ������µ�ȷ��̽����[N x 4] ��ʱ̽����[N x 5]
% �ٶ����е�̽�����ǣ�N x 3 �����ݽṹ ���µ�̽���� ȷ��̽��������ɣ�
%    ���е���ʱ̽�����ǣ�N x 5 �����ݽṹ��xyz �۲���Ŀ ���۲�֡��id
function [detect_res, new_temp_detect_res] = PointTraceFilter(frame_id, ...
    new_detect_res, ... % �µ�̽���� N x 3
    confirm_detect_res, ... % ȷ�ϵ�̽���� N x 4
    temp_detect_res, ... % ��ʱ��̽���� N x 5
    same_point_range, ... % �����˲���ֵ�뾶 single number
    minimal_number, ...  % ��Сȷ����Ŀ single number
    expired_number)  % ��ʱ�洢��������� single number
if nargin <= 4
    same_point_range = 0.5; % 0.5�����ڵ�������һ��Ŀ��
    minimal_number = 5;    % ��������Ҫ���ȷ��
    expired_number = 20;   % ���������ʮ֡û�г��� ȥ�� temp_detect_res �� confirm_detect_res �Ľ��
end

% ���ں����е����� �������̽������
% �����ںϰ뾶��same_point_range
delete_list = [];
confirm_list = zeros(1, size(new_detect_res, 1));
for ii = size(new_detect_res, 1):-1:1
    if ii == 1
        break % �������ֻ��һ��Ŀ�� ֱ������
    end
    if confirm_list
        continue % ���ȷ��ɾ�� ����
    end
    current_tar = new_detect_res(ii, :); % �Ȼ�ȡ�õ�
    for jj = ii-1:-1:1
        if confirm_list
            continue;
        end
        temp_tar = new_detect_res(ii, :); % �������
        range_of_new_tar = abs(norm(current_tar - temp_tar)); % �������
        if range_of_new_tar < same_point_range
            delete_list = [delete_list jj]; % ɾ���б�
            confirm_list(jj) = 1;
        end
    end
end

clear confirm_list;
if ~isempty(delete_list)
    new_detect_res(delete_list, :) = []; % ɾ������
end

% �˳�confirm_detect_res�Ľ��
delete_list = [];
for ii = 1:size(confirm_detect_res, 1)
    if abs(confirm_detect_res(ii, 4) - frame_id) >= expired_number
        delete_list = [delete_list ii];
    end
end
confirm_detect_res(delete_list, :) = [];
    
% �˲���� �ȿ���ʣ���new_detect�Ƿ�����ȷ�Ϻ���������
for ii = 1:size(confirm_detect_res, 1)
    conf_detect_res = confirm_detect_res(ii, 1:3);
    data_index = [];
    for jj = size(new_detect_res, 1):-1:1
        curr_detect_res = new_detect_res(jj, :);
        if abs(norm(conf_detect_res - curr_detect_res)) > same_point_range
            continue; % ̫Զ�� ���ǿ��ܵĺ�����
        end
        data_index = [data_index jj];
        confirm_detect_res(ii, 1:3) = new_detect_res(jj, :); % ����Ŀ��λ��
        confirm_detect_res(ii, 4) = frame_id;                % ֡ID
        break;
    end
    if ~isempty(data_index)
        new_detect_res(data_index, :) = [];
    end
end

delete_list = [];
% �˲���ȥ�����ڵ���ʱ̽����
for ii = size(temp_detect_res, 1):-1:1
    if abs(temp_detect_res(ii, 5) - frame_id) >= expired_number
        delete_list = [delete_list ii];
    end
end
if ~isempty(delete_list)
    temp_detect_res(delete_list, :) = []; % ɾ������
end

% ���������ݽ��н����˲�
% ����temp_detect_res
if size(temp_detect_res, 1) > 0
    for ii = size(temp_detect_res, 1):-1:1
        data_prior = temp_detect_res(ii, 1:3);
        data_index = []; % ������¼�˲��Ľ��
        for jj = size(new_detect_res, 1):-1:1
            data_iter = new_detect_res(jj, :);
            if abs(norm(data_iter - data_prior)) > same_point_range
                continue; % ����㲻���ڽ���
            end
            data_index = [data_index jj];
        end
        if ~isempty(data_index)
            for iii = 1:length(data_index)
                temp_detect_res(ii, 1:3) = temp_detect_res(ii, 1:3) + ...
                    new_detect_res(data_index(iii), :);
            end
            temp_detect_res(ii, 1:3) = temp_detect_res(ii, 1:3) / ...
                (length(data_index) + 1);
            temp_detect_res(ii, 4) = temp_detect_res(ii, 4) + 1; % ���� + 1
            temp_detect_res(ii, 5) = frame_id;                   % ����֡ id
            new_detect_res(data_index, :) = []; % ������Щ����
        end
    end   
else
    % ̽����
    for ii = size(new_detect_res, 1):-1:1
        temp_detect_res = [temp_detect_res; new_detect_res(ii, :) 1 frame_id];
        new_detect_res(ii, :) = []; % ɾ������
    end
end
    
% ���temp_detect_res ȷ���Ƿ��ܼ���confirm_detect_res ��
delete_list = [];
for ii = 1:size(temp_detect_res, 1)
    if temp_detect_res(ii, 4) >= minimal_number
        delete_list = [delete_list ii];
        temp_data = [temp_detect_res(ii, 1:3) frame_id];
        confirm_detect_res = [confirm_detect_res; temp_data];
    end
end
temp_detect_res(delete_list, :) = []; % ɾ������

% ��ʣ�µ�new ������ݼ��뵽temp_detect�б�����
for ii = size(new_detect_res, 1):-1:1
    temp_detect_res = [temp_detect_res; new_detect_res(ii, :) 1 frame_id];
    new_detect_res(ii, :) = [];
end

detect_res = confirm_detect_res;
new_temp_detect_res = temp_detect_res;
    
end