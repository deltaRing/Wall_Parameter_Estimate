% K-Means �����㷨
% ���룺n x 3 ά�ȵ�����
% ���룺K ��ʼѡ�������
% ������������� Center
% ������������� Data n x 3 + 1 (x y z index)
function [center, data_new] = kmeans(data, K)
    % ���ѡ���ʼ��������
    init_index  = randi(size(data, 1), [1, K]);  % ��������� 
    init_data   = data(init_index, :);                       % ��ѡ������
    data_index  = [];                                        % �洢�����ݼ�������
    for ii = 1:length(init_index), data_index = [data_index; init_data(ii, :) ii]; end
    data_size   = size(data_index, 2);                       % ���ݴ�С
    
    while 1
        for ii = size(data, 1):-1:1
            data_distance = []; % ���ݾ���
            for jj = 1:K
                distance = norm(init_data(jj, :) - data(ii, :));  % ��K�ľ���
                data_distance = [data_distance distance];         % �洢����
            end
            [value, index] = min(data_distance);            % �ҵ�����ľ���
            data_index = [data_index; data(ii, :) index];   % �������
        end
        
        % ���¼����������
        new_center = [];  % �µľ�������
        for iii = 1:K
            DOI = find(data_index(:, data_size) == iii);       % Ѱ������
            cluster_data = data_index(DOI, 1:data_size - 1);   % ѡ������
            if isempty(cluster_data),
                disp("K-means failed to cluster");
                center   = [];
                data_new = []; % false ���
                return; % ������û�ҵ�
            end
            new_center   = [new_center; mean(cluster_data, 1)];
        end
        
        % �Ѿ�������
        if norm(new_center - init_data) < 1e-6, break, end
        init_data = new_center; % ��������
        data_index  = [];       % �洢�����ݼ�������
    end
    
    data_new = data_index;   % �洢�����ݼ�������
    center   = new_center;   % ��¼����������
    
    % ��������
    if 0
        figure(10020)
        hold on
        for ii = 1:K
            data_scatter = data_index(find(data_index(:,4)==ii), 1:3);
            scatter3(data_scatter(:,1),data_scatter(:,2),data_scatter(:,3))
        end
        title("K-means��������ָ��K=2��")
        xlabel("���� (meter)")
        ylabel("�ٶ� (m/s)")
        zlabel("�Ƕ� (rad)")
        legend
    end
end