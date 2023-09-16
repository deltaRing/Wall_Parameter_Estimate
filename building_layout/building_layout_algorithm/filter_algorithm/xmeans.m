% ʹ��XMEANS ���о���
% ����1���ȴ���������� data
% ����2����С�ľ�����Ŀ Kmin
% ����3�����ľ�����Ŀ Kmax
% ���1���������� center
% ���2���µ����� data_new
function [center, data_new] = xmeans(data, kmax)
    % �������ݵĴ�С��n x 3 + 1��
    k = 1; % �ӵ������࿪ʼ����
    while 1
        ok = k;
        % ������С��Kmin������
        [center_init, data_init] = kmeans(data, k);
        % ���kmeans����ʧ�� ���½��м���
        while isempty(data_init), [center_init, data_init] = kmeans(data, k); end
        index_num = size(data_init, 2);  % index���ڵ�λ��
        free_para = index_num - 1;       % ���ɲ�����
        
        o_bic_scores = [];
        for ii = 1:k
            % �ֱ���� K = 1 �Լ� K = 2 ��ʱ�� BIC�����Ƕ���
            data_select = data_init(find(data_init(:, index_num) == ii), 1:3);  % ѡ�������
            data_size   = size(data_init, 1);                                   % ��������
            center_ii   = center_init(ii,:);                                    % ������λ�õĵ�
            distance    = norm(data_select - center_ii)^2;                      % ������λ�õľ���
            score_1     = BIC_points(1, data_size, free_para, ...
                [distance], [data_size]);                                       % ����BIC����
            o_bic_scores  = [o_bic_scores score_1];
        end
        
        SK    = 2; % �����Ӿ���
        ADD_K = 0; % ��Ҫ�����K
        s_bic_scores = [];
        for ii = 1:k 
            data_select         = data_init(find(data_init(:, index_num) == ii), 1:3); % ѡ�������
            [center_2, data_2]  = kmeans(data_select, 2);
            while isempty(data_2), [center_2, data_2]  = kmeans(data_select, 2); end
            data_select_1       = data_2(find(data_2(:, index_num) == 1), 1:3);   % ѡ������� 
            data_select_2       = data_2(find(data_2(:, index_num) == 2), 1:3);  
            data_size1          = size(data_select_1, 1);                              % ��������
            data_size2          = size(data_select_2, 1);                     
            distance1           = norm(data_select_1 - center_2(1, :))^2;              % ���ݵ��������ĵķ���
            distance2           = norm(data_select_2 - center_2(2, :))^2;   
            score_2             = BIC_points(2, data_size, free_para, ...
                [distance1 distance2], [data_size1 data_size2]);    % ����BIC����
            s_bic_scores  = [s_bic_scores score_2];                 % ������������
        end
        
        ADD_K = length(find(s_bic_scores > o_bic_scores * 1.1)); % ������ͬ
        k = k + ADD_K;
        
        if ok == k || k >= kmax, break; end
    end
    % ���������
    [center, data_new] = kmeans(data, k);
    while isempty(center)
        [center, data_new] = kmeans(data, k);
    end
    
    if 0
        figure(10021)
        scatter3(data_select(:, 1),data_select(:, 2),data_select(:, 3))
        figure(10022)
        scatter3(data_select_1(:, 1),data_select_1(:, 2),data_select_1(:, 3))
        hold on
        scatter3(data_select_2(:, 1),data_select_2(:, 2),data_select_2(:, 3))
    end
end

% ����BIC����
% ����1��K �ص����� ��K = 1��2��
% ����2��R �������� �����еĵ㣩
% ����3��M ����ά�� �����ɲ�����(3)
% ����4��distance �������ĵ��������ݵľ���ƽ��
% ����5��cluster_size ÿ����Ⱥ������
% ���1��score BIC���� 
function score = BIC_points(K, R, M, distance, cluster_size)
    var = 1 / (R - K) * distance;  % ����
    L = 0;
    for ii = 1:K
        L = L + logLikelihood(K, R, cluster_size(ii), M, var(ii));
    end
    numParameters = (M + 1) * K; % BIC��ʽ
    score = L - 0.5 * numParameters * log(R);
end

% ���������Ȼ����
% ����1��K   �ص�����
% ����2��R   ��������
% ����3��Ri  ������
% ����4��M   ����ά�ȣ�3��
% ����5��var ����
% ���1��logLike:��������
function loglike = logLikelihood(K, R, Ri, M, var)
    p1 = -Ri * log(2 * pi);
    p2 = -Ri * M * log(var);
    p3 = -(Ri - K);
    p4 = Ri * log(Ri);
    p5 = -Ri * log(R);
    loglike = (p1 + p2 + p3) / 2.0 + p4 + p5;
end