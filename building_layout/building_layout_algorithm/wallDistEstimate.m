% wallDistEstimate: ǽ�ھ������
% ���룺range_profile ������
% ���룺drone_height ���˻��߶� (ƽ����)
% ���룺range_index ���뵥Ԫ
% ��������˻���ǽ�ڵľ���
function dist = wallDistEstimate(range_profile, drone_height, range_index)
    test_cell_num = 16;   % ���Ե�Ԫ
    gain          = 0.05; % ����ֵ
    L             = 4;    % ������ͽ��ܵ�Ԫ
    H             = 12;   % ������߽��ܵ�Ԫ
    ROI           = [];   % ����Ȥ�ľ���
    VOI           = [];   % ����Ȥ�ľ��뵥Ԫ����Ӧ��ֵ
    select_index  = 5;    % ѡȡ�ľ��뵥Ԫ
    for tt = 1:size(range_profile, 2)
        result_oscfar = OSCFAR_RangeProfile(range_profile(:, tt), ... 
            test_cell_num, gain, L, H);      % ����CFAR���
        [value, index] = sort(abs(result_oscfar)); % �����ݽ������� ֻѡ�񼸸���߷��ȵĵ�
        % ѡ����뵥Ԫ
        index = index(end-select_index: end) + test_cell_num / 2;
        value = value(end-select_index: end);
        % �������˻���ǽ�ڵ�ֱ�߾���
        prior = range_index(index).^2 - drone_height^2;
        value = value(find(prior > 0));
        ranges = sqrt(prior(find(prior > 0)));
        % ��¼���������Ȥ�ĵ�Ԫ
        ROI = [ROI ranges];
        VOI = [VOI value'];
    end

    dist = 0;
    threshold_VOI = mean(VOI);
    threshold_ROI = mean(ROI);
    great_ROI = find(ROI > threshold_ROI * 3.5);
    minor_ROI = find(ROI < drone_height * 0.5);
    noise_VOI = find(VOI < threshold_VOI * 0.25);
    remove_ROI = unique(sort([great_ROI minor_ROI noise_VOI]));
    ROI(remove_ROI) = [];
    % ��ȥ�쳣ֵ
    
    % GMM �˲�
    g_mu = drone_height;
    g_var = 1.0;
    g_w = 0.5;
    w_mu = mean(ROI);
    w_var = 1.0;
    w_w = 0.5;

    for ii = 1:10
        if g_var < 1e-4 || w_var < 1e-4
            break
        end
        [g_mu_n, g_var_n, g_w_n, w_mu_n, w_var_n, w_w_n] = GMM_estimate_wallDis(ROI, g_mu, g_var, g_w, w_mu, w_var, w_w);
        if abs(g_mu_n - g_mu) < 1e-3 || abs(g_mu_n - g_mu) < 1e-3
            break
        end
        g_mu = g_mu_n; g_var = g_var_n; g_w = g_w_n;
        w_mu = w_mu_n; w_var = w_var_n; w_w = w_w_n;
        % ������ֵ
    end
    dist = w_mu;
end

