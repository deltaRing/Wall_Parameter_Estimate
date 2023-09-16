% GMM_estimate_wallDis ���ø�˹���ģ�������Ƴ�ǽ�����
% ���룺observe���۲�ֵ����
% ���룺g_mu�������ֵ
% ���룺g_var�����淽��
% ���룺g_w������Ȩ��
% ���룺w_mu��ǽ���ֵ
% ���룺w_var��ǽ�淽��
% ���룺w_w��ǽ��Ȩ��
% �����g_mu_n���µĵ����ֵ
% �����g_var_n���µĵ��淽��
% �����g_w_n���µĵ���Ȩ��
% �����w_mu_n���µ�ǽ���ֵ
% �����w_var_n���µ�ǽ�淽��
% �����w_w_n���µ�ǽ��Ȩ��
function [g_mu_n, g_var_n, g_w_n, ...
    w_mu_n, w_var_n, w_w_n] ...
    = GMM_estimate_wallDis(observe, g_mu, g_var, ...
    g_w, w_mu, w_var, w_w)
    if nargin == 1
        g_mu = 1.0; g_var = 1.0; g_w = 0.5; % Ĭ�ϵĵ����ʼֵ
        w_mu = 4.0; w_var = 1.0; w_w = 0.5; % Ĭ�ϵ�ǽ���ʼֵ
    end
    
    % ������Ӧ
    R_w = zeros(1, length(observe));
    R_g = zeros(1, length(observe));
    
    for ii = 1:length(observe)
        px_w = w_w * pdf('norm', observe(ii), w_mu, w_var);
        px_g = g_w * pdf('norm', observe(ii), g_mu, g_var);
        R_w(ii) = px_w / (px_w + px_g);
        R_g(ii) = px_g / (px_w + px_g);
    end
    
    % ��������
    sum_w = 0; sum_g = 0;
    for ii = 1:length(observe)
        sum_w = sum_w + R_w(ii) * observe(ii);
        sum_g = sum_g + R_g(ii) * observe(ii);
    end
    g_mu_n = sum_g / sum(R_g);
    w_mu_n = sum_w / sum(R_w);
    
    % ���㷽��
    sum_w = 0; sum_g = 0;
    for ii = 1:length(observe)
        sum_w = sum_w + R_w(ii) * (observe(ii) - w_mu_n)^2;
        sum_g = sum_g + R_g(ii) * (observe(ii) - g_mu_n)^2;
    end
    g_var_n = sum_g / sum(R_g); % �õ�����ķ���
    w_var_n = sum_w / sum(R_w); % �õ�ǽ��ķ���
    
    % ����Ȩ��
    g_w_n = sum(R_g) / length(observe);
    w_w_n = sum(R_w) / length(observe);
end

