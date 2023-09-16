% GMM_estimate_wallDis 采用高斯混合模型来估计出墙体距离
% 输入：observe：观测值向量
% 输入：g_mu：地面均值
% 输入：g_var：地面方差
% 输入：g_w：地面权重
% 输入：w_mu：墙面均值
% 输入：w_var：墙面方差
% 输入：w_w：墙面权重
% 输出：g_mu_n：新的地面均值
% 输出：g_var_n：新的地面方差
% 输出：g_w_n：新的地面权重
% 输出：w_mu_n：新的墙面均值
% 输出：w_var_n：新的墙面方差
% 输出：w_w_n：新的墙面权重
function [g_mu_n, g_var_n, g_w_n, ...
    w_mu_n, w_var_n, w_w_n] ...
    = GMM_estimate_wallDis(observe, g_mu, g_var, ...
    g_w, w_mu, w_var, w_w)
    if nargin == 1
        g_mu = 1.0; g_var = 1.0; g_w = 0.5; % 默认的地面初始值
        w_mu = 4.0; w_var = 1.0; w_w = 0.5; % 默认的墙面初始值
    end
    
    % 计算响应
    R_w = zeros(1, length(observe));
    R_g = zeros(1, length(observe));
    
    for ii = 1:length(observe)
        px_w = w_w * pdf('norm', observe(ii), w_mu, w_var);
        px_g = g_w * pdf('norm', observe(ii), g_mu, g_var);
        R_w(ii) = px_w / (px_w + px_g);
        R_g(ii) = px_g / (px_w + px_g);
    end
    
    % 计算期望
    sum_w = 0; sum_g = 0;
    for ii = 1:length(observe)
        sum_w = sum_w + R_w(ii) * observe(ii);
        sum_g = sum_g + R_g(ii) * observe(ii);
    end
    g_mu_n = sum_g / sum(R_g);
    w_mu_n = sum_w / sum(R_w);
    
    % 计算方差
    sum_w = 0; sum_g = 0;
    for ii = 1:length(observe)
        sum_w = sum_w + R_w(ii) * (observe(ii) - w_mu_n)^2;
        sum_g = sum_g + R_g(ii) * (observe(ii) - g_mu_n)^2;
    end
    g_var_n = sum_g / sum(R_g); % 得到地面的方差
    w_var_n = sum_w / sum(R_w); % 得到墙体的方差
    
    % 更新权重
    g_w_n = sum(R_g) / length(observe);
    w_w_n = sum(R_w) / length(observe);
end

