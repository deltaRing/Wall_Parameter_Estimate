% 绘制 墙体分布
% 利用高斯分布 生成墙体
% 输入1：墙体起始位置
% 输入2：墙体结束位置
function drawWall_wp(wallStart, wallEnd)
% 先求得均值
x_mean = mean([wallStart(:, 1); wallEnd(:, 1)]);
y_mean = mean([wallStart(:, 2); wallEnd(:, 2)]);
% 再求得方差
x_var  = sum(([wallStart(:, 1); wallEnd(:, 1)] - x_mean).^2) / x_mean;
y_var  = sum(([wallStart(:, 2); wallEnd(:, 2)] - y_mean).^2) / y_mean;
% 得到分布
mu = [x_mean; y_mean];
sigma = [x_var 0;
    0 y_var];
x = -10:0.01:10;
y = zeros(length(x));
[X1,X2] = meshgrid(x,x);
for i=1:length(x) * length(x)
    y(i) = gaussPDF([X1(i);X2(i)], mu, sigma);
end
z=reshape(y,length(x),length(x));
figure(1);
mesh(X1,X2, z);
title('PDF of bivariate Gaussian distribution');
xlabel('x'); ylabel('y'); zlabel('PDF');
end

function [y] = gaussPDF(X, mu, sigma)
%高斯分布的概率密度函数
%输入：
%        X -- 结果
%        mu -- 均值
%        sigma -- 标准差
%输出：
%        y -- 高斯分布
y = 1./((2*pi)^(size(X,1)/2)*det(sigma)^(1/2))*...
          exp(-0.5*((X-mu)')/sigma*((X-mu)));
end
