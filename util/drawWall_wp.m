% ���� ǽ��ֲ�
% ���ø�˹�ֲ� ����ǽ��
% ����1��ǽ����ʼλ��
% ����2��ǽ�����λ��
function drawWall_wp(wallStart, wallEnd)
% ����þ�ֵ
x_mean = mean([wallStart(:, 1); wallEnd(:, 1)]);
y_mean = mean([wallStart(:, 2); wallEnd(:, 2)]);
% ����÷���
x_var  = sum(([wallStart(:, 1); wallEnd(:, 1)] - x_mean).^2) / x_mean;
y_var  = sum(([wallStart(:, 2); wallEnd(:, 2)] - y_mean).^2) / y_mean;
% �õ��ֲ�
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
%��˹�ֲ��ĸ����ܶȺ���
%���룺
%        X -- ���
%        mu -- ��ֵ
%        sigma -- ��׼��
%�����
%        y -- ��˹�ֲ�
y = 1./((2*pi)^(size(X,1)/2)*det(sigma)^(1/2))*...
          exp(-0.5*((X-mu)')/sigma*((X-mu)));
end
