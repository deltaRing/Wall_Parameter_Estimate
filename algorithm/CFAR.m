function [res, det] = CFAR(data, disable_range, disable_velo, R, P)
    if nargin < 2
        disable_range = 45;
        disable_velo = 40;
        R=16;   % �ο���Ԫ��
        P=5;    % ������Ԫ��
    elseif nargin == 3
        R=16;   % �ο���Ԫ��
        P=5;    % ������Ԫ��
    end
    
    data(1:disable_range, :) = 0;
    data(:, 1:disable_velo) = 0;
    data(:, end-disable_velo:end) = 0;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CA-CFAR %%%%%%%%%%%%%%%%%%%%%%%%%
%     data = abs(data);
    det = [];
    det_ = []; % 
    if length(size(data)) >= 3
        fprintf("2D data is required!");
        return;
    end
    M=size(data,1);
    L_slipper=R;%��������
    L_move=1;%�������
    ratio = 2.0;
    remove_ratio = 0.5;
    L_num=floor((M-L_slipper)/L_move);%��������
    res = zeros(M - L_slipper, size(data, 2));
    res_ = zeros(M - L_slipper, size(data, 2)); % Ԥ�ȷ������ݴ�С
    for jj = 1:size(data,2)
        for ii = 1:L_num
            start_idx = (ii-1)*L_move+1;
            middle_idx_1 = start_idx+floor(R/2)-P;
            middle_idx_2 = start_idx + floor(R/2) + P;
            end_idx = start_idx + R;
            index = [start_idx:middle_idx_1, middle_idx_2:end_idx];
            Z = sum(abs(data(index, jj))) / (R - P * 2);
            val = abs(data(ii+floor(R/2), jj));
            if Z * ratio < val
                res_(ii,jj) = data(ii+floor(R/2), jj); % ������CFAR
            end
        end
    end
    
    %% ȥ������ǿ�ȹ��͵�Ŀ��
    max_det = max(max(abs(res_)));
    for ii = 1:size(res_, 1)
        for jj = 1:size(res_, 2)
            if abs(res_(ii, jj)) >= max_det * remove_ratio
                det = [det; ii, jj, res_(ii, jj)];
                res(ii, jj) = res_(ii, jj);
            end
        end
    end
    
end