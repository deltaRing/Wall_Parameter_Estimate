% 检测RD谱图 用来滤除 人体的成分 同时根据低速单元 确定无人机回波成分
% 输入：距离谱图 rangeProfile
% 输入：速度FFT点数 veloFFTNum
% 输入：速度轴
% 输入：速度阈值
% 输出：感兴趣的距离单元范围
function ROIs = checkRDmap(rangeProfile, veloFFTNum, velo_axis, velo_threshold)
    if nargin == 3
        velo_threshold = 1.0; % 1.0米以下的都属于无人机引起的
    end
    
    ROIs = [];
    % 2D-FFT
    rd_map = fftshift(fft(rangeProfile, veloFFTNum, 2), 2); % 获取距离-多普勒谱图
    % 找到最小的
    [~, index]   = find(velo_threshold > velo_axis);
    disable_velo = veloFFTNum - length(index);
    % CA-CFAR 2D-FFT
    [detect_profile, detect_res] = CFAR(rd_map, 0, disable_velo);
%     figure(10003)
%     imagesc(abs(rd_map))
figure(10004)
imagesc(abs(detect_profile))
    
end