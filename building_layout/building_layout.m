clear all
close all
clc

% 能估计：
% 无人机到地面、墙体的距离
% 墙体的长度
% 墙体的角度

addpath algorithm    
addpath building_layout/building_layout_algorithm   
addpath building_layout/building_layout_algorithm/filter_algorithm   
% 1.9 米
% 2.8 米
% 1.2 米
% 1.3 米
% 墙壁真实距离：3.2米

fnumber = 256;
filename = "2023-07-01-16-41-53.bin";
    
fid = fopen(filename,'rb'); 
sdata = fread(fid,'int16'); 
GHz = 1e9;
MHz = 1e6;
us = 1e-6;
%% 雷达参数（使用mmWave Studio默认参数）
c=3.0e8;  
B=4000*1e6;       %调频带宽
K = 68*1e12;  %调频斜率
T=B/K;         %采样时间
Tc=65e-6;     %chirp总周期
fs=10*1e6;       %采样率
f0=77e9;       %初始频率
lambda=c/f0;   %雷达信号波长
d=lambda/2;    %天线阵列间距
n_samples=256; %采样点数/脉冲
N=256;         %距离向FFT点数
n_chirps=64;   %每帧脉冲数
M=256;         %多普勒向FFT点数   
n_RX=4;        %RX天线通道数
n_TX=3;        %TX天线通道数
Q = 512;       %角度FFT
tx = 3;        %发射天线数目
rx = 4;        %接受天线数目
PRI = 40e-3;
    
radar_range_profile = [];
detect_range_profile = [];
range_axis = linspace(0, fs * c / 2 / K, N);
velo_axis = linspace(-lambda / 4 / Tc, lambda / 4 / Tc, M);
time_axis = linspace(0, PRI * fnumber, n_chirps * n_samples);
    
for xx = 1:fnumber-1   
        %16bits，复数形式(I/Q两路)，4RX,3TX,有符号16bit，小端模式
        sdata2 = sdata((xx-1)*n_samples*n_chirps*rx*tx*2+1:xx*n_samples*n_chirps*rx*tx*2);
        %% 1843+DCA1000
        % 阵列排布
        %       rx1 rx2  rx3 rx4 
        %           =======> 方位向
        % tx1 ： []  []  []  []   ||
        % tx2 ： []  []  []  []   ||
        % tx3 ： []  []  []  []   ||
        %               俯仰向    \/
    fileSize = size(sdata2, 1);
    lvds_data = zeros(1, fileSize/2);
    count = 1;
    for i=1:4:fileSize-5
        lvds_data(1,count) =1i* sdata2(i) + sdata2(i+2); 
        lvds_data(1,count+1) =1i* sdata2(i+1)+sdata2(i+3); 
        count = count + 2;
    end
    lvds_data = reshape(lvds_data, tx*n_samples*n_RX, n_chirps);
    t_lvds_data = [];
    for tr = 1:tx * rx
        t_lvds_data(tr, :, :) = lvds_data((tr - 1) * fnumber+1:tr * fnumber, :);
    end

    tx_select_rx = 1;
    rx_select_tx = 1;

    data_tx = [];
    range_profile  = [];
    OSCFAR_profile = [];
    for ttxx = 1:tx
        if rx_select_tx == ttxx
            data_radar_1 = squeeze(t_lvds_data(rx_select_tx * rx - 3, :, :));   %RX1
            data_radar_2 = squeeze(t_lvds_data(rx_select_tx * rx - 2, :, :));   %RX2
            data_radar_3 = squeeze(t_lvds_data(rx_select_tx * rx - 1, :, :));   %RX3
            data_radar_4 = squeeze(t_lvds_data(rx_select_tx * rx, :, :));       %RX4    

            data_radar_1 = data_radar_1(:,1:end-9) - data_radar_1(:,10:end);
            data_radar_2 = data_radar_2(:,1:end-9) - data_radar_2(:,10:end);
            data_radar_3 = data_radar_3(:,1:end-9) - data_radar_3(:,10:end);
            data_radar_4 = data_radar_4(:,1:end-9) - data_radar_4(:,10:end);
        end
        
        temp_data = squeeze(t_lvds_data(ttxx * rx - 3, :, :));    % 选择发射天线1
        temp_data = temp_data(:,1:end-9) - temp_data(:,10:end);    % 均值滤波
        data_tx   = [data_tx; temp_data];
    end
    
    
     %距离FFT
     range_win = hamming(n_samples);   %加海明窗
     doppler_win = hamming(n_chirps);
     range_profile1 = abs(squeeze((fft(data_radar_1, N))));
     range_profile2 = abs(squeeze((fft(data_radar_2, N))));
     range_profile3 = abs(squeeze((fft(data_radar_3, N))));
     range_profile4 = abs(squeeze((fft(data_radar_4, N))));

     ROIs = checkRDmap(range_profile1, M, velo_axis); % 检测感兴趣的距离单元

     OSCFAR_profile = OSCFAR_RangeProfile(range_profile1);
     if isempty(range_profile)
         range_profile = range_profile1 + range_profile2 + range_profile3 + range_profile4;
     else
         range_profile = range_profile + range_profile1 + range_profile2 + range_profile3 + range_profile4;
     end
     speed_profile = fftshift(fft(range_profile1, M, 2), 2);
     range_profile = range_profile / rx;
     figure(10000)
     imagesc(abs(range_profile))
     figure(10005)
     imagesc(abs(speed_profile))
     %多普勒FFT
     radar_range_profile = [radar_range_profile, range_profile];
     detect_range_profile = [detect_range_profile, OSCFAR_profile];
end
 
figure(10001)
imagesc(time_axis, range_axis, db(radar_range_profile))
xlabel("Time (s)")
ylabel("Range (m)")
title("Range profile in flying status")
figure(10002)
imagesc(time_axis, range_axis(16:end-16), db(detect_range_profile));
xlabel("Time (s)")
ylabel("Range (m)")
title("Range profile filtered by OSCFAR")

img = remap_profile_to_img(detect_range_profile);
result = houghLineDetect(img);
range_axis(fix(result / 480 * n_samples)); % default parameter : 480