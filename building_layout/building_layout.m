clear all
close all
clc

% �ܹ��ƣ�
% ���˻������桢ǽ��ľ���
% ǽ��ĳ���
% ǽ��ĽǶ�

addpath algorithm    
addpath building_layout/building_layout_algorithm   
addpath building_layout/building_layout_algorithm/filter_algorithm   
% 1.9 ��
% 2.8 ��
% 1.2 ��
% 1.3 ��
% ǽ����ʵ���룺3.2��

fnumber = 256;
filename = "2023-07-01-16-41-53.bin";
    
fid = fopen(filename,'rb'); 
sdata = fread(fid,'int16'); 
GHz = 1e9;
MHz = 1e6;
us = 1e-6;
%% �״������ʹ��mmWave StudioĬ�ϲ�����
c=3.0e8;  
B=4000*1e6;       %��Ƶ����
K = 68*1e12;  %��Ƶб��
T=B/K;         %����ʱ��
Tc=65e-6;     %chirp������
fs=10*1e6;       %������
f0=77e9;       %��ʼƵ��
lambda=c/f0;   %�״��źŲ���
d=lambda/2;    %�������м��
n_samples=256; %��������/����
N=256;         %������FFT����
n_chirps=64;   %ÿ֡������
M=256;         %��������FFT����   
n_RX=4;        %RX����ͨ����
n_TX=3;        %TX����ͨ����
Q = 512;       %�Ƕ�FFT
tx = 3;        %����������Ŀ
rx = 4;        %����������Ŀ
PRI = 40e-3;
    
radar_range_profile = [];
detect_range_profile = [];
range_axis = linspace(0, fs * c / 2 / K, N);
velo_axis = linspace(-lambda / 4 / Tc, lambda / 4 / Tc, M);
time_axis = linspace(0, PRI * fnumber, n_chirps * n_samples);
    
for xx = 1:fnumber-1   
        %16bits��������ʽ(I/Q��·)��4RX,3TX,�з���16bit��С��ģʽ
        sdata2 = sdata((xx-1)*n_samples*n_chirps*rx*tx*2+1:xx*n_samples*n_chirps*rx*tx*2);
        %% 1843+DCA1000
        % �����Ų�
        %       rx1 rx2  rx3 rx4 
        %           =======> ��λ��
        % tx1 �� []  []  []  []   ||
        % tx2 �� []  []  []  []   ||
        % tx3 �� []  []  []  []   ||
        %               ������    \/
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
        
        temp_data = squeeze(t_lvds_data(ttxx * rx - 3, :, :));    % ѡ��������1
        temp_data = temp_data(:,1:end-9) - temp_data(:,10:end);    % ��ֵ�˲�
        data_tx   = [data_tx; temp_data];
    end
    
    
     %����FFT
     range_win = hamming(n_samples);   %�Ӻ�����
     doppler_win = hamming(n_chirps);
     range_profile1 = abs(squeeze((fft(data_radar_1, N))));
     range_profile2 = abs(squeeze((fft(data_radar_2, N))));
     range_profile3 = abs(squeeze((fft(data_radar_3, N))));
     range_profile4 = abs(squeeze((fft(data_radar_4, N))));

     ROIs = checkRDmap(range_profile1, M, velo_axis); % ������Ȥ�ľ��뵥Ԫ

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
     %������FFT
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