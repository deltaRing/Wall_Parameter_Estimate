clear all;
close all;
clc;

addpath building_layout/building_opt
addpath building_layout/building_layout_algorithm/filter_algorithm
addpath building_layout/building_layout_algorithm/select_algorithm
addpath building_layout/building_layout_algorithm
addpath building_layout
addpath algorithm
addpath building
addpath util

wallStart = [-1.0, 2.2];
wallEnd   = [3.0, 2.2];
barrierStart = [2.2 1.7];
barrierEnd   = [2.2 -1.0];

GHz = 1e9;
MHz = 1e6;
us = 1e-6;
%% �״������ʹ��mmWave StudioĬ�ϲ�����
c=3.0e8;  
B=4000*1e6;       %��Ƶ����
K = 68*1e12;      %��Ƶб��
T=B/K;            %����ʱ��
Tc=65e-6;         %chirp������
fs=10*1e6;       %������
f0=77e9;          %��ʼƵ��
lambda=c/f0;      %�״��źŲ���
d=lambda/2;       %�������м��
n_samples=256;    %��������/����
N=256;            %������FFT����
n_chirps=64;      %ÿ֡������
M=512;            %��������FFT����
n_RX=4;           %RX����ͨ����
n_TX=3;           %TX����ͨ����
Q = 512;          %�Ƕ�FFT
tx = 3;           %����������Ŀ
rx = 4;          %����������Ŀ
fnumber = 256;
PRI = 4e-3;

emitterAngle  = pi / 4;
correctMatrix = [cos(emitterAngle) -sin(emitterAngle); sin(emitterAngle) cos(emitterAngle)];
fontSize = 16;

range_axis = linspace(0, fs * c / 2 / K, N);
velo_axis = linspace(-lambda / 4 / Tc, lambda / 4 / Tc, M);
an_axis_az = linspace(-asin(lambda/2/d), asin(lambda/2/d), Q); % �״��ǵ���������
an_axis_el = linspace(-asin(lambda/2/d), asin(lambda/2/d), Q);

% 45 angle:(3.2)
% 1.9 ��
% 2.8 ��
% 1.2 ��
% 1.3 ��

%  0 angle��(4.0)
% 1: 2.0meter  2:2.9meter(������ʹ��)(���䵽ǽ����), 3:1.3(��Ҫʹ�ã� 4��1.45

%3rd
% 1.7 2.0 2.6 2.9
% 2.3 1.9 2.0 2.4

fname = "0_wall/2023-07-01-16-39-48.bin";
mode  = 2;
% �����̵���;
% 1��NLOS��λ
% 2��ǽ��������

MODE_NLOS_POSI = 1;
MODE_WALL_PARA = 2;

xxx = [];
yyy = [];

% ̽����
confirm_detect = [];
temp_detect    = [];
origin_detect  = []; % �������κο����Ŵ�ʩ�Ľ��
detect_results = []; % RD_ѡ���ļ����
amp_results    = []; % ������Ϣ
cluster_index  = 0;  % ��¼�������ֵ
Wall_X1 = [];
Wall_X2 = [];

fid = fopen(fname,'rb'); 
sdata = fread(fid,'int16');    
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
    
    tx_data = [];
    tx_select_rx = 1;
    rx_select_tx = 1;
    
    data_tx = [];
    
    for ttxx = 1:tx
        if ttxx == rx_select_tx
            data_radar_1 = squeeze(t_lvds_data(rx_select_tx * rx - 3, :, :));   %RX1
            data_radar_2 = squeeze(t_lvds_data(rx_select_tx * rx - 2, :, :));   %RX2
            data_radar_3 = squeeze(t_lvds_data(rx_select_tx * rx - 1, :, :));   %RX3
            data_radar_4 = squeeze(t_lvds_data(rx_select_tx * rx, :, :));       %RX4
        end
        
        temp_data = squeeze(t_lvds_data(ttxx * rx - 3, :, :));          % ����RX1
        data_tx(:, :, ttxx) = temp_data(:,1:end-8) - temp_data(:,9:end);
    end
   
    data_radar_11 = data_radar_1(:,1:end-8) - data_radar_1(:,9:end);
    data_radar_22 = data_radar_2(:,1:end-8) - data_radar_2(:,9:end);
    data_radar_33 = data_radar_3(:,1:end-8) - data_radar_3(:,9:end);
    data_radar_44 = data_radar_4(:,1:end-8) - data_radar_4(:,9:end);
    
    data_radar=[];            
    data_radar(:,:,1) = data_radar_11;     %��ά�״�ز�����
    data_radar(:,:,2) = data_radar_22;
    data_radar(:,:,3) = data_radar_33;
    data_radar(:,:,4) = data_radar_44;
    %% 3άFFT����
    %����FFT
    range_win = hamming(n_samples);   %�Ӻ�����
    doppler_win = hamming(n_chirps);
    range_profile = [];
    for k=1:n_RX
        range_profile(:,:,k) = fft(data_radar(:,:,k), N);
    end
    
    range_profile_tx = [];
    for k=1:tx
        range_profile_tx(:,:,k) = fft(data_tx(:,:,k), N);
    end
    
    %������FFT
    speed_profile = [];
    for k=1:n_RX
        speed_profile(:,:,k) = fftshift(fft(range_profile(:,:,k), M, 2), 2);
    end
   
    speed_profile_tx = [];
    for k=1:n_TX
        speed_profile_tx(:,:,k) = fftshift(fft(range_profile_tx(:,:,k), M, 2), 2);
    end
    
    rd_result = squeeze(sum(speed_profile, 3) / size(speed_profile, 3));
    [detect_profile, detect_res] = CFAR(rd_result);
%     %% detect_res(:, 2) is range 
    az_rmap = fftshift(squeeze(sum(abs(fft(speed_profile, Q, 3)), 2)), 2);    % ���� x �Ƕ�ά��
    el_rmap = fftshift(squeeze(sum(abs(fft(speed_profile_tx, Q, 3)), 2)), 2); % ���� x �Ƕ�ά��
%     angle_detect = AngleDetectSearchingAngle(az_rmap, detect_res);
%     % ���� (detect_res(:, 2), detect_res(:, 1) + 9) ���궨 rd ��ͼ���� ������
%     % detect_res(:, 1) �� ������
%     if isempty(angle_detect)
%         xxxx = [];
%         yyyy = [];
%     else
%         angles = an_axis_az(angle_detect(:,2));
%         ranges = range_axis(angle_detect(:,1));
%         detect_result_UOI = [ranges; angles];
%         
%         drawUOIarea(az_rmap, detect_result_UOI, range_axis, an_axis_az);
% 
%         xxxx = ranges .* sin(angles);
%         yyyy = ranges .* cos(angles);
%     end
%     
%     origin_detect = [origin_detect; xxxx' yyyy'];
%     % 
%     new_detect_res = zeros(length(xxxx), 3);
%     new_detect_res(:, 1) = xxxx; 
%     new_detect_res(:, 2) = yyyy;
%     % �������

figure(10006)
    suptitle("Radar waveform��Real Part��");
    subplot(221)
    plot(real(data_radar_1(:, 1)))
    axis([0 256 min(real(data_radar_1(:, 1))) max(real(data_radar_1(:, 1)))])
    subplot(222)
    plot(real(data_radar_2(:, 1)))
    axis([0 256 min(real(data_radar_2(:, 1))) max(real(data_radar_2(:, 1)))])
    subplot(223)
    plot(real(data_radar_3(:, 1)))
    axis([0 256 min(real(data_radar_3(:, 1))) max(real(data_radar_3(:, 1)))])
    subplot(224)
    plot(real(data_radar_4(:, 1)))
    axis([0 256 min(real(data_radar_4(:, 1))) max(real(data_radar_4(:, 1)))])
figure(10007)
    range_profile1 = flip(fft(data_radar_1, N));
    imagesc(linspace(1, 64), range_axis, abs(squeeze(range_profile(:,:,1))))
    xlabel("Chirps", 'fontsize',fontSize,'FontName','Times New Roman')
    ylabel("distance (m)", 'fontsize',fontSize,'FontName','Times New Roman')
    title("Range Profile", 'fontsize',fontSize,'FontName','Times New Roman')
figure(10008)
    speed_profile1 = fft(range_profile1, M, 2);
    imagesc(velo_axis, range_axis, db(speed_profile_tx(:,:,1)))
    xlabel("velocity (m/s)", 'fontsize',fontSize,'FontName','Times New Roman')
    ylabel("distance (m)", 'fontsize',fontSize,'FontName','Times New Roman')
    title("Range-Doppler Result", 'fontsize',fontSize,'FontName','Times New Roman')
figure(10009)
    imagesc(velo_axis, range_axis, abs(detect_profile))
    xlabel("velocity (m/s)", 'fontsize',fontSize,'FontName','Times New Roman')
    ylabel("distance (m)", 'fontsize',fontSize,'FontName','Times New Roman')
    title("CA-CFAR Result", 'fontsize',fontSize,'FontName','Times New Roman')
figure(10010)
    imagesc(an_axis_az, range_axis, db(az_rmap));
    xlabel("degree(��)",'fontsize',fontSize,'FontName','Times New Roman')
    ylabel("distance(m)",'fontsize',fontSize,'FontName','Times New Roman')
    title("Azimuth Range-Angle Result",'fontsize',fontSize,'FontName','Times New Roman')
figure(10012)
    imagesc(an_axis_el, range_axis, db(el_rmap));
    xlabel("degree(��)",'fontsize',fontSize,'FontName','Times New Roman')
    ylabel("distance(m)",'fontsize',fontSize,'FontName','Times New Roman')
    title("Elevation Range-Angle Result",'fontsize',fontSize,'FontName','Times New Roman')
    
if mode == MODE_WALL_PARA
    [detect_units, amps] = rd_select_unit(abs(squeeze(speed_profile(:,:,1))), abs(az_rmap), abs(el_rmap), ...
    range_axis, velo_axis, an_axis_az);
    detect_results = [detect_results; detect_units];
    amp_results    = [amp_results; amps];
    if fix(xx / 30) ~= cluster_index
        cluster_index = fix(xx / 30);
        [cluster, cluster_num] = DBSCAN(detect_results); % ����취�˳�����
        detect_results2 = cluster(:,1:3);                 % �˳������������ 
        [center, data_new] = xmeans(detect_results2, 20);
        figure(10013)
        for ii = 1:size(center, 1)
            data_scatter = data_new(find(data_new(:,4)==ii), 1:3);
            scatter3(data_scatter(:,1),data_scatter(:,2),data_scatter(:,3), 5)
            hold on
            scatter3(center(ii,1),center(ii,2),center(ii,3), 50, 'filled')
        end
        hold off
        title("X-means cluster")
        xlabel("Range (meter)")
        ylabel("Velocity (m/s)")
        zlabel("Angle (rad)")
        legend  

        figure(10014)
        for ii = 1:cluster_num
            data_scatter = cluster(find(cluster(:,4)==ii), 1:3);
            scatter3(data_scatter(:,1),data_scatter(:,2),data_scatter(:,3), 5)
            hold on
        end
        hold off
        title("DBSCAN cluster")
        xlabel("Range (meter)")
        ylabel("Velocity (m/s)")
        zlabel("Angle (rad)")
        legend       

        [X1, X2] = observe(data_new, center, Wall_X1 ,Wall_X2);
        if size(X1, 1) > 1
            index_W1 = []; 
            for ii = 1:size(X1, 1)
                index_W1_ = find(abs(detect_results(:,1) - X1(ii,1)) < 0.25);
                index_W1 = [index_W1; index_W1_];
            end
            AMP_W1 = amp_results(index_W1);
            index_AMP_W1 = find(AMP_W1 > 0.8 * max(AMP_W1));
            X1_ = mean(detect_results(index_W1(index_AMP_W1), 1));
            
            X1_index = find(abs(X1(:,1) - X1_) < 0.25);
            Wall_X1 = [Wall_X1; X1(X1_index, :)];
        else
            Wall_X1 = [Wall_X1; X1];
        end
        
        if size(X2, 1) > 1
            index_W2 = [];
            for ii = 1:size(X2, 1)
                index_W2_ = find(abs(detect_results(:,1) - X2(ii,1)) < 0.25);
                index_W2 = [index_W2; index_W2_];
            end

            AMP_W2 = amp_results(index_W2);
            index_AMP_W2 = find(AMP_W2 > 0.8 * max(AMP_W2));
            X2_ = mean(detect_results(index_W2(index_AMP_W2), 1));
            X2_index = find(abs(X2(:,1) - X2_) < 0.25);
            Wall_X2 = [Wall_X2; X2(X2_index, :)];
        else
            Wall_X2 = [Wall_X2; X2];
        end
        
        detect_results = [];
        amp_results = [];
    end

end

end

if mode == MODE_WALL_PARA
    figure(10023)
    X1_x = sin(Wall_X1(:,2)) .* Wall_X1(:, 1);
    X1_y = cos(Wall_X1(:,2)) .* Wall_X1(:, 1);
    X2_x = sin(Wall_X2(:,2)) .* Wall_X2(:, 1);
    X2_y = cos(Wall_X2(:,2)) .* Wall_X2(:, 1);
    scatter(X1_x, X1_y)
    hold on
    scatter(X2_x, X2_y)
    axis([-5 5 -1 9])
    xlabel("X-axis (m)")
    ylabel("Y-axis (m)")
    legend('Wall start point P1', 'Wall end point P2')
    
    drawWall_opt([X1_x, X1_y], [X2_x, X2_y]);
    
    % 45 angle:(3.2)
% 1.9 ��
% 2.8 ��
% 1.2 ��
% 1.3 ��

%  0 angle��(4.0)
% 1: 2.0meter  2:2.9meter(������ʹ��)(���䵽ǽ����), 3:1.3(��Ҫʹ�ã� 4��1.45
    
    X1_x = sin(Wall_X1(:,2)) .* Wall_X1(:, 1);
    X1_y = cos(Wall_X1(:,2)) .* Wall_X1(:, 1);
    X2_x = sin(Wall_X2(:,2)) .* Wall_X2(:, 1);
    X2_y = cos(Wall_X2(:,2)) .* Wall_X2(:, 1);
    drawWall_3d([-1.0 4.0; 1.0 4.0], [X1_x X1_y], [X2_x, X2_y], [0,0,2.9]);
    
    X1_x = sin(Wall_X1(:,2)) .* Wall_X1(:, 1);
    X1_y = cos(Wall_X1(:,2)) .* Wall_X1(:, 1);
    X2_x = sin(Wall_X2(:,2)) .* Wall_X2(:, 1);
    X2_y = cos(Wall_X2(:,2)) .* Wall_X2(:, 1);
    drawWall_3d([-0.8 2.0; 0.6 3.8], [X1_x X1_y], [X2_x, X2_y], [0,0,2.9]);
end

fclose(fid);