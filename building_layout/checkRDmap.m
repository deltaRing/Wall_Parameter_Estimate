% ���RD��ͼ �����˳� ����ĳɷ� ͬʱ���ݵ��ٵ�Ԫ ȷ�����˻��ز��ɷ�
% ���룺������ͼ rangeProfile
% ���룺�ٶ�FFT���� veloFFTNum
% ���룺�ٶ���
% ���룺�ٶ���ֵ
% ���������Ȥ�ľ��뵥Ԫ��Χ
function ROIs = checkRDmap(rangeProfile, veloFFTNum, velo_axis, velo_threshold)
    if nargin == 3
        velo_threshold = 1.0; % 1.0�����µĶ��������˻������
    end
    
    ROIs = [];
    % 2D-FFT
    rd_map = fftshift(fft(rangeProfile, veloFFTNum, 2), 2); % ��ȡ����-��������ͼ
    % �ҵ���С��
    [~, index]   = find(velo_threshold > velo_axis);
    disable_velo = veloFFTNum - length(index);
    % CA-CFAR 2D-FFT
    [detect_profile, detect_res] = CFAR(rd_map, 0, disable_velo);
%     figure(10003)
%     imagesc(abs(rd_map))
figure(10004)
imagesc(abs(detect_profile))
    
end