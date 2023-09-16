% ���û���任�Ծ��������ֱ�߼��
% ���룺img��ͼ��
% �����result�������(������)
function result = houghLineDetect(img)
    BW = edge(img,'canny');    % ʹ�� Canny �������ұ�Ե��������ֵͼ��
    [H, T, R] = hough(BW);     % R = x * cos(T) + y * sin(T);
    result = [];
    % figure(10003);
    % imshow(H,[], 'XData',T,'YData', R, 'InitialMagnification','fit');
    % xlabel('\theta'),ylabel('\rho');    %Ϊ����������ϱ�ǩ����\  ��Ϊϣ����ĸ
    % axis on,axis normal,hold on;    
    
    % �ҳ������ֵ�� ��ֵΪH�����ֵ����0.3
    % ֻ�г�������ֵ�Żᱻ�϶�Ϊһ������������ֱ�ߣ���ͼ���б���Ϊһ���㡣
    P = houghpeaks(H,5,'threshold',ceil(0.3*max(H(:))));  
    
    x = T(P(:,2)); y = R(P(:,1));   
    % xΪ����P�ڶ���ȫ������ĽǶ�ֵ��yΪ����P�ĵ�һ��ȫ������ľ���ֵ��
    % plot(x,y,'s','color','white');  %���ƶ�ά��ͼ���������״Ϊ���Σ���s��������ɫΪ��ɫ
    %% ��������������滭
    lines = houghlines(BW,T,R,P,'FillGap',5,'MinLength',7); 
    % houghlines���������ҳ���׼����任�Ķ�ֵͼ������������ڻ���任
    % BW��8λ�ĵ�ͨ��������ͼ��
    % 'FillGap'��һ����ʵ������ʾͬһͼ���������߶εľ��롣�������ߵľ���С��ָ��ֵʱ��houghlines�����ͻὫ�������ߺϲ�Ϊһ���ߡ�������FillGap��ֵΪ5��
    % 'MinLength'��һ����ʵ��������ȷ���Ƿ񱣴������ĳ���С�����ֵ�������ͻᱻ����������ᱣ�档������MinLength��ֵΪ7��
    % figure(10004), imshow(img), hold on;   %figure����һ���µ�ͼ������
    max_len = 0;    %��ʼ��max_len
    for k = 1:length(lines) %����lines
        xy = [lines(k).point1;lines(k).point2];
        result = [result lines(k).point2(2)];
        % plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');    %���߶Σ���Ϊ2����ɫ
        % plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');   %�߶���㣬��״��ţ��߿�Ϊ2����ɫΪ��
        % plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');  %�߶��յ㣬��״��ţ��߿�Ϊ2����ɫΪ��
        % ����ע��һ���߶κ͵�Ĳ�ͬ��ʾ��ʽ
        len = norm(lines(k).point1 - lines(k).point2);  %ѡ������߶�
        if(len > max_len)
            max_len = len;
            xy_long = xy;
        end
    end
    
    result = unique(result);
    %% ����߶���Ϊ��ɫ��ͻ����ʾ
    % plot(xy_long(:,1),xy_long(:,2),'LineWidth',2,'Color','cyan');
end