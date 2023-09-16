% 利用霍夫变换对距离像进行直线检测
% 输入：img：图像
% 输出：result：检测结果(距离向)
function result = houghLineDetect(img)
    BW = edge(img,'canny');    % 使用 Canny 方法查找边缘，创建二值图像
    [H, T, R] = hough(BW);     % R = x * cos(T) + y * sin(T);
    result = [];
    % figure(10003);
    % imshow(H,[], 'XData',T,'YData', R, 'InitialMagnification','fit');
    % xlabel('\theta'),ylabel('\rho');    %为横纵坐标加上标签，‘\  ’为希腊字母
    % axis on,axis normal,hold on;    
    
    % 找出五个极值点 阈值为H中最大值乘以0.3
    % 只有超过该阈值才会被认定为一条符合条件的直线，在图像中表现为一个点。
    P = houghpeaks(H,5,'threshold',ceil(0.3*max(H(:))));  
    
    x = T(P(:,2)); y = R(P(:,1));   
    % x为矩阵P第二列全部坐标的角度值，y为矩阵P的第一列全部坐标的距离值。
    % plot(x,y,'s','color','white');  %绘制二维线图，坐标点形状为方形（‘s’），颜色为白色
    %% 查找线条并对其绘画
    lines = houghlines(BW,T,R,P,'FillGap',5,'MinLength',7); 
    % houghlines函数可以找出标准霍夫变换的二值图像线条，其基于霍夫变换
    % BW是8位的单通道二进制图像
    % 'FillGap'是一个正实数，表示同一图像中两条线段的距离。当两条线的距离小于指定值时，houghlines函数就会将这两条线合并为一条线。（这里FillGap的值为5）
    % 'MinLength'是一个正实数，用来确定是否保存线条的长度小于这个值，线条就会被擦除，否则会保存。（这里MinLength的值为7）
    % figure(10004), imshow(img), hold on;   %figure创建一个新的图窗窗口
    max_len = 0;    %初始化max_len
    for k = 1:length(lines) %遍历lines
        xy = [lines(k).point1;lines(k).point2];
        result = [result lines(k).point2(2)];
        % plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');    %画线段，宽为2，绿色
        % plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');   %线段起点，形状叉号，线宽为2，颜色为黄
        % plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');  %线段终点，形状叉号，线宽为2，颜色为红
        % 可以注意一下线段和点的不同表示方式
        len = norm(lines(k).point1 - lines(k).point2);  %选出最长的线段
        if(len > max_len)
            max_len = len;
            xy_long = xy;
        end
    end
    
    result = unique(result);
    %% 将最长线段设为青色以突出显示
    % plot(xy_long(:,1),xy_long(:,2),'LineWidth',2,'Color','cyan');
end