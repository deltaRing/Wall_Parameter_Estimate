% 绘制3d的墙体
% 输入参数1：墙体位置
% 输入参数2：检测结果
% 输入参数3：无人机所在位置
% 输出：无
function drawWall_3d(wall_loc, detect_loc_1, detect_loc_2, drone_loc)
    if size(wall_loc) ~= 2, return, end
    if isempty(detect_loc_1), return, end
    if isempty(detect_loc_2), return, end
    
    % 墙体位置
    wall_start  = wall_loc(1, :);
    wall_end    = wall_loc(2, :);
    wall_vec    = wall_end - wall_start;
    wall_angle  = atan2(wall_vec(2), wall_vec(1));
    wall_height = 1;
    
    % 滤除异常点
    startDBSCAN = DBSCAN(detect_loc_1, 0.5, 3);
    endDBSCAN   = DBSCAN(detect_loc_2, 0.5, 3);

    meanStart = mean(startDBSCAN(:,1:2), 1);
    meanEnd   = mean(endDBSCAN(:,1:2),   1);
    new_wall_start = detect_loc_1(find(sum(abs(detect_loc_1 - meanStart), 2) < 0.5), :);
    new_wall_end   = detect_loc_2(find(sum(abs(detect_loc_2 - meanEnd), 2) < 0.5), :);
    
    % 生成墙体
    wall_angle_2  = pi / 2;
    if wall_angle ~= 0
        wall_angle_2 = -1 / wall_angle;
    end
    
    % 墙体厚度
    thickness    = 0.1; 
    % 计算delta增量
    nega_delta   = -thickness * [cos(wall_angle_2), sin(wall_angle_2)];
    posi_delta   = thickness  * [cos(wall_angle_2), sin(wall_angle_2)];
    % 墙体顶点
    wall_c1 = wall_start + nega_delta;
    wall_c2 = wall_start + posi_delta;
    wall_c3 = wall_end   + nega_delta;
    wall_c4 = wall_end   + posi_delta;
    
    vertex = [
        wall_c1(1) wall_c1(2) 0;
        wall_c2(1) wall_c2(2) 0;
        wall_c3(1) wall_c3(2) 0;
        wall_c4(1) wall_c4(2) 0;
        wall_c1(1) wall_c1(2) wall_height;
        wall_c2(1) wall_c2(2) wall_height;
        wall_c3(1) wall_c3(2) wall_height;
        wall_c4(1) wall_c4(2) wall_height;
        ]; 
    
    % 6个面顶点编号
    faces=[2 4 3 1;
            2 4 8 6;
            2 1 5 6;
            1 3 7 5;
            4 3 7 8;
            8 7 5 6];

    % 作图
    patch('Faces',faces,'Vertices',vertex,'Facecolor','Green')
    axis([-5, 5, -1, 6, 0, 3])
    view(3)
    view(-45, 60)
    grid on
    
    hold on
    plot_drone(drone_loc);
    
    % 绘制墙体所在的位置
    % 采用椭圆形来进行绘制
    for ws = 1:size(new_wall_start, 1)
        [X,Y,Z] = cylinder(0.05);
        
        wsx = new_wall_start(ws, 1);
        wsy = new_wall_start(ws, 2);
        
        X = X + wsx; Y = Y + wsy; Z = Z * wall_height;
        hold on
        tar = mesh(X, Y, Z);
        fill3(X(1,:),Y(1,:),Z(1,:),'b');
        fill3(X(end,:),Y(end,:),Z(end,:),'b');
        set(tar,'FaceColor','b')
    end
    
    for we = 1:size(new_wall_end, 1)
        [X,Y,Z] = cylinder(0.05);
        
        wex = new_wall_end(we, 1);
        wey = new_wall_end(we, 2);
        
        X = X + wex; Y = Y + wey; Z = Z * wall_height;
        hold on
        tar = mesh(X, Y, Z);
        fill3(X(1,:),Y(1,:),Z(1,:),'b');
        fill3(X(end,:),Y(end,:),Z(end,:),'b');
        set(tar,'FaceColor','b')
    end
    
    estimated_wall_vec = meanEnd - meanStart;
    e_wall_angle = atan2(estimated_wall_vec(2), estimated_wall_vec(1));
     % 生成墙体
    e_wall_angle2  = pi / 2;
    if e_wall_angle ~= 0
        e_wall_angle2 = -1 / e_wall_angle;
    end
    % 计算增量
    nega_delta   = -thickness * [cos(e_wall_angle2), sin(e_wall_angle2)];
    posi_delta   = thickness  * [cos(e_wall_angle2), sin(e_wall_angle2)];
    % 计算墙体顶点
    e_wall_c1 = meanStart + nega_delta;
    e_wall_c2 = meanStart + posi_delta;
    e_wall_c3 = meanEnd   + nega_delta;
    e_wall_c4 = meanEnd   + posi_delta;
    
    e_vertex = [
        e_wall_c1(1) e_wall_c1(2) 0;
        e_wall_c2(1) e_wall_c2(2) 0;
        e_wall_c3(1) e_wall_c3(2) 0;
        e_wall_c4(1) e_wall_c4(2) 0;
        e_wall_c1(1) e_wall_c1(2) wall_height;
        e_wall_c2(1) e_wall_c2(2) wall_height;
        e_wall_c3(1) e_wall_c3(2) wall_height;
        e_wall_c4(1) e_wall_c4(2) wall_height;
        ]; 

    % 作图
    patch('Faces',faces,'Vertices',e_vertex,'Facecolor','Blue')
    % 调整视野角度
    view(45, 40)
    xlabel("X-Axis (m)",'FontSize',14, "FontName","Times New Roman")
    ylabel("Y-Axis (m)",'FontSize',14, "FontName","Times New Roman")
    zlabel("Z-Axis (m)",'FontSize',14, "FontName","Times New Roman")
    title("The estimated scenario",'FontSize',14, "FontName","Times New Roman")
end