% 绘制无人机
% plot_drone

% 输入1：posi_d:  无人机当前位置
% 可选输入1：blade_r: 桨叶半径
% 可选输入2：frame_r: 无人机框架半径
% 可选输入3：frame_h: 无人机框架高度

function plot_drone(posi_d, blade_r, frame_r, frame_h)
    if nargin == 1
        blade_r = 0.25;
        frame_r = 0.05;
        frame_h = 0.3;
    end
    
    [Xb,Yb,Zb] = cylinder(blade_r);
    [Xd,Yd,Zd] = cylinder(frame_r);
    Zb = Zb * 0.1;
    Zd = Zd * -frame_h;
    
    offset_x = blade_r + frame_r / 2;
    offset_y = blade_r + frame_r / 2;
    
    % 无人机四个桨叶的位置
    Xb1 = Xb + offset_x; Yb1 = Yb + offset_y;
    Xb2 = Xb + offset_x; Yb2 = Yb - offset_y;
    Xb3 = Xb - offset_x; Yb3 = Yb + offset_y;
    Xb4 = Xb - offset_x; Yb4 = Yb - offset_y;
    % 无人机的位置
    Dx  = posi_d(1);
    Dy  = posi_d(2);
    Dz  = posi_d(3);
    
    Blade1 = mesh(Xb1 + Dx, Yb1 + Dy, Zb + Dz);
    hold on
    Blade2 = mesh(Xb2 + Dx, Yb2 + Dy, Zb + Dz);
    Blade3 = mesh(Xb3 + Dx, Yb3 + Dy, Zb + Dz);
    Blade4 = mesh(Xb4 + Dx, Yb4 + Dy, Zb + Dz);
    
    xb1 = Xb1 + Dx; yb1 = Yb1 + Dy; zb1 = Zb + Dz;
    xb2 = Xb2 + Dx; yb2 = Yb2 + Dy; zb2 = Zb + Dz;
    xb3 = Xb3 + Dx; yb3 = Yb3 + Dy; zb3 = Zb + Dz;
    xb4 = Xb4 + Dx; yb4 = Yb4 + Dy; zb4 = Zb + Dz;
    
    EndPlate1=fill3(xb1(1,:),yb1(1,:),zb1(1,:),'r');
    EndPlate2=fill3(xb2(1,:),yb2(1,:),zb2(1,:),'r');
    EndPlate3=fill3(xb3(1,:),yb3(1,:),zb3(1,:),'r');
    EndPlate4=fill3(xb4(1,:),yb4(1,:),zb4(1,:),'r');
    EndPlate5=fill3(xb1(end,:),yb1(end,:),zb1(end,:),'r');
    EndPlate6=fill3(xb2(end,:),yb2(end,:),zb2(end,:),'r');
    EndPlate7=fill3(xb3(end,:),yb3(end,:),zb3(end,:),'r');
    EndPlate8=fill3(xb4(end,:),yb4(end,:),zb4(end,:),'r');
    
    Drone  = mesh(Xd + Dx, Yd + Dy, Zd + Dz);
    
    % 设置颜色
    set(Blade1,'FaceColor','r')
    set(Blade2,'FaceColor','r')
    set(Blade3,'FaceColor','r')
    set(Blade4,'FaceColor','r')
    set(Drone,'FaceColor','r')
    set(Blade1,'EdgeAlpha',0)
    set(Blade2,'EdgeAlpha',0)
    set(Blade3,'EdgeAlpha',0)
    set(Blade4,'EdgeAlpha',0)
    set(Drone,'EdgeAlpha',0)
 
    hold off
end