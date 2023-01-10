function animation = drone_Animation(video, dispWater, x,y,z,FoVh,FoVv, tx,ty,tz, Ns,xSource,ySource,waveLength,decayrate,amplitude)
% This Animation code is for QuadCopter. Written by Jitendra Singh
tt=0:length(x);
yaw   = ones(length(x))*0.785;
roll  = zeros(length(x));
pitch = zeros(length(x));

%% setup video
title = 'Multi-Castaway Track';
if(video)
    % create the video writer with 10 fps
    videoFileName = [title,datestr(now,'dd_mm_yy_HH_MM')];
    %[videoFileName, val] = nextname(title,'01','.avi');
    %videoFileName = [plotTitle,'.avi'];
    writerObj = VideoWriter(videoFileName, 'MPEG-4');
    writerObj.FrameRate = 8;
    % open the video writer
    open(writerObj);
end
%% Define design parameters

D2R = pi/180;
R2D = 180/pi;
b   = 15.0;   % the length of total square cover by whole body of quadcopter in meter
a   = b/3;   % the legth of small square base of quadcopter(b/4)
H   = 0.06;  % hight of drone in Z direction (4cm)
H_m = H+H/2; % hight of motor in z direction (5 cm)
r_p = b/4;   % radius of propeller
%% Conversions
ro = 45*D2R;                   % angle by which rotate the base of quadcopter
Ri = [cos(ro) -sin(ro) 0;
    sin(ro) cos(ro)  0;
    0       0       1];     % rotation matrix to rotate the coordinates of base
base_co = [-a/2  a/2 a/2 -a/2; % Coordinates of Base
    -a/2 -a/2 a/2 a/2;
    0    0   0   0];
base = Ri*base_co;             % rotate base Coordinates by 45 degree
to = linspace(0, 2*pi);
xp = r_p*cos(to);
yp = r_p*sin(to);
zp = zeros(1,length(to));
%% Define Figure plot
fig1 = figure('units','normalized','DefaultTextFontName', "Verdana", 'DefaultAxesFontName', "Verdana",'defaultAxesTickLabelInterpreter','latex',  'defaultLegendInterpreter','latex', 'OuterPosition', [0.063020833333333,0.037962962962963,0.883333333333333,0.95]);%
hg   = gca;
set(hg,'FontSize',15)
% view(28.865342681043614,28.988717694659215);%view(0,90);%view(290,35)%
grid on;
set(hg,'xminorgrid','on','yminorgrid','on')
axis equal;
minLim = round(min(min(x),min(y))-10);%min(min(min(x),min(y)),min(min(xSource),min(ySource)))-50;
maxLim = round(max(max(x),max(y))+20);%max(max(max(x),max(y)),max(max(xSource),max(ySource)))+50;
xlim([minLim maxLim]); ylim([minLim maxLim]); zlim([0 max(z)]);
xlabel('X[m]','units','normalized', 'fontSize',25, 'fontName', "Verdana", 'Interpreter', "latex");%, 'position',[0.906141517903917,0.084864543553277,0], 'rotation',60
ylabel('Y[m]','units','normalized', 'fontSize',25, 'fontName', "Verdana", 'Interpreter', "latex");%, 'position',[0.374522172148518,0.010389079767215,0], 'rotation', -12
zlabel('Z[m]','units','normalized', 'fontSize',25, 'fontName', "Verdana", 'Interpreter', "latex");%, 'position',[-0.076357956425874,0.347299921688738,0]
xticks([minLim:25:maxLim]); yticks([minLim:25:maxLim]); zticks([0:25:max(z)]);
hold(gca, 'on');

xlim([0 125]);
ylim([0 125]);

%% add water surface
if(dispWater==true)
    [Xw, Yw] = meshgrid(minLim:0.5:maxLim); Hw = 20; Zw = Hw + zeros(size(Xw));
    water = surf(Xw,Yw,Zw); water.FaceColor = [0 0.5 1];  %water.FaceAlpha = 0.9;
    water.EdgeColor = 'w'; water.EdgeAlpha = 0.15;
    g = 9.8;
    
    uni_random_noise_gen = @(min,max) min + (max-min).*rand();
    norm_random_noise_gen = @(mu,s) s.*randn() + mu;
    
    waveNum = @(wl) 2*pi/wl;
    wavePeriod = @(wl) sqrt(2*pi*wl/g*tanh(waveNum(wl)*Hw));
    waveFreq = @(wl) 2*pi/wavePeriod(wl);
    
    wave_eq_2 = @(axx,t,amp,dr,wl) (waveFreq(wl)*amp)*exp(-dr*(axx))/2 .* sin((waveNum(wl)*axx-(waveFreq(wl)*t)));
    
    D = {};
    for n=1:Ns
        d = sqrt((Xw - xSource(n)).^2 + (Yw - ySource(n)).^2); D = [D,d];
    end
    twave = zeros(1,Ns);
else
    Hw = 20;
    water.ZData = Hw;
end
dt=1;

%% add spheres for targets
[Xb,Yb,Zb] = sphere(20); clrs = get(gca,'colororder');
Xb = Xb*4; Yb = Yb*4; Zb=Zb*4;
[Nb,~,~] = size(tx);
balls = {};
for n = 1:Nb
    ball = surf(Xb + tx(n,1), Yb + ty(n,1), Zb + Hw + tz(n,1));
    ball.FaceColor = clrs(n,:); ball.EdgeAlpha = 0.1; balls = [balls, ball];
end

%% Design Different parts
% design the base square
drone(1) = patch([base(1,:)],[base(2,:)],[base(3,:)],'k');
drone(2) = patch([base(1,:)],[base(2,:)],[base(3,:)+H],'k');
alpha(drone(1:2),0.7);
% design 2 parpendiculer legs of quadcopter
% [xcylinder ycylinder zcylinder] = cylinder([H/2 H/2]);
% drone(3) =  surface(b*zcylinder-b/2,ycylinder,xcylinder+H/2,'facecolor','k');
% drone(4) =  surface(ycylinder,b*zcylinder-b/2,xcylinder+H/2,'facecolor','k') ;
% alpha(drone(3:4),0.6);
% % design 4 cylindrical motors
% drone(5) = surface(xcylinder+b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','k');
% drone(6) = surface(xcylinder-b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','k');
% drone(7) = surface(xcylinder,ycylinder+b/2,H_m*zcylinder+H/2,'facecolor','k');
% drone(8) = surface(xcylinder,ycylinder-b/2,H_m*zcylinder+H/2,'facecolor','k');
% alpha(drone(5:8),0.7);
% design 4 propellers
% drone(9)  = patch(xp+b/2,yp,zp+(H_m+H/2),'EdgeColor','r', 'FaceColor', 'k', 'FaceAlpha', 0.5,'LineWidth',0.5);
% drone(10) = patch(xp-b/2,yp,zp+(H_m+H/2),'EdgeColor','g', 'FaceColor', 'k', 'FaceAlpha', 0.5,'LineWidth',0.5);
% drone(11) = patch(xp,yp+b/2,zp+(H_m+H/2),'EdgeColor','r', 'FaceColor', 'k', 'FaceAlpha', 0.5,'LineWidth',0.5);
% drone(12) = patch(xp,yp-b/2,zp+(H_m+H/2),'EdgeColor','g', 'FaceColor', 'k', 'FaceAlpha', 0.5,'LineWidth',0.5);
% alpha(drone(9:12),0.3);
%% create a group object and parent surface
combinedobject = hgtransform('parent',hg );
set(drone,'parent',combinedobject)
%  drawnow

for i = 1:length(x)
    if(dispWater==true)
        water.ZData = Zw;
        
        %movieVector(i) =  getframe(fig1);
        %delete(b);
        
        %update waves
        
        for sourceIndex=1:Ns
            twave(sourceIndex) = twave(sourceIndex) + dt;
            % Accumulate wave positions to reflect interference
            d = D{sourceIndex};
            
            water.ZData = water.ZData + wave_eq_2(d, twave(sourceIndex),amplitude(sourceIndex),decayrate(sourceIndex),waveLength(sourceIndex));
        end
    end
    %     set(water, 'ZData', zdata);
    
    %update target balls
    for n=1:Nb
        %         tz(n,i) = max(max(zdata));
        set(balls(n), 'XData', Xb + tx(n,i),'YData', Yb + ty(n,i),'ZData',Zb + tz(n,i)); % Zb + max(max(water.ZData))
        plot3(tx(n,1:i),ty(n,1:i),tz(n,1:i),'LineWidth',1.5,'color',clrs(n,:));
    end
    
    %update FoV
    try
        delete(fov_plot)
        delete(ba)
        delete(timestep_plot)
        %         delete(bb)
        delete(fov_plot1)
        delete(fov_plot2)
        delete(fov_plot3)
        delete(fov_plot4)
    catch
    end
    X=([-z(i)*tand(FoVh/2), z(i)*tand(FoVh/2), z(i)*tand(FoVh/2), -z(i)*tand(FoVh/2),-z(i)*tand(FoVh/2)]);
    Y=([-z(i)*tand(FoVv/2), -z(i)*tand(FoVv/2), z(i)*tand(FoVv/2), z(i)*tand(FoVv/2),-z(i)*tand(FoVv/2)]);
    x_coor= x(i) + X;% + Xrot;
    y_coor= y(i) + Y;% + Yrot;
    fov_plot = plot3(x_coor, y_coor, ones(1,length(x_coor))*max(max(water.ZData)), '-.', 'LineWidth', 2, 'Color', 'r');
%     patch([x_coor nan],[y_coor nan],[ones(1,length(x_coor))*max(max(water.ZData)) nan],[ones(1,length(x_coor))*i nan], 'edgecolor', 'interp'); 

%     fov_plot1 = plot3([x_coor(1), x(i)], [y_coor(1) y(i)], [max(max(water.ZData)), z(i)], '-.r');
%     fov_plot2 = plot3([x_coor(2), x(i)], [y_coor(2) y(i)], [max(max(water.ZData)), z(i)], '-.r');
%     fov_plot3 = plot3([x_coor(3), x(i)], [y_coor(3) y(i)], [max(max(water.ZData)), z(i)], '-.r');
%     fov_plot4 = plot3([x_coor(4), x(i)], [y_coor(4) y(i)], [max(max(water.ZData)), z(i)], '-.r');
    
    tailLength = 10;
    ba = plot3(x(max(i-tailLength,1):i),y(max(i-tailLength,1):i),z(max(i-tailLength,1):i)-1,'LineWidth',2,'color','k');
    %     bb = plot3(x(i:min(i+tailLength,length(x))),y(i:min(i+tailLength,length(x))),z(i:min(i+tailLength,length(x)))-1,'LineWidth',2,'color',[0.5 0.5 0.5]);
    
    translation = makehgtform('translate',...
        [x(i) y(i) z(i)]);
    %set(combinedobject, 'matrix',translation);
    rotation1 = makehgtform('xrotate',(pi/180)*(roll(i)));
    rotation2 = makehgtform('yrotate',(pi/180)*(pitch(i)));
    rotation3 = makehgtform('zrotate',yaw(i));
    %scaling = makehgtform('scale',1-i/20);
    set(combinedobject,'matrix',...
        translation*rotation3*rotation2*rotation1);
%      dim = [0.601785714285717,0.862165057160846,0.135141315470185,0.0535905698488];
%      timestep_plot = annotation('textbox',dim,'EdgeColor','k','LineWidth',2,'BackgroundColor','w','String',['Timestep: ',num2str(i), 's']...
%                 ,'Color','k','FontSize',25,'FontWeight','Bold','FitBoxToText','on','interpreter','latex');
%     
    drawnow
    %     pause(0.5);
    
    if(video)
        writeVideo(writerObj, getframe(fig1));
    end
    
    if(i==1 || i==10 || i==17 || i==23 || i==35)
        fprintf('take snapshot i=%d\n', i);
    end
end
if(video)
    % close the writer object
    close(writerObj);
    disp("Video Done")
end
