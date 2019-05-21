function [] = plotBrachBot(Y,T,l1,l2,lc1,lc2)
%PLOTBRACHBOT Plots robot as a function of current state, also visulazies
%velocites
%   Detailed explanation goes here

% Color map for identification
colorblue = [0 0.447 0.741];
colorred = [0.85 0.325 0.098];
coloryellow = [0.9290 0.6940 0.1250];
colorgreen = [0.4660 0.6740 0.1880];

sl = figure('Name','Brachiating Robot');
set(gcf, 'Position',  [100 100 1090 590]); % [left bottom width height]
subplot('Position',  [0.05 0.2 0.3 0.6]);
theta1 = Y(:,1);
theta2 = Y(:,2);
% theta1 = Y(1,1);
% theta2 = Y(1,2);

j1x = sin(theta1)*l1;
j1z = -cos(theta1)*l1;
j2x = j1x + l2*sin(theta1+theta2);
j2z = j1z - l2*cos(theta1+theta2);
x = [j1x,j2x];
z = [j1z,j2z];
COMx = [(sin(theta1)*lc1+j1x+lc2*sin(theta1+theta2))./2];
COMz = [-cos(theta1)*lc1 + j1z-lc2*cos(theta1+theta2)];
COMvx = [0;diff(COMx)./diff(T)];
COMvz = [0;diff(COMz)./diff(T)];

plot([-1,1],[0,0],'LineWidth',3,'Color','k');
hold on;
plot(-.6,0,'Marker','o','LineWidth',10,'Color','k');
plot(.6,0,'Marker','o','LineWidth',10,'Color','k');
hh2 = plot([0,x(1,1);x(1,1),x(1,2)],[0,z(1,1);z(1,1),z(1,2)],'.-','Marker','o','MarkerSize',5,'LineWidth',5);

axis([-1 1 -1.5 0.5]), grid on;
ht = title(sprintf('Time: %0.2f sec', T(1)));
% Position graph
subplot('Position',  [0.4 0.56 0.2 0.24]);
    px = line(T(1:1), COMx(1:1), 'LineWidth', 2, 'Color',colorred);
%     plot(T(1:1),COMx(1:1),'LineWidth',2,'Color',colorred);
%     p3 = plot(T(1),COMz(1),'LineWidth',2,'Color',colorblue);
    mpx = line(T(1), COMx(1), 'Marker', 'o', 'LineWidth', 4, 'Color',colorred);
%     m3 = line(T(1),COMz(1),'Marker', 'o','LineWidth',4,'Color',colorblue);
    title('Position of COM');
    axis([0,T(end),-1,1])
    grid on
    xlabel('time (sec)');ylabel('position (m)');
    % Velocity graph
subplot('Position',  [0.4 0.2 0.2 0.24]);
    vx = plot(T(1:1),COMvx(1:1),'LineWidth',2,'Color',coloryellow);
    mvx = line(T(1),COMvx(1),'Marker','o','LineWidth',4,'Color',coloryellow);
    title('Velocity of COM');
    axis([0,T(end),-2,2])
    grid on
    xlabel('time (sec)');ylabel('velocity (m/s)');
    
% Orbit in State Space    
subplot('Position',  [0.65 0.2 0.3 0.6]);
    ss = plot(COMx(1:1),COMvx(1:1),'LineWidth',2,'Color', colorgreen);
    mss = line(COMx(1),COMvx(1),'Marker', 'o','LineWidth',4,'Color', colorgreen);
    title('Orbit in State Space');
    axis([1.2*min(COMx),1.2*max(COMx),-1.2*max(COMvx),1.2*max(COMvx)])
    grid on; grid minor
    xlabel('position (m)');ylabel('velocity (m/s)');


% tic; 
for id = 1:45:length(T)
    set(hh2(1), 'Xdata',[0,x(id,1)],'Ydata',[0,z(id,1)]);
    set(hh2(2), 'Xdata',x(id,:)    ,'Ydata',z(id,:));
    set(ht, 'String', sprintf('Time: %0.2f sec', T(id)));
    set(px(1), 'XData', T(1:id)  , 'YData', COMx(1:id));
    set(mpx(1), 'XData', T(id)    , 'YData', COMx(id));
    set(vx(1), 'XData', T(1:id)  , 'YData', COMvx(1:id));
    set(mvx(1), 'XData', T(id)    , 'YData', COMvx(id));
    set(ss(1), 'XData', COMx(1:id), 'YData', COMvx(1:id));
    set(mss(1), 'XData', COMx(id) , 'YData', COMvx(id));
    drawnow;
    
    frame = getframe(sl);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if id == 1
        imwrite(imind,cm,'BBnoControl.gif','gif','LoopCount',inf);
    else
        imwrite(imind,cm,'BBnoControl.gif','gif','WriteMode','append','DelayTime',.001);
    end
end
% for i=1:10:length(T)
%     theta1 = Y(i,1);
%     theta2 = Y(i,2);
%     j1x = sin(theta1)*l1;
%     j1z = -cos(theta1)*l1;
%     j2x = j1x + l2*sin(theta1+theta2);
%     j2z = j1z - l2*cos(theta1+theta2);
%     plot([0,j1x],[0,j1z],'r','LineWidth',5);
%     hold on;
%     plot([j1x,j2x],[j1z,j2z],'b','LineWidth',5);
%     axis([-1 1 -2 0]), grid on;
%     pause(0.01);
%     hold off;
% end

