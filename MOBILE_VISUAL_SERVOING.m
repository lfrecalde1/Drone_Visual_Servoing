%XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
%XXXXXXXXXXXXXXXXXXX TRAJECTORY CONTROL DJI DRONE XXXXXXXXXXXXXXXXXXXXXXXXX
%XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

%% CLEAN VARIABLES
clc,clear all,close all;

%% DEFINITION OF TIME VARIABLES
ts = 0.1;
tf = 60;
to = 0;
t = (to:ts:tf);

%% ROS PARAMETER FOR COMUNICATION
rosshutdown
rosinit('192.168.1.2', 'NodeHost', '192.168.1.2', 'Nodename', '/Matlab_Visual_Servoing');

%% OBJECTS CREATION OF TOPICS ROS
robot = rospublisher('/Pioner_3dx/cmd_vel');
velmsg = rosmessage(robot);
odom = rossubscriber('/Pioner_3dx/odom');
pixel_data = rossubscriber('/Pioner_3dx/Pixels_data');

%% CONSTANTS VALUES OF THE ROBOT
a = 0.2; 
b = 0.1;
c = 0.2;
L = [a, b, c];
rotz = -pi/2;
roty = pi/2;

%% CONSTANT VALUES OF CAMERA
fx = 6.7630115434170293e+03 * 0.0774;
fy = 6.8516194384907485e+03 * 0.0774;
u0 = 2.4597651153505402e+02;
v0 = 1.9955473351505680e+02;

camera_parameters = [fx, fy, u0, v0];
%% READ VALUES FOR INITIAL CONDITIONS
[h, hp] = odometry(odom, L);
%% READ VALUES OF PIXELS
[uv, z, obj, flag] = image_data(pixel_data, camera_parameters);

% Projection obj general frame
obj_3d = Object_3d(obj, h, rotz, roty);

%% DESIRED PIXELS VALUES OF THE SYSTEM

uv_d = [-40.9765, 2.4453, 136.0235, -115.5547, 190.0235, 80.4453, -23.9765, 67.4453]';

for k = 1:length(t)
    tic;
    %% GENERATE VECTOR OF PIXELS ERRORS
    Pe(:,k) = uv_d-uv(:,k);
    
    %% CONTROL LAW F VISUAL SERVOING SYSTEM
    control = visual_servoing_control_mobile(camera_parameters, L, uv(:,k), uv_d, h(:,k), z(:,k), rotz, roty);
    
    %% OBTAIN CONTROL VALUES OF THE VECTOR
    u(k) = control(1);
    w(k) = control(2);
    
    %% SEND CONTROL VALUES TO THE ROBOT
    send_velocities(robot, velmsg, [u(k), 0, 0, 0, 0 , w(k)], flag(1,k))
    %% READ STATES VALUES OF THE ROBOT
    [h(:,k+1), hp(:,k+1)] = odometry(odom, L);
    
    %% READ PIXELS AND DEEP VALUE
    [uv(:,k+1), z(:,k+1), obj(:,k+1), flag(1,k+1)] = image_data(pixel_data, camera_parameters);
    obj_3d(:,k+1) = Object_3d(obj(:,k+1), h(:,k+1), rotz, roty);
    while(toc<ts)
    end
    toc;
end
%% SET VALUES TO ZERO ON THE DESIRED VELOCITIES
send_velocities(robot, velmsg, [0, 0, 0, 0, 0 , 0], 1)
rosshutdown;
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);

plot3(h(1,:),h(2,:),h(3,:),'-','Color',[226,76,44]/255,'linewidth',1); hold on;
grid on;
plot3(obj_3d(1,:),obj_3d(2,:),obj_3d(3,:),'Color',[100,76,44]/255,'linewidth',1); hold on;
plot3(obj_3d(4,:),obj_3d(5,:),obj_3d(6,:),'Color',[100,76,44]/255,'linewidth',1); hold on;
plot3(obj_3d(7,:),obj_3d(8,:),obj_3d(9,:),'Color',[100,76,44]/255,'linewidth',1); hold on;
plot3(obj_3d(10,:),obj_3d(11,:),obj_3d(12,:),'Color',[100,76,44]/255,'linewidth',1); hold on;
legend({'$h$','$O_1$','$O_2$','$O_3$','$O_4$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{System evolution}$','Interpreter','latex','FontSize',9);
ylabel('$[m]$','Interpreter','latex','FontSize',9);
xlabel('$[m]$','Interpreter','latex','FontSize',9);
xlim([0 5])
ylim([0 5])
zlim([0 2])
print -dpng Sistema
print -depsc Sistema

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1,1:length(u)),u,'Color',[223,67,85]/255,'linewidth',1); hold on
plot(t(1,1:length(w)),w,'Color',[56,171,217]/255,'linewidth',1); hold on
grid on;
legend({'$\mu$','$\omega$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Control Values}$','Interpreter','latex','FontSize',9);
ylabel('$[m/s][rad/s]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);
print -dpng Control
print -depsc Control

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
set(gca,'Ydir','reverse')

plot(uv(1,:),uv(2,:),'--','Color',[223,67,85]/255,'linewidth',1); hold on
plot(uv(3,:),uv(4,:),'--','Color',[56,171,217]/255,'linewidth',1); hold on
plot(uv(5,:),uv(6,:),'--','Color',[46,188,89]/255,'linewidth',1); hold on
plot(uv(7,:),uv(8,:),'--','Color',[56,10,217]/255,'linewidth',1); hold on

plot(uv_d(1,:),uv_d(2,:),'x','Color',[223,67,85]/255,'linewidth',1); hold on
plot(uv_d(3,:),uv_d(4,:),'x','Color',[56,171,217]/255,'linewidth',1); hold on
plot(uv_d(5,:),uv_d(6,:),'x','Color',[46,188,89]/255,'linewidth',1); hold on
plot(uv_d(7,:),uv_d(8,:),'x','Color',[56,10,217]/255,'linewidth',1); hold on
grid on;
legend({'$P_1$','$P_2$','$P_3$','$P_4$','$P_{1d}$','$P_{2d}$','$P_{3d}$','$P_{4d}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Evolucion Pixels}$','Interpreter','latex','FontSize',9);
ylabel('$y$','Interpreter','latex','FontSize',9);
xlabel('$x$','Interpreter','latex','FontSize',9);
xlim([-u0 u0])
ylim([-v0 v0])
set(gca,'Ydir','reverse')
print -dpng pixel
print -depsc pixel

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(Pe)),Pe,'linewidth',1); hold on;
grid on;
legend({'$\tilde{P}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Evolution of Control Errors}$','Interpreter','latex','FontSize',9);
ylabel('$[pixels]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);
print -dpng error
print -depsc error