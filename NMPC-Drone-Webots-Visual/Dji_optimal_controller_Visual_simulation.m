%XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
%XXXXXXXXXXXXXXXXXXX TRAJECTORY CONTROL DJI DRONE XXXXXXXXXXXXXXXXXXXXXXXXX
%XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

%% CLEAN VARIABLES
clc,clear all,close all;

%% DEFINITION OF TIME VARIABLES
ts = 0.1;
tf = 50;
to = 0;
t = (to:ts:tf);

%% CONSTANTS VALUES OF THE ROBOT
a = 0.0; 
b = 0.0;
c = 0.0;
L = [a, b, c];

%% INITIAL CONDITIONS
x = 0.0;
y = 0.0;
z = 0.0069;
yaw = 0*(pi/180);

%% DIRECT KINEMATICS
x = x +a*cos(yaw) - b*sin(yaw);
y = y +a*sin(yaw) + b*cos(yaw);
z = z + c;

h = [x;...
     y;...
     z;...
     yaw];

%% Definicion de los parametros de la camara
%% Defincion de la distancia focal del sistema
f = 0.015;
uo = 512;
vo = 512;
pw = 1e-05;

param = [f; uo; vo; pw];

%% Matrix para la imagen no se considera distorison de lentes
K = [1/pw, 0, uo;...
     0, 1/pw, vo;...
     0, 0,        1];
 
%% COnversion al plano de la imagen 
F = [f, 0, 0, 0;...
     0, f, 0, 0;...
     0, 0, 1, 0];
 

%% Objeto en el espacio 
P = [1, 1.1, 1.1, 1, 1;...
     -0.05, 0.05,0.05,-0.05,-0.05;...   
     0.1, 0.1,0.05,0.05,0.1];
 
%% DEFINITION CAMERA PARAMETERS

p(:,1) = digital_camera(K, F, P, h, param);

%% definicion de los pixeles deseados del sistema
pd = [151.6854, -221.9044, -185.3933, -221.9044, -185.3933, -102.7291, 151.6854, -102.7291, 151.6854, -221.9044]';
pd = pd*ones(1,length(t));

%% Horizonte de prediccion
N = 5;

bounded = [1.5, 1.3, 1.2, 2.5]; 

[v0, UB, LB] = optimization_parameters(N, bounded);

%% Parametros del optimizador
options = optimset('Display','off',...
                'TolFun', 1e-8,...
                'MaxIter', 10000,...
                'Algorithm', 'active-set',...
                'FinDiffType', 'forward',...
                'RelLineSrchBnd', [],...
                'RelLineSrchBndDuration', 1,...
                'TolConSQP', 1e-6);    


% SIMULATION 
for k=1:1:length(t)-N
    tic; 
    %% GENERAL VECTOR OF ERROR SYSTEM
    he(:, k) = pd(:,k)-p(:,k);
    
    %% GENERALIZED CONTROL LAW
    f_obj1 = @(v)  Cost_Function_complete(h(:,k), v, ts, N, L, p(:,k), pd, P, param, k);
    control = fmincon(f_obj1,v0,[],[],[],[],LB,UB,[],options);
    %control = classic_controller(h(:,k), p(:,k), pd(:,k), P, param, L);
    
    %% OBTAIN CONTROL VALUES OF THE VECTOR
    ul(k) = control(1,1);
    um(k) = control(2,1);
    un(k) = control(3,1);
    w(k) = control(4,1);
    
    v = [ul(k); um(k); un(k); w(k)];
   
    %% GET VALUES OF DRONE
    h(:,k+1) = system_drone(h(:,k), v, ts, L);
    p(:,k+1) = system_camera(h(:,k+1), v, p(:,k), P, param, L, ts);
    
    v0 = horizonte(control);
    while(toc<ts)
    end
    toc;
    %% SAVE SAMPLE TIME
    t_sample(k)=toc;
end

%%
close all; paso=1; 
%a) Parámetros del cuadro de animación
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 8 3]);
luz = light;
luz.Color=[0.65,0.65,0.65];
luz.Style = 'infinite';
%b) Dimenciones del Robot
   Drone_Parameters(0.02);
%c) Dibujo del Robot    
    G2=Drone_Plot_3D(h(1,1),h(2,1),h(3,1),0,0,h(4,1));hold on

    plot3(h(1,1),h(2,1),h(3,11),'--','Color',[56,171,217]/255,'linewidth',1.5);hold on,grid on
    plot3(P(1,:),P(2,:),P(3,:),'Color',[100,76,44]/255,'linewidth',1); hold on;

view(20,15);
for k = 1:10:length(t)-N
    drawnow
    delete(G2);
   
    G2=Drone_Plot_3D(h(1,k),h(2,k),h(3,k),0,0,h(4,k));hold on
    
    plot3(h(1,1:k),h(2,1:k),h(3,1:k),'--','Color',[56,171,217]/255,'linewidth',1.5);
    plot3(P(1,:),P(2,:),P(3,:),'Color',[100,76,44]/255,'linewidth',1); hold on;

    legend({'$\mathbf{h}$','$\mathbf{h}_{des}$'},'Interpreter','latex','FontSize',11,'Location','northwest','Orientation','horizontal');
    legend('boxoff')
    title('$\textrm{Movement Executed by the Aerial Robot}$','Interpreter','latex','FontSize',11);
    xlabel('$\textrm{X}[m]$','Interpreter','latex','FontSize',9); ylabel('$\textrm{Y}[m]$','Interpreter','latex','FontSize',9);zlabel('$\textrm{Z}[m]$','Interpreter','latex','FontSize',9);
    
end
print -dpng SIMULATION_1
print -depsc SIMULATION_1

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
set(gca,'Ydir','reverse')

plot(p(1,:),p(2,:),'--','Color',[223,67,85]/255,'linewidth',1); hold on
plot(p(3,:),p(4,:),'--','Color',[56,171,217]/255,'linewidth',1); hold on
plot(p(5,:),p(6,:),'--','Color',[46,188,89]/255,'linewidth',1); hold on
plot(p(7,:),p(8,:),'--','Color',[56,10,217]/255,'linewidth',1); hold on

plot(pd(1,1:length(p)),pd(2,1:length(p)),'x','Color',[223,67,85]/255,'linewidth',1); hold on
plot(pd(3,1:length(p)),pd(4,1:length(p)),'x','Color',[56,171,217]/255,'linewidth',1); hold on
plot(pd(5,1:length(p)),pd(6,1:length(p)),'x','Color',[46,188,89]/255,'linewidth',1); hold on
plot(pd(7,1:length(p)),pd(8,1:length(p)),'x','Color',[56,10,217]/255,'linewidth',1); hold on
grid on;
legend({'$P_1$','$P_2$','$P_3$','$P_4$','$P_{1d}$','$P_{2d}$','$P_{3d}$','$P_{4d}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Evolucion Pixels}$','Interpreter','latex','FontSize',9);
ylabel('$v$','Interpreter','latex','FontSize',9);
xlabel('$u$','Interpreter','latex','FontSize',9);
xlim([-uo uo])
ylim([-vo vo])
set(gca,'Ydir','reverse')
print -dpng pixel
print -depsc pixel

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(he)),he,'linewidth',1); hold on;
grid on;
legend({'$\tilde{P}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Evolution of Control Errors}$','Interpreter','latex','FontSize',9);
ylabel('$[m]$','Interpreter','latex','FontSize',9);
print -dpng error
print -depsc error

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(ul)),ul,'Color',[226,76,44]/255,'linewidth',1); hold on
plot(t(1:length(ul)),um,'Color',[46,188,89]/255,'linewidth',1); hold on
plot(t(1:length(ul)),un,'Color',[26,115,160]/255,'linewidth',1); hold on
plot(t(1:length(ul)),w,'Color',[83,57,217]/255,'linewidth',1); hold on
grid on;
legend({'$\mu_{lc}$','$\mu_{mc}$','$\mu_{nc}$','$\omega_{c}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Control Values}$','Interpreter','latex','FontSize',9);
ylabel('$[rad/s]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(t_sample)),t_sample,'Color',[46,188,89]/255,'linewidth',1); hold on
grid on;
legend({'$t_{sample}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Control Values}$','Interpreter','latex','FontSize',9);
ylabel('$[s]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);  
print -dpng SAMPLE_TIME
print -depsc SAMPLE_TIME


