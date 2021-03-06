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
a = 0.1; 
b = 0.1;
c = 0.0;
L = [a, b, c];

%% INITIAL CONDITIONS
x = 0.0;
y = 0.0;
z = 0;
yaw = 0*(pi/180);

%% DIRECT KINEMATICS
x = x +a*cos(yaw) - b*sin(yaw);
y = y +a*sin(yaw) + b*cos(yaw);
z = z + c;

h = [x;...
     y;...
     z;...
     yaw];

%% DESIRED SIGNALS OF THE SYSYEM
hxd = 0.25*t+2;
hxdp = 0.25*ones(1,length(t));

hyd = 2*sin(t/8)+0.05*t-4;
hydp = (1/8)*2*cos(t/8)+0.05;

hzd = 10+1.5*sin(t/10);
hzdp = 1.5*(1/10)*cos(t/10);

hthd = (atan2(hydp,hxdp));


%% GENERALIZED DESIRED SIGNALS
hd = [hxd;...
      hyd;...
      hzd;...
      hthd];
  

%% Horizonte de prediccion
N = 5;

bounded = [1.5, 1.5, 1.5, 2.5]; 

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

%% SIMULATION 
for k=1:1:length(t)-N
    tic; 
    %% GENERAL VECTOR OF ERROR SYSTEM
    he(:, k) = hd(:,k)-h(:,k);
    
    %% GENERALIZED CONTROL LAW
    f_obj1 = @(v)  Cost_Function_drone(h(:,k), hd, v, ts, N, L, k);
    control = fmincon(f_obj1,v0,[],[],[],[],LB,UB,f_obj2,options);
%     control = controller(h(:,k), hd(:,k), hdp(:,k), K1, K2, L);
    
    %% OBTAIN CONTROL VALUES OF THE VECTOR
    ul(k) = control(1);
    um(k) = control(2);
    un(k) = control(3);
    w(k) = control(4);
    
    %% GET VALUES OF DRONE
    h(:,k+1) = system_drone(h(:,k), control(:,1), ts, L);
    v0 = horizonte(control);
    while(toc<ts)
    end
    toc;
end

%%
close all; paso=1; 
%a) Par??metros del cuadro de animaci??n
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
    plot3(hxd(1),hyd(1),hzd(1),'Color',[32,185,29]/255,'linewidth',1.5);

view(20,15);
for k = 1:30:length(t)-N
    drawnow
    delete(G2);
   
    G2=Drone_Plot_3D(h(1,k),h(2,k),h(3,k),0,0,h(4,k));hold on
    
    plot3(hxd(1:k),hyd(1:k),hzd(1:k),'Color',[32,185,29]/255,'linewidth',1.5);
    plot3(h(1,1:k),h(2,1:k),h(3,1:k),'--','Color',[56,171,217]/255,'linewidth',1.5);
    
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
plot(t(1:length(he)),he(1,:),'Color',[226,76,44]/255,'linewidth',1); hold on;
plot(t(1:length(he)),he(2,:),'Color',[46,188,89]/255,'linewidth',1); hold on;
plot(t(1:length(he)),he(3,:),'Color',[26,115,160]/255,'linewidth',1);hold on;
plot(t(1:length(he)),he(4,:),'Color',[83,57,217]/255,'linewidth',1);hold on;
grid on;
legend({'$\tilde{h_{x}}$','$\tilde{h_{y}}$','$\tilde{h_{z}}$','$\tilde{h_{\psi}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Evolution of Control Errors}$','Interpreter','latex','FontSize',9);
ylabel('$[m]$','Interpreter','latex','FontSize',9);

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
