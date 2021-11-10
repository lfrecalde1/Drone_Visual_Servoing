%% PROGRAM FOR ESTIMATE FOCAL DISTANCE %%
clc, clear all, close all;


%% DEFINITION OF TIME VARIABLES
ts = 0.1;
tf = 100;
to = 0;
t = (to:ts:tf);

%% Initiaa valur of f
f0 = [523;523];
LB = [0;0];
UB = [10000; 10000];
%% Read values of pixels
load('imagen_data.mat');

load('object_camera.mat');

%% DEFINITION OF OPTIMIZATION PARAMETERS
options = optimset('Display','off',...
                'TolFun', 1e-8,...
                'MaxIter', 10000,...
                'Algorithm', 'active-set',...
                'FinDiffType', 'forward',...
                'RelLineSrchBnd', [],...
                'RelLineSrchBndDuration', 1,...
                'TolConSQP', 1e-6);    
f_ob = @(f) camera_model_cost(f, uv(:,1), obj(:,1));
[x,fval,exit,salida] = fmincon(f_ob,f0,[],[],[],[],LB,UB,[],options);


%% Error using diferents values
% error1 = camera_model_cost(f0, uv, obj);
error1 = uv(:,10) - camera_model(f0, obj(:,10));
norm1 = norm(error1,2)
% error2 = camera_model_cost(x, uv ,obj);
error2 = uv(:,10) - camera_model(x, obj(:,10));
norm2 = norm(error2,2)