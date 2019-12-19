% QP-based MPC to track conical spiral for a 3-axis triple integrator
clc;clear;close all;

% Iinitial Conditions
p_0 = [8 0 20];
v_0 = [0 0 0];
a_0 = [0 0 0];

% Prediction horizon = K*dt; Control horizon = 1
dt = 0.1;
K = 20;

% weight for pos vel acc jerk
weight = [10;1;1;1];

% Constraints 
% |v_xy|<=6, |a_xy|<=3, |j_xy|<=3, -1<=v_z<=6, -1<=a_z<=3, -2<=j_z<=2
constraintXY = [6 6 3 3 3 3];
constraintZ = [6 1 3 1 2 2];

% the spral path for tracking
% W = 0.08 rad/s, V = -0.5m/s, H = 20m; r = 10m
Path = [];
W = 0.4;

% horizontal movement of 1m/s
for t = 0:dt:8-dt
    p_t = [-t 0 0];
    Path = [Path; p_0 + p_t];
end

for t = 8:dt:50
    r = 0.25 * (t-8);
    p_t = [r*sin(W*t) r*cos(W*t) -0.5*(t-8)];
    Path = [Path; [0 0 20] + p_t];
end

% MPC solver
logX = getMPCresult(K,dt,weight,constraintXY,p_0(1),v_0(1),a_0(1),Path(:,1));
logY = getMPCresult(K,dt,weight,constraintXY,p_0(2),v_0(2),a_0(2),Path(:,2));
logZ = getMPCresult(K,dt,weight,constraintZ,p_0(3),v_0(3),a_0(3),Path(:,3));

% plot results
figure
plot(logX(:,1),logX(:,2),logY(:,1),logY(:,2),logZ(:,1),logZ(:,2));
legend('posX', 'velX', 'accX')

figure
plot(logX(2:end,1),logX(2:end,5),logY(2:end,1),logY(2:end,5),logZ(2:end,1),logZ(2:end,5));
legend('jerkX', 'jerkY', 'jerkZ')

figure
scatter3(Path(1:size(logX,1),1),Path(1:size(logY,1),2),Path(1:size(logZ,1),3));
hold on
plot3(logX(:,2),logY(:,2),logZ(:,2));
legend('reference', 'tracking')

figure
plot( logZ(:,1),Path(1:size(logZ,1),3),logZ(:,1),logZ(:,2));
legend('reference_Z', 'tracking_Z')
