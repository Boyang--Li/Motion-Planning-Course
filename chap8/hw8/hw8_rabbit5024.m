% QP-based MPC to track conical spiral for a 3-axis triple integrator

% Iinitial Conditions
% p_0 = [0; 0; 20], W = 0.08 rad/s, V = -0.5m/s, H = 20m; r = 10m
p_0 = [0 0 20];
P = [0 0 20];
W = 0.08;
delta_t = 0.1;
% Constraints 
% |v_xy|<=6, |a_xy|<=3, |j_xy|<=3, -1<=v_z<=6, -1<=a_z<=3, -2<=j_z<=2

%% plot the spral path
for t = delta_t:delta_t:40
    r = 0.25 * t;
    p_t = [r*sin(W*t) r*cos(W*t) -0.5*t];
    P = [P; p_0 + p_t];
end

scatter3(P(:,1),P(:,2),P(:,3));