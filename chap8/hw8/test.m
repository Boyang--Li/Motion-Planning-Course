% single axis mpc regulator
clear

% initial condition
p_0 = 10;
v_0 = 0;
a_0 = 0;
K = 20;
dt = 0.2;
log = [0 p_0 v_0 a_0];
w1 = 1;
w2 = 1;
w3 = 1;
w4 = 1;
w5 = 100;

for t=dt:dt:10
    % Construct the prediction matrix
    [Tp, Tv, Ta, Bp, Bv, Ba] = getPredictioinMatrix(K,dt,p_0,v_0,a_0);

    % Construct the optimization problem simple
    H = w4*eye(K)+w1*(Tp'*Tp)+w2*(Tv'*Tv)+w3*(Ta'*Ta);
    F = w1*Bp'*Tp + w2*Bv'*Tv + w3*Ba'*Ta;
    
    % hard constrains
    A = [Tv; -Tv; Ta; -Ta];
    b = [ones(20,1)-Bv; ones(20,1)+Bv; ones(20,1)-Ba; ones(20,1)+Ba];
    
%     % Construct the optimization problem soft constraint
%     H = blkdiag(w4*eye(K)+w1*(Tp'*Tp)+w2*(Tv'*Tv)+w3*(Ta'*Ta),w5*eye(K));
%     F = [w1*Bp'*Tp + w2*Bv'*Tv + w3*Ba'*Ta zeros(1,K)];

%     % soft constrains
%     A = [Tv zeros(K);-Tv -eye(K);Ta zeros(K);-Ta zeros(K);zeros(size(Ta)) -eye(K)];
%     b = [ones(20,1)-Bv;ones(20,1)+Bv;ones(20,1)-Ba;ones(20,1)+Ba;zeros(K,1)];
    
    J = quadprog(H,F,A,b);
    
    j = J(1);
    p_0 = p_0 +v_0*dt + 0.5*a_0*dt^2 + 1/6*j*dt^3;
    v_0 = v_0 +a_0*dt + 0.5*j*dt^2;
    a_0 = a_0+j*dt;
    
    log = [log; t p_0 v_0 a_0];
end

plot(log(:,1),log(:,2), log(:,1),log(:,3), log(:,1),log(:,4));
legend('pos', 'vel', 'acc')
% second method
J2 = -H\F';