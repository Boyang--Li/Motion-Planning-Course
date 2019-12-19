%% compute the MPC path/traj? 
function log = getMPCresult(K,dt,weight,constraint,p_0,v_0,a_0,Path)
log = [];
i = 1;
for t=dt:dt:(size(Path,1)-K)*dt
    % Construct the prediction matrix
    [Tp, Tv, Ta, Bp, Bv, Ba] = getPredictioinMatrix(K,dt,p_0,v_0,a_0);
    
    % Tracking slot
    P_cmd = Path(i:i+K-1);
    
    % Construct the optimization problem simple
    H = weight(1)*(Tp'*Tp)+weight(2)*(Tv'*Tv)+weight(3)*(Ta'*Ta)+weight(4)*eye(K);
    F = weight(1)*(Bp'*Tp - P_cmd'*Tp) + weight(2)*Bv'*Tv + weight(3)*Ba'*Ta;
    
    % hard constrains States x|Outpot u
    A = [Tv; -Tv; Ta; -Ta; eye(K); -eye(K)];
    b = [constraint(1)*ones(20,1)-Bv; constraint(2)*ones(20,1)+Bv;...
        constraint(3)*ones(20,1)-Ba; constraint(4)*ones(20,1)+Ba;...
        constraint(5)*ones(20,1);constraint(6)*ones(20,1)];
    
%     % Construct the optimization problem soft constraint
%     H = blkdiag(w4*eye(K)+w1*(Tp'*Tp)+w2*(Tv'*Tv)+w3*(Ta'*Ta),w5*eye(K));
%     F = [w1*Bp'*Tp + w2*Bv'*Tv + w3*Ba'*Ta zeros(1,K)];

%     % soft constrains
%     A = [Tv zeros(K);-Tv -eye(K);Ta zeros(K);-Ta zeros(K);zeros(size(Ta)) -eye(K)];
%     b = [ones(20,1)-Bv;ones(20,1)+Bv;ones(20,1)-Ba;ones(20,1)+Ba;zeros(K,1)];
    
    J = quadprog(H,F,A,b);
    
    j = J(1);
    p_0  = p_0  +v_0 *dt + 0.5*a_0 *dt^2 + 1/6*j *dt^3;
    v_0  = v_0  +a_0 *dt + 0.5*j *dt^2;
    a_0  = a_0 +j *dt;
    
    log = [log; t p_0  v_0  a_0  j];
    
    i=i+1;
end