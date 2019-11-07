function [Aeq beq]= getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond)
n_all_poly = n_seg*(n_order+1);
%#####################################################
% p,v,a,j constraint in start,
Aeq_start = zeros(4, n_all_poly);
beq_start = zeros(4, 1);

% STEP 2.1: write expression of Aeq_start and beq_start
for i = 4:n_order
    for j = 1:n_order
        Aeq_start(i-3,j) = factorial(i)/factorial(i-4)*ts^(i-4);
    end
end
beq_start = start_cond;

%#####################################################
% p,v,a constraint in end
Aeq_end = zeros(4, n_all_poly);
beq_end = zeros(4, 1);

% STEP 2.2: write expression of Aeq_end and beq_end
for i = 4:n_order
    for j = 1:n_order
        Aeq_end(i-3,(i-4)*n_order+j) = factorial(i)/factorial(i-4)*ts^(i-4);
    end
end
beq_end = start_cond;

%#####################################################
% position constrain in all middle waypoints
Aeq_wp = zeros(n_seg-1, n_all_poly);
beq_wp = zeros(n_seg-1, 1);

% STEP 2.3: write expression of Aeq_wp and beq_wp
for i = 1:n_seg-1
    for j = 1:n_order
        Aeq_wp(i,(i-1)*n_order+j) = factorial(i)/factorial(i-4)*ts^(i-4);
    end
end

for i = 1:n_seg-1
    beq_wp(i) = waypoints(i+1);
end

%#####################################################
% position continuity constrain between each 2 segments
Aeq_con_p = zeros(n_seg-1, n_all_poly);
beq_con_p = zeros(n_seg-1, 1);

% STEP 2.4: write expression of Aeq_con_p and beq_con_p
for i = 1:n_seg-1
    for j = 1:n_order
        Aeq_con_p(i,(i-1)*n_order+j) = factorial(i)/factorial(i-j)*ts^(i-j);
    end
end


%#####################################################
% velocity continuity constrain between each 2 segments
Aeq_con_v = zeros(n_seg-1, n_all_poly);
beq_con_v = zeros(n_seg-1, 1);
% STEP 2.5: write expression of Aeq_con_v and beq_con_v
%
%
%
%

%#####################################################
% acceleration continuity constrain between each 2 segments
Aeq_con_a = zeros(n_seg-1, n_all_poly);
beq_con_a = zeros(n_seg-1, 1);
% STEP 2.6: write expression of Aeq_con_a and beq_con_a
%
%
%
%

%#####################################################
% jerk continuity constrain between each 2 segments
Aeq_con_j = zeros(n_seg-1, n_all_poly);
beq_con_j = zeros(n_seg-1, 1);
% STEP 2.7: write expression of Aeq_con_j and beq_con_j
%
%
%
%

%#####################################################
% combine all components to form Aeq and beq
Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a; Aeq_con_j];
beq_con = [beq_con_p; beq_con_v; beq_con_a; beq_con_j];
Aeq = [Aeq_start; Aeq_end; Aeq_wp; Aeq_con];
beq = [beq_start; beq_end; beq_wp; beq_con];
end