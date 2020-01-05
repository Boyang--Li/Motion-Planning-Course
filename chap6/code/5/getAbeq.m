function [Aeq beq]= getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond)
n_all_poly = n_seg*(n_order+1);
n_coef = n_order+1;
%#####################################################
% p,v,a,j constraint in start,
Aeq_start = zeros(4, n_all_poly);
beq_start = zeros(4, 1);

% STEP 2.1: write expression of Aeq_start and beq_start
% for k = 0:3
%     for i = 0:(n_order)
%         if (i>=k)
%             Aeq_start(k+1,i+1) = factorial(i)/factorial(i-k)*0^(i-k);
%         end
%     end
% end

Aeq_start(:,1:n_coef) = [calc_tvec(0,n_order,0);
    calc_tvec(0,n_order,1);
    calc_tvec(0,n_order,2);
    calc_tvec(0,n_order,3)];

beq_start = start_cond';

%#####################################################
% p,v,a constraint in end
Aeq_end = zeros(4, n_all_poly);
beq_end = zeros(4, 1);

% STEP 2.2: write expression of Aeq_end and beq_end
% tmp = zeros(4, (n_order+1));
% for k = 0:3
%     for i = 0:(n_order)
%         if (i>=k)
%             tmp(k+1,i+1) = factorial(i)/factorial(i-k)*ts(1)^(i-k);
%         end
%     end
% end
% Aeq_end (:,(n_all_poly-n_order):end) = tmp;

Aeq_end(:,n_coef*(n_seg-1)+1:n_coef*n_seg) = ...
    [calc_tvec(ts(end),n_order,0);
    calc_tvec(ts(end),n_order,1);
    calc_tvec(ts(end),n_order,2);
    calc_tvec(ts(end),n_order,3)];
beq_end = end_cond';

%#####################################################
% position constrain in all middle waypoints
Aeq_wp = zeros(n_seg-1, n_all_poly);
beq_wp = zeros(n_seg-1, 1);

% STEP 2.3: write expression of Aeq_wp and beq_wp
for i=1:n_seg-1
    Aeq_wp(i,n_coef*i+1:n_coef*(i+1)) = calc_tvec(0,n_order,0);
    beq_wp(i) = waypoints(i+1);
end

%#####################################################
% position continuity constrain between each 2 segments
Aeq_con_p = zeros(n_seg-1, n_all_poly);
beq_con_p = zeros(n_seg-1, 1);

% STEP 2.4: write expression of Aeq_con_p and beq_con_p


%#####################################################
% velocity continuity constrain between each 2 segments
Aeq_con_v = zeros(n_seg-1, n_all_poly);
beq_con_v = zeros(n_seg-1, 1);
% STEP 2.5: write expression of Aeq_con_v and beq_con_v


%#####################################################
% acceleration continuity constrain between each 2 segments
Aeq_con_a = zeros(n_seg-1, n_all_poly);
beq_con_a = zeros(n_seg-1, 1);
% STEP 2.6: write expression of Aeq_con_a and beq_con_a


%#####################################################
% jerk continuity constrain between each 2 segments
Aeq_con_j = zeros(n_seg-1, n_all_poly);
beq_con_j = zeros(n_seg-1, 1);
% STEP 2.7: write expression of Aeq_con_j and beq_con_j

for i=1:n_seg-1
    tvec_p1 = calc_tvec(ts(i),n_order,0);
    tvec_p2 = calc_tvec(0,n_order,0);
    tvec_v1 = calc_tvec(ts(i),n_order,1);
    tvec_v2 = calc_tvec(0,n_order,1);
    tvec_a1 = calc_tvec(ts(i),n_order,2);
    tvec_a2 = calc_tvec(0,n_order,2);
    tvec_j1 = calc_tvec(ts(i),n_order,3);
    tvec_j2 = calc_tvec(0,n_order,3);
    Aeq_con_p(i,n_coef*(i-1)+1:n_coef*(i+1))=[tvec_p1,-tvec_p2];
    Aeq_con_v(i,n_coef*(i-1)+1:n_coef*(i+1))=[tvec_v1,-tvec_v2];
    Aeq_con_a(i,n_coef*(i-1)+1:n_coef*(i+1))=[tvec_a1,-tvec_a2];
    Aeq_con_j(i,n_coef*(i-1)+1:n_coef*(i+1))=[tvec_j1,-tvec_j2];
end

%#####################################################
% combine all components to form Aeq and beq
Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a; Aeq_con_j];
beq_con = [beq_con_p; beq_con_v; beq_con_a; beq_con_j];
Aeq = [Aeq_start; Aeq_end; Aeq_wp; Aeq_con];
beq = [beq_start; beq_end; beq_wp; beq_con];
end