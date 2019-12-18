function [Aeq, beq] = getAbeq(n_seg, n_order, ts, start_cond, end_cond)
n_all_poly = n_seg*(n_order+1);
n_coef = n_order + 1;

[b_start, db_start, ddb_start] = calc_bpolyvec(0,n_order);
[b_end, db_end, ddb_end] = calc_bpolyvec(1,n_order);

%#####################################################
% STEP 2.1 p,v,a constraint in start
Aeq_start = zeros(3, n_all_poly);
Aeq_start(:,1:n_coef) = [b_start; db_start; ddb_start];
beq_start = start_cond';

%#####################################################
% STEP 2.2 p,v,a constraint in end
Aeq_end = zeros(3, n_all_poly);
Aeq_end(:,1:n_coef) = [b_end; db_end; ddb_end];
beq_end = end_cond';

%#####################################################
% STEP 2.3 position continuity constrain between 2 segments
Aeq_con_p = zeros(n_seg-1, n_all_poly);
beq_con_p = zeros(n_seg-1, 1);

%#####################################################
% STEP 2.4 velocity continuity constrain between 2 segments
Aeq_con_v = zeros(n_seg-1, n_all_poly);
beq_con_v = zeros(n_seg-1, 1);

%#####################################################
% STEP 2.5 acceleration continuity constrain between 2 segments
Aeq_con_a = zeros(n_seg-1, n_all_poly);
beq_con_a = zeros(n_seg-1, 1);

for i=1:n_seg-1
    Aeq_con_p(i,n_coef*(i-1)+1:n_coef*(i+1))=[b_end,-b_start];
    Aeq_con_v(i,n_coef*(i-1)+1:n_coef*(i+1))=[db_end,-db_start];
    Aeq_con_a(i,n_coef*(i-1)+1:n_coef*(i+1))=[ddb_end,-ddb_start];
end

%#####################################################
% combine all components to form Aeq and beq
Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a];
beq_con = [beq_con_p; beq_con_v; beq_con_a];
Aeq = [Aeq_start; Aeq_end; Aeq_con];
beq = [beq_start; beq_end; beq_con];
end