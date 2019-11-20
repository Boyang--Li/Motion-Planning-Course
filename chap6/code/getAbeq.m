function [Aeq, beq] = getAbeq(n_seg, n_order, ts, start_cond, end_cond)
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % STEP 2.1 p,v,a constraint in start 
    Aeq_start = [calc_tvec(0,n_order);
    n_order * calc_tvec(0,n_order );
    calc_tvec(0,n_order)];
    beq_start = start_cond';
    
    %#####################################################
    % STEP 2.2 p,v,a constraint in end
    Aeq_end = [];
    beq_end = end_cond';
    
    %#####################################################
    % STEP 2.3 position continuity constrain between 2 segments
    Aeq_con_p = [];
    beq_con_p = [];

    %#####################################################
    % STEP 2.4 velocity continuity constrain between 2 segments
    Aeq_con_v = [];
    beq_con_v = [];

    %#####################################################
    % STEP 2.5 acceleration continuity constrain between 2 segments
    Aeq_con_a = [];
    beq_con_a = [];
    
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
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a];
    beq_con = [beq_con_p; beq_con_v; beq_con_a];
    Aeq = [Aeq_start; Aeq_end; Aeq_con];
    beq = [beq_start; beq_end; beq_con];
end