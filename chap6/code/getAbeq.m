function [Aeq, beq] = getAbeq(n_seg, n_order, ts, start_cond, end_cond)
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % STEP 2.1 p,v,a constraint in start 
    Aeq_start = [];
    beq_start = [];
    
    %#####################################################
    % STEP 2.2 p,v,a constraint in end
    Aeq_end = [];
    beq_end = [];
    
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

    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a];
    beq_con = [beq_con_p; beq_con_v; beq_con_a];
    Aeq = [Aeq_start; Aeq_end; Aeq_con];
    beq = [beq_start; beq_end; beq_con];
end