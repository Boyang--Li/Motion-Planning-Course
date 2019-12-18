function [Aieq, bieq] = getAbieq(n_seg, n_order, corridor_range, ts, v_max, a_max)
n_all_poly = n_seg*(n_order+1);
n_coef = n_order+1;

[b_start, db_start, ddb_start] = calc_bpolyvec(0,n_order);
[b_end, db_end, ddb_end] = calc_bpolyvec(1,n_order);
%#####################################################
% STEP 3.2.1 p constraint
Aieq_p = zeros(n_seg-1, n_all_poly);
bieq_p_min = corridor_range(2:end, 1);
bieq_p_max = corridor_range(2:end, 2);

%#####################################################
% STEP 3.2.2 v constraint
Aieq_v = zeros(n_seg-1, n_all_poly);
bieq_v = v_max * ones(n_seg-1, 1);

%#####################################################
% STEP 3.2.3 a constraint
Aieq_a = zeros(n_seg-1, n_all_poly);
bieq_a = zeros(n_seg-1, 1);


for i=1:n_seg-1
    Aieq_p(i,n_coef*(i-1)+1:n_coef*(i))=b_end;
    Aieq_v(i,n_coef*(i-1)+1:n_coef*(i))=db_end;
    Aieq_a(i,n_coef*(i-1)+1:n_coef*(i))=ddb_end;
end

%#####################################################
% combine all components to form Aieq and bieq
Aieq_p = [-Aieq_p;Aieq_p];
Aieq_v = [-Aieq_v;Aieq_v];
Aieq_a = [-Aieq_a;Aieq_a];
bieq_p = [-bieq_p_min;bieq_p_max];
bieq_v = [-bieq_v;bieq_v];
bieq_a = [-bieq_a;bieq_a];

Aieq = [Aieq_p; Aieq_v; Aieq_a];
bieq = [bieq_p; bieq_v; bieq_a];

end