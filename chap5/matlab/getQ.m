function Q = getQ(n_seg, n_order, ts)
Q = [];
for k = 1:n_seg
    Q_k = zeros(n_order);
    %#####################################################
    % STEP 1.1: calculate Q_k of the k-th segment
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for i = 4:n_order
        for j = 4:n_order
            Q_k(i,j) = (i*(i-1)*(i-2)*(i-3)*j*(j-1)*(j-2)*(j-3)/(i+j-7))*ts^(i+j-7);
        end
    end
    Q = blkdiag(Q, Q_k);
end
end