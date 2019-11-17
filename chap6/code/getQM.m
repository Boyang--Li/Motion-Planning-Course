function [Q, M] = getQM(n_seg, n_order, ts)
    Q = [];
    M = [];
    M_k = getM(n_order);
    for k = 1:n_seg
        %#####################################################
        % STEP 2.1 calculate Q_k of the k-th segment 
        Q_k = [];

        Q = blkdiag(Q, Q_k);
        M = blkdiag(M, M_k);
    end
end