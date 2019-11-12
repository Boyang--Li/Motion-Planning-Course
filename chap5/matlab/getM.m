function M = getM(n_seg, n_order, ts)
M = [];
for k = 1:n_seg
    M_k = zeros(n_order+1);
    %#####################################################
    % STEP 1.1: calculate M_k of the k-th segment

    for i = 1:(n_order+1)/2
        M_k(i,:) = flip(calc_tvec(0,n_order,i-1));
    end
    
    for i = (n_order+1)/2+1:(n_order+1)
        M_k(i,:) = flip(calc_tvec(ts(k),n_order,i-(n_order+1)/2-1));
    end
    
    M = blkdiag(M, M_k);
end
end