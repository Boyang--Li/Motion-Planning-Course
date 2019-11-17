function Ct = getCt(n_seg, n_order)
    %#####################################################
    % STEP 2.1: finish the expression of Ct

    A = ones(1,4);
    C0 = diag(A);
    Ck = diag(A);
    
    C_element1 = [1;0;0;0;1;0;0;0];
    for i = 1:(n_seg-1)
        Ci1 = blkdiag(Ci1,C_element1);
    end
    
    A2 = [1;1;1];
    C_element2 = [[0,0,0];diag(A2);[0,0,0];diag(A2)];
    for i = 1:(n_seg-1)
        Ci2 = blkdiag(Ci2,C_element2);
    end
    Ci2 = [zeros(4,3*(n_seg-1));Ci2;zeros(4,3*(n_seg-1))];
    
    Ct = [blkdiag(C0,Ci1,Ck),Ci2];
end