function [Q, M] = getQM(n_seg, n_order, ts)
Q = [];
M = [];
d_order = 4;
M_j = getM(n_order);

for j=0:n_seg-1
    Q_j = zeros(n_order+1);
    for i=d_order:n_order
        for l=d_order:n_order
            if i<=l
                Q_j(i+1, l+1) = factorial(i)/factorial(i-4)*...
                    factorial(l)/factorial(l-4)*...
                    ts(j+1)^(3-2*4)/(i+l-7);
            else
                Q_j(i+1,l+1)=Q_j(l+1,i+1);
            end
        end
    end    
    Q = blkdiag(Q, Q_j);
    M = blkdiag(M, M_j);
end

% second method
% for j=0:n_seg-1
%     Q_j = zeros(n_order+1);
%     for i=d_order:n_order
%         for l=i:n_order
%             Q_j(i+1, l+1) = factorial(i)/factorial(i-4)*...
%                     factorial(l)/factorial(l-4)*...
%                     ts(j+1)^(3-2*4)/(i+l-7);
%             if i~=l
%                 Q_j(l+1,i+1)=Q_j(i+1,l+1);
%             end
%         end
%     end    
%     Q = blkdiag(Q, Q_j);
%     M = blkdiag(M, M_j);
% end

% for k = 1:n_seg
%     Q_k = zeros(n_order+1);
%     %#####################################################
%     % STEP 1.1: calculate Q_k of the k-th segment
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     for i = 4:n_order+1
%         for j = 4:n_order+1
%             Q_k(n_order+2-i,n_order+2-j) = (i*(i-1)*(i-2)*(i-3)*j*(j-1)*(j-2)*(j-3)/(i+j-7))*ts(n_seg)^(i+j-7);
%         end
%     end
%     Q = blkdiag(Q, Q_k);
% end