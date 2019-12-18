function [tvec, dtvec, ddtvec] = calc_bpolyvec(t,n_order)

tvec = zeros(1,n_order+1);
dtvec = zeros(1,n_order+1);
ddtvec = zeros(1,n_order+1);

% position
for i=0:n_order
    tvec(i+1) = nchoosek(n_order,i) * t^i * (1-t)^(n_order-i);
end

% velocity
for i=0:(n_order-1)
    dtvec(i+1) = n_order * (tvec(i+2) - tvec(i+1))* t^i * (1-t)^(n_order-i);
end

% acc
for i=0:n_order-2
    ddtvec(i+1) = n_order * (dtvec(i+2) - dtvec(i+1))* t^i * (1-t)^(n_order-i);
end

end
