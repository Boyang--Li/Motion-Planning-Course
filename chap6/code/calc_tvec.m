% get the Bernstein polynomial parameters
function tvec = calc_tvec(t,n_order)
n = n_order + 1;
tvec = zeros(1,n);
for i=1:n
    v = i-1;
    tvec(i) = nchoosek(n_order,v) * t^v * (1-t)^(n_order-v);
end
end

