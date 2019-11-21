function [b, db, ddb] = calc_bpolyvec(x,n_order)
syms t tvec
n = n_order + 1;
% tvec = zeros(1,n);
for i=1:n
    v = i-1;
    tvec(i) = nchoosek(n_order,v) * t^v * (1-t)^(n_order-v);
    dtvec(i) = diff(tvec(i),t,1);
    ddtvec(i) = diff(tvec(i),t,2);
end
% t = x;
b = subs(tvec,x);
db = subs(dtvec,x);
ddb = subs(ddtvec,x);
end
