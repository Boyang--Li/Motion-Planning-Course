function coef_list = YHTrangle(n)
coef_list = zeros(1,n);
for i=1:n
    coef_list(i) = (-1)^(i+n) * nchoosek(n-1,i-1);
end
end
