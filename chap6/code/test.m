%% Use the matlab robotics toolbox to generate B-spine path
cpts = [50 100 180 250 280; 50 120 150 80 0];
tpts = [0 1];
tvec = 0:0.01:1;
[q, qd, qdd, pp] = bsplinepolytraj(cpts,tpts,tvec);


%% direct plot
x_pos = [];
y_pos = [];
idx = 1;
n_order = size(cpts,2)-1;
for t=0:0.01:1
    x_pos(idx) = 0;
    y_pos(idx) = 0;
    for i = 0:n_order
        basis_p = nchoosek(n_order, i) * t^i * (1-t)^(n_order-i);
        x_pos(idx) = x_pos(idx) + cpts(1,i+1) * basis_p;
        y_pos(idx) = y_pos(idx) + cpts(2,i+1) * basis_p;
    end
    idx = idx + 1;
end

scatter(cpts(1,:),cpts(2,:),'DisplayName','comtrol points');
hold on
plot(x_pos,y_pos,'b--o','DisplayName','direct');
plot(q(1,:),q(2,:),'DisplayName','matlab function');
legend;
hold off
