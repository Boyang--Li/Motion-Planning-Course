%% General solve
syms pf p0 v0 a0
syms f(T)

f(T) = (20/T^6) * (pf-p0-v0*T-0.5*a0*T^2)^2;
eqn = diff(f,T)==0;
S = solve(eqn,T);

%% Example with certain value
% pf = 1;
% p0 = 0;
% v0 = 1;
% a0 = 1;
% syms f(T)
% 
% f(T) = (20/T^6) * (pf-p0-v0*T-0.5*a0*T^2)^2;
% eqn = diff(f,T)==0;
% S = solve(eqn,T);
% 
% T = 0.01:0.01:5;
% y = subs(f);
% plot(y,T);