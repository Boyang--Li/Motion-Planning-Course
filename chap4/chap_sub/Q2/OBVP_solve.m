% Solve the OBVP problem to get the form of optimal time

% Known variables: delta_px, delta_py, delta_pz, delta_vx, delta_vy,
% delta_vz
syms delta_px delta_py delta_pz delta_vx delta_vy delta_vz
syms px0 py0 pz0 vx0 vy0 vz0 pxF pyF pzF
syms alpha1 alpha2 alpha3 beta1 beta2 beta3
syms J f(T)
delta_px = pxF - vx0 * T - px0;
delta_py = pyF - vy0 * T - py0;
delta_pz = pzF - vz0 * T - pz0;
delta_vx = - vx0;
delta_vy = - vy0;
delta_vz = - vz0;

matrix1 = [-12/T^3 0 0 6/T^2 0 0;
    0 -12/T^3 0 0 6/T^2  0;
    0 0 -12/T^3 0 0 6/T^2 ;
    6/T^2 0 0 -2/T 0 0;
    0 6/T^2 0 0 -2/T  0;
    0 0  6/T^2 0 0 -2/T ;];
%
eqn = [alpha1; alpha2; alpha3; beta1; beta2; beta3] == ...
    matrix1 * [delta_px;delta_py;delta_pz;delta_vx;delta_vy;delta_vz];

alpha1 = (12*(px0 - pxF + T*vx0))/T^3 - (6*vx0)/T^2;
alpha2 = (12*(py0 - pyF + T*vy0))/T^3 - (6*vy0)/T^2;
alpha3 = (12*(pz0 - pzF + T*vz0))/T^3 - (6*vz0)/T^2;
beta1 = (2*vx0)/T - (6*(px0 - pxF + T*vx0))/T^2;
beta2 = (2*vy0)/T - (6*(py0 - pyF + T*vy0))/T^2;
beta3 = (2*vz0)/T - (6*(pz0 - pzF + T*vz0))/T^2;

S1 = solve(eqn,alpha1);

J = T + (1/3 * alpha1^2 * T^3 + alpha1 * beta1 * T^2) +...
    (1/3 * alpha2^2 * T^3 + alpha2 * beta2 * T^2) +...
    (1/3 * alpha3^2 * T^3 + alpha3 * beta3 * T^2);

f(T) = T + (T^3*((12*px0 - 12*pxF + 12*T*vx0)/T^3 -...
    (6*vx0)/T^2)^2)/3 + (T^3*((12*py0 - 12*pyF +...
    12*T*vy0)/T^3 - (6*vy0)/T^2)^2)/3 +...
    (T^3*((12*pz0 - 12*pzF + 12*T*vz0)/T^3 -...
    (6*vz0)/T^2)^2)/3 - ...
    T^2*((6*px0 - 6*pxF + 6*T*vx0)/T^2 -...
    (2*vx0)/T)*((12*px0 - 12*pxF + 12*T*vx0)/T^3 -...
    (6*vx0)/T^2) - T^2*((6*py0 - 6*pyF + 6*T*vy0)/T^2 -...
    (2*vy0)/T)*((12*py0 - 12*pyF + 12*T*vy0)/T^3 -...
    (6*vy0)/T^2) - T^2*((6*pz0 - 6*pzF + 6*T*vz0)/T^2 -...
    (2*vz0)/T)*((12*pz0 - 12*pzF + 12*T*vz0)/T^3 -...
    (6*vz0)/T^2);

df = diff(f,T);
eqn = diff(f,T)==0;
S = solve(eqn,T);
