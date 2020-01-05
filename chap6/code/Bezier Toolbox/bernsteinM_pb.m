function Bp=bernsteinM_pb(p)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function returns the bernstein matrix for power basis, which can be
% used to contruct Bezier curves. Input is a scalar value
%
% Syntax
% Bp=bernsteinM_pb(p)
%
% where p is the order of the bernstein polynomial
%
% Bezier curve:
% C = Q*Bp*T
%
% in which Q are the control points of size [m,p+1] and in which T is a 
% column vector of length p+1:
% T = [t^0; t^1; ... ; t^p]
% where t is in the interval [0,1]
% note, T can also be a matrix in which the values of t increase of
% increasing columns.
%
% Eindhoven University of Technology
% Robbin van Hoek - okt 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%predifine for speed for most common orders
if p==5
    Bp=[1 -5 10 -10 5 -1;
        0 5 -20 30 -20 5;
        0 0 10 -30 30 -10;
        0 0 0 10 -20 10;
        0 0 0 0 5 -5;
        0 0 0 0 0 1];
elseif p==4
    Bp=[1 -4 6 -4 1;
        0 4 -12 12 -4;
        0 0 6 -12 6;
        0 0 0 4 -4;
        0 0 0 0 1];
elseif p==3
    Bp=[1 -3 3 -1;
        0 3 -6 3;
        0 0 3 -3;
        0 0 0 1];
elseif p==2
    Bp=[1 -2 1;
        0 2 -2;
        0 0 1];
else %for arbitrary high order we can generate the matrix as following:
    %function creates a bernstein matrix for power basis for order p
    B1=fliplr(pascal(p+1,2)).*(-1)^(p-1); %pascal matrix of right order
    Bp=B1.*triu(repmat(abs(B1(:,end)).',p+1,1)).*((-1).^(1:p+1));
end


end

