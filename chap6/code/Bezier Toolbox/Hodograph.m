function OUT = Hodograph(IN)
% This function will calculate the hodograph of the bezier curve
% Inputs can be either a piecewise bezier spline, shaped as a struct, or a
% single numerical array representing the control points of a single bezier
% curve.
% Outputs will correspond to given inputs, if the input is a piecewise
% bezier spline, output will a struct containing the control points for the
% hodograph of each piece. If input is a numerical array representing the
% control points for a single bezier curve, the output is the hodograph of
% the input curve.
%
% Syntax:
% Bezd = Hodograph(Bez);
% Qd   = Hodograph(Q);
%
% Q  are control points of bezier curve
% Qd are control points of hodograph of bezier curve
%
% Eindhoven University of Technology
% Robbin van Hoek - okt 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%check which mode is used
if isstruct(IN)
    Bez=IN;
    structmode=1;
else
    structmode=0;
    Q=IN;
end

%is structmode is used, loop over each segment.
if structmode
    BezD(numel(Bez))=struct;
    for ii=1:numel(Bez)
        BezD(ii).Q = Hodograph(Bez(ii).Q);
    end
else
    
    %dimensions input bezier curve
    [m,N] = size(Q);
    n=N-1; %n=size(Q,2)-1; %degree of bezier
    
    %compute hodograph
    Qd=zeros(m,n);
    for kk=0:n-1
        idx=kk+1;
        Qd(:,idx) = n*(Q(:,idx+1)-Q(:,idx));
    end
end

%set outputs for corresponding mode
if structmode
    OUT=BezD;
else
    OUT=Qd;
end

end

