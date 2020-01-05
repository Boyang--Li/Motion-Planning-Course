function Bez = EvaluateBezierStruct(Bez,nt)
% This function populates the struct of a piecewise bezier spline. The
% input struct Bez, should contain a field Q, which gives the control
% points for each segment of the bezier curve. nt will give the number of
% linearly spaced samples between 0 and 1, over which to evaluate the curve
%
% The following fields will then be added.
% X the coordinates of the generated curve
% theta the heading of the generated curve
% C the curvature   of the generated curve
% L the total length of the generated curve
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5


for ii=1:numel(Bez)
    
    Q=Bez(ii).Q;
    if isfield(Bez,'w')
        w = Bez(ii).w;
    else
        w = ones(1,size(Bez(1).Q,2));
    end
    
    %define 'time' vectors for each segment
    t=linspace(0,1,nt);
    [X,theta,curvature,L]     = BezierEval(Q,t,w);
    
    Bez(ii).n=t;
    Bez(ii).X=X;  %curve values for segment
    Bez(ii).theta=theta;
    Bez(ii).C=curvature;
    Bez(ii).L=L;
    if ii==1
        Bez(ii).Lt=L;
    else
        Bez(ii).Lt= L + Bez(ii-1).Lt(end);
    end
end


end