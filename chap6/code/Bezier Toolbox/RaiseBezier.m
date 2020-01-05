function OUT = RaiseBezier(IN,varargin)
% This function will raise the order of the bezier curve, while maintaining
% its shape. Inputs can be either a piecewise bezier spline, shaped as a struct, or a
% single numerical array representing the control points of a single bezier
% curve.
% Outputs will correspond to given inputs, if the input is a piecewise
% bezier spline, output will also be a struct. If input is a numerical
% array with control points, the output is also a numerical array with
% control points of one order higher.
% Optionally a second argument can be provided to the function if the order
% should be raised by more then one.
%
% Syntax:
% Bezr = RaiseBezier(Bez)
% Bezr = RaiseBezier(Bez,r)
% Qr   = RaiseBezier(Q)
% Qr   = RaiseBezier(Q,r)
%
% Eindhoven University of Technology
% Robbin van Hoek - okt 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if isstruct(IN)
    Bez=IN;
    structmode=1;
else
    Q_in=IN;
    structmode=0;
end

%check how many orders need raising
if nargin>1
    raise=varargin{1};
else
    raise=1;
end

%is structmode is used, loop over each segment.
if structmode
    for ii=1:numel(Bez)
        Bez(ii).Q = RaiseBezier(Bez(ii).Q,raise);
    end
    %set output for structmode
    OUT=Bez;
else
    
    for rr=1:raise
        %raising algorithm
        [m,k]     =size(Q_in); %amount of control points (=order+1)
        Q_out     = zeros(m,k+1);
        for ii=0:k
            ind=ii+1;
            if ii==0
                wprev=zeros(size(Q_in(:,1)));
                wcurr=Q_in(:,ind);
            elseif ii==k
                wprev=Q_in(:,ind-1);
                wcurr=zeros(size(Q_in(:,1)));
            else
                wprev=Q_in(:,ind-1);
                wcurr=Q_in(:,ind);
            end
            Q_out(:,ind) = 1/k*((k-ii)*(wcurr)  + ii*(wprev) );
        end
        
        %update input for raising
        Q_in=Q_out;
    end
    %set output for single curve mode
    OUT=Q_out;
end

end