function [C,varargout] =BezierEval(Q,t,varargin)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function evaluates the Bezier curve described by control points Q,
% and returns the coordinates of the curve itself, the angle of each point,
% the curvature, and the total distance on the curve in euclidian distance.
%
%
% Syntax:
%[C,theta,K,L]  = BezierEval(Q,t)
%[C,theta,K]    = BezierEval(Q,t)
%[C,theta]      = BezierEval(Q,t)
%
% Inputs:
% Q(m,n)        m dimenional space, n control points
% t(1,l)        row vector, parametrisation variable
%
%
% Outputs:
% C(m,l)        Bezier C(t)
% K(1,l)        curvature
% L(1,l)        cumulative path length
%
% Eindhoven University of Technology
% Robbin van Hoek - okt 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% checks on inputs

%check if a rational or regular bezier curve is considered.
if nargin>2
    w=varargin{1};
    if all(w==w(1))
        rational = 0; %if all weights are the same, the resulting curve is regular instead of rational
    else
        rational = 1;
    end
else
    rational = 0;
end

%dimension of space in which bezier is defined
m = size(Q,1);   

if rational ==1
    if nargout==1
        [C]             = RBezierEval(Q,t,w);
    elseif nargout==2
        [C,theta]       = RBezierEval(Q,t,w);
        varargout{1}=theta;
    elseif nargout==3
        [C,theta,K]     = RBezierEval(Q,t,w);
        varargout{1}=theta;
        varargout{2}=K;
    else
        [C,theta,K,L]   = RBezierEval(Q,t,w);
        varargout{1}=theta;
        varargout{2}=K;
        varargout{3}=L;
    end
    
else
    
    %% create bernstein basis functions: power basis
    if nargout==1
        [C] = BezierLocal(Q,t);
    elseif nargout==2
        [C,Cd] = BezierDeriv(Q,t);
    elseif nargout>2
        [C,Cd,Cdd] = BezierDeriv(Q,t);
    end
    
    if nargout>1
        dx=real(Cd(1,:));
        dy=real(Cd(2,:));
        theta = atan2(dy,dx);
        
        varargout{1}=theta;
        
        if nargout>2
            colnorm = @(X,pow) sum(abs(X).^pow,1).^(1/pow);
            
            
            if m==2
                crossvector = cross([Cd;zeros(1,numel(t))],[Cdd;zeros(1,numel(t))]);
                K = crossvector(3,:)./(colnorm(Cd,2).^3); %signed curvature in 2D
            elseif m==3
                crossvector = cross(Cd,Cdd);
                K = colnorm(crossvector,2)./(colnorm(Cd,2).^3); %unsigned curvature in 3D
            end
            
            %curvature and path length
            %K=(Cd(1,:).*Cdd(2,:)-Cd(2,:).*Cdd(1,:)) ./ ((Cd(1,:).^2+(Cd(2,:)).^2).^1.5);
            %             crossvector = cross([Cd;zeros(1,numel(t))],[Cdd;zeros(1,numel(t))]);
            %             K = crossvector(3,:)./(colnorm(Cd,2).^3);
            varargout{2}=K;
            if nargout>3
                if numel(t)>1
                    L = cumtrapz(t,colnorm(Cd,2)); %path length
                else
                    L=0;
                end
                varargout{3}=L;
            end
        end
        
    end
    
end


end



function [C,varargout] = BezierDeriv(Q,t)
C=BezierLocal(Q,t);
temp=Q;
varargout{nargout-1}=[]; %preallocation
for ii=1:nargout-1
    temp=HodoGraph(temp); %create weights for derivative function
    varargout{ii} =BezierLocal(temp,t);
end
%varargout{nargout} =BezierLocal(temp,t);
end


function [C] = BezierLocal(Q,t)
p = size(Q,2)-1; %degree of bezier curve

%faster alternative: (only for nonrational bezier curves)
%C = (bernsteinMatrix(p,t)*Q.').';

%alternative if symbolic toolbox is not available (slightly slower)
C = Q*bernsteinM_pb(p)*(t.^([0:p]'));

end

function Qd=HodoGraph(Q)
n=size(Q,2)-1; %degree of bezier
Qd = zeros(size(Q,1),n);
for kk=0:n-1
    idx=kk+1;
    Qd(:,idx) = n*(Q(:,idx+1)-Q(:,idx));
end
end

%% Rational bezier curve evaluation function
function [C,varargout] =RBezierEval(Q,t,w)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function evaluates the rational Bezier curve described by control
% points Q, and weights w and returns the coordinates of the curve itself,
% the angle of each point, the curvature, and the total distance on the
% curve in euclidian distance.
%
% Syntax:
%[C,theta,K,L]  = BezierEval(Q,t,w)
%[C,theta,K]    = BezierEval(Q,t,w)
%[C,theta]      = BezierEval(Q,t,w)
%
% Inputs:
% Q(m,n)        m dimenional space, n control points
% w(1,n)        row vector, weights per control point (optional)
% t(1,l)        row vector, parametrisation variable
%
% Outputs:
% C(m,l)        Bezier C(t)
% K(1,l)        curvature
% L(1,l)        cumulative path length
%
% Eindhoven University of Technology
% Robbin van Hoek - okt 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% checks on inputs

%check if rational or regular bezier curve. if no info, assume regular
p = size(Q,2)-1; %degree of bezier curve

%% create bernstein basis functions: power basis
Bp = bernsteinM_pb(p);

% %create transformation matrices such that kron(Upk,Up) = T2* (t.^([0:p^3]')
% T2 = zeros((p^2+1)*(p+1),p^2*p+1);
% for ii=1:(p^2+1)
%     T2(((ii-1)*(p+1)+1):(((ii-1)*(p+1)+1)+p),  ii:(ii+p)) = eye(p+1);
% end

%first derivative premultiplication
Du  = diag((1:p),-1); %derivative operator (premultiplication) for derivatives

%second derivative premultiplication
Duk = zeros((p+1)^2);
for ii=1:(p+1)
    col = zeros(2,1);
    row = zeros(2,1);
    if ii==1
        col(1) = ii-1 + 1;  %offset by 1 to remove column at index zero
        col(2) = p+ii-1; %offset by 1 to remove column at index zero
        row(1) = 1+(ii-1)*(p+1) + 1;
        row(2) = (p+1)+(ii-1)*(p+1);
        
        mt   = numel((row(1):row(2)));
        block1 = diag((ii:ii+(mt-1)));
        Duk((row(1):row(2)),(col(1):col(2))) = block1 ;
        
    else
        col(1) = ii-1;  %offset by 1 to remove column at index zero
        col(2) = p+ii-1; %offset by 1 to remove column at index zero
        row(1) = 1+(ii-1)*(p+1);
        row(2) = (p+1)+(ii-1)*(p+1);
        
        mt   = numel((row(1):row(2)));
        block2 = diag((ii-1:ii+(mt-2)));
        Duk((row(1):row(2)),(col(1):col(2))) = block2 ;
    end
end

%weights on control points
W  = diag(w);
Wu = ones(1,numel(w))*W*Bp;
Vu = Q*W*Bp;


%evaluate numerically
% Up   = t.^([0:p]');   %time power matrix
% Up2  = t.^([0:p^2]');   %time power matrix
% Up3  = t.^([0:p^3]');   %time power matrix
% Up4  = t.^([0:p^4]');   %time power matrix

%description of curve
Up   = repmat(t,numel((0:p^1)),1).^(repmat((0:p^1)',1,numel(t)));   %time power matrix
C   = (Vu*Up)./repmat((Wu*Up),size(Q,1),1);       %curve

if nargout>1
    %create transformation matrices such that kron(Up,Up) = T1* (t.^([0:p^2]')
    T1 = zeros((p+1)^2,p^2+1);
    for ii=1:(p+1)
        T1(((ii-1)*(p+1)+1):(((ii-1)*(p+1)+1)+p),ii:(ii+p)) = eye(p+1);
    end
    
    %first dervivative coefficients
    Ak = kron(Wu,Vu*Du)-kron(Vu,Wu*Du); %see eq. A.3 of meeting 7
    Bk = kron(Wu,Wu);
    Up2  = repmat(t,numel((0:p^2)),1).^(repmat((0:p^2)',1,numel(t)));   %time power matrix
    
    Cd  = (Ak*T1*Up2)./repmat((Bk*T1*Up2),size(Q,1),1); %first derivatives
end
%Up3  = repmat(t,numel([0:p^3]),1).^(repmat([0:p^3]',1,numel(t)));   %time power matrix
if nargout>2
    %create transformation matrices such that kron(Upk,Upk) = T3* (t.^([0:p^4]')
    T3 = zeros((p+1)^4,p^4+1);
    for ii=1:(p+1)^2
        T3(((ii-1)*(p+1)^2+1):(((ii-1)*(p+1)^2+1)+(p+1)^2-1),ii:(ii+(p+1)^2-1)) = eye((p+1)^2);
    end
    
    %second derivative
    Ak2 = kron(Bk,Ak*Duk)-kron(Ak,Bk*Duk);
    Bk2 = kron(Bk,Bk);
    Up4  = repmat(t,numel((0:p^4)),1).^(repmat((0:p^4)',1,numel(t)));   %time power matrix
    
    Cdd = (Ak2*T3*Up4)./repmat((Bk2*T3*Up4),size(Q,1),1);
end

% C   = (Vu*Up)./(Wu*Up);       %curve
% Cd  = (Ak*T1*Up2)./(Bk*T1*Up2); %first derivatives
% Cdd = (Ak2*T3*Up4)./(Bk2*T3*Up4);

if nargout>1
    dx=real(Cd(1,:));
    dy=real(Cd(2,:));
    theta = atan2(dy,dx);
    varargout{1}=theta;
    if nargout>2
        %curvature and path length
        colnorm = @(X,pow) sum(abs(X).^pow,1).^(1/pow);
        
        %K=(Cd(1,:).*Cdd(2,:)-Cd(2,:).*Cdd(1,:)) ./ ((Cd(1,:).^2+(Cd(2,:)).^2).^1.5);
        m = size(Q,1);   %dimension of space
        if m==2
            crossvector = cross([Cd;zeros(1,numel(t))],[Cdd;zeros(1,numel(t))]);
            K = crossvector(3,:)./(colnorm(Cd,2).^3); %signed curvature in 2D
        elseif m==3
            crossvector = cross(Cd,Cdd);
            K = colnorm(crossvector,2)./(colnorm(Cd,2).^3); %unsigned curvature in 3D
        end
        
        varargout{2}=K;
        if nargout>3
            %L=zeros(size(t));
            if numel(t)>1
                L = cumtrapz(t,colnorm(Cd,2)); %path length
            else
                L=0;
            end
            varargout{3}=L;
        end
    end
end


end
