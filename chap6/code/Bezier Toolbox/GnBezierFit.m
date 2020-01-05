function [Bez,W] = GnBezierFit(W,G,varargin)
% this function fits a piecewise cubic bezier curve with continuous
% curvature (C2 curve) through given waypoints W. Works for both open and
% closed waypoint arrays. If the euclidian distance between the first and
% last waypoint is smaller then 'tol', a closed bezier curve is fitted,
% in which the first and last segment are also smoothly connected.
% If difference is larger then 'tol', an open bezier curve is fitted where
% the curvature of the start and end point is zero. This function uses the
% adaptations proposed in, http://www.jacos.nl/jacos_html/spline/theory.
% The resulting splines typically have lower curvature then with the
% original method.
%
% Syntax:
% [Bez] = G3BezierFit2(W,Gi);           %default version uses weighting=1
% [Bez] = G3BezierFit2(W,Gi,weighting); %toggle variable weighting

% Inputs:
% W should be an array size 2*n in which n is the number of waypoints
% nt the number of samples on each segment for evalutation
% theta the angle over which input W is rotated around the origin
% Gi the order of smoothness (eg G=1 -> continuous tangents, G=2
%
% Outputs:
% output Bez is a struct of size n-1 with fields Q and C
% Q are the control points size [size(W,1),(G+2)]
% w are the weights for rational bezier curve (unrelated to weighting
%   toggle) these are set to an array of ones in this function.
%
%
% Eindhoven University of Technology
% Robbin van Hoek - okt 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%column norm
ColNorm = @(X,pow) sum(abs(X).^pow,1).^(1/pow);

%check if closed track
tol    = 1e-3; %distance tolerance for close track and repeated points
closed = (norm(W(:,1)-W(:,end),2)<tol);

if numel(varargin)==1
    weighting=varargin{1};
else
    weighting=1;
end

%check for repeated points (except end and start point) and delete them
dW = diff(W,1,2);  %difference between consecutive knots
dist = ColNorm(dW,2); % should be n long
Idel = find(dist==0)+1;
W(:,Idel)=[];

% for ii=1:size(W,2)-1
%     if (norm(W(:,ii)-W(:,ii+1),2)<tol)
%         Idel=[Idel,ii];
%     end
% end
% W(:,Idel)=[];


%% calculate points
%predefine constant matrices
Bi1=pascal(G+1,1);
Bi1=fliplr(Bi1(2:end,2:end)).*((-1).^(1:G).');
Bi2 = fliplr(Bi1).*(-1).^repmat((0:G-1).',1,G);

if closed %% closed spline calculation with n independent knots
    
    %create knot array (n+1)x2
    K=W(:,1:end-1).';
    
    %check number of knots
    [n,m]=size(K);
    
    %w_i = K_i - K_{i+1}   for i={0,...,n-1}
    dK = diff(K([1:end,1],:),1,1);  %difference between consecutive knots
    w = ColNorm(dK.',2).'; % should be n long
    if weighting==0
        w=ones(size(w));
    end
    
    %initialize matrices
    N=n;
    Ai = zeros(G*N,G*N);
    ri = zeros(G*N,m);
    
    for idx=1:N
        Bi  = zeros(G,G*2);
        if idx==N
            wi  = w(idx);
            wi1 = w(1);
            Ki1 = K(1,:);
        else
            wi  = w(idx);
            wi1 = w(idx+1);
            Ki1 = K(idx+1,:);
        end
        
        Bi(:,1:G)       = Bi1.*wi1.^(1:G).';
        Bi(:,(G+1):end) = Bi2.*wi.^(1:G).';
        
        Ri = (wi1.^(1:G).*(-1).^(0:(G-1))+wi.^(1:G)).'*Ki1;
        
        %fill in block
        ics = 1+(idx-1)*G;
        irs = 1+(idx-1)*size(Bi,1);
        if idx==N
            Ai(irs:(irs+size(Bi,1)-1), [(end-(G-1)):end,1:G])=Bi;
        else
            Ai(irs:(irs+size(Bi,1)-1), ics:(ics+size(Bi,2)-1))=Bi;
        end
        
        %fill in 'b' vector
        ri(irs:(irs+size(Bi,1)-1),:) = Ri;
        
    end
    
else %% open curve with n+1 independent knots
    %create knot array (n+1)x2
    K = W.';
    [n,m] = size(K);
    
    %w_i = K_i - K_{i+1}   for i={0,...,n-1}
    dK = diff(K,1,1);       %difference between consecutive knots
    w  = ColNorm(dK.',2).'; %should be n long
    if weighting==0
        w=ones(size(w));
    end
    
    %initialize matrices
    Ai = zeros(G*(n-1),G*(n-1));
    ri = zeros(G*(n-1),m);
    for idx=1:n-2
        Bi  = zeros(G,2*G);
        wi  = w(idx);
        wi1 = w(idx+1);
        Ki1 = K(idx+1,:);
        
        Bi(:,1:G)       = Bi1.*wi1.^(1:G).';
        Bi(:,(G+1):end) = Bi2.*wi.^(1:G).';
        
        %Ri = [wi1+wi; wi^2-wi1^2; wi1^3+wi^3]*Ki1;
        Ri = (wi1.^(1:G).*(-1).^(0:(G-1))+wi.^(1:G)).'*Ki1;
        
        %fill in block
        ics = 1+(idx-1)*G;
        irs = 1+(idx-1)*size(Bi,1);
        Ai(irs:(irs+size(Bi,1)-1), ics:(ics+size(Bi,2)-1))=Bi;
        
        %fill in 'b' vector
        ri(irs:(irs+size(Bi,1)-1),:) = Ri;
    end
    
    %%% complete boundairy conditions
    
    %index of rows for initial and final constrains
    idx_ini = ((size(Ai,2)-G+1):(size(Ai,2)-ceil(G/2)));
    idx_fin = ((size(Ai,2)-ceil(G/2)+1):size(Ai,2));
    temp = Bi2(2:end,:); %derivative coefficients
    
    if ~isempty(temp) %temp will be empty if only G1 continuity is used
        %initial    floor(Gi/2) constrains (2nd derivative and higher)
        Ai(idx_ini,1:size(temp,2)) = temp(1:numel(idx_ini),:);
        ri(idx_ini,:) = ones(numel(idx_ini),1)*K(1,:);
        %final      ceil(Gi/2) constrains  (2nd derivative and higher)
        Ai(idx_fin,(end-size(temp,2)+1):end) = fliplr(temp(1:numel(idx_fin),:));
        ri(idx_fin,:) = ones(numel(idx_fin),1)*K(end,:);
    end
    
end

%% assembly bezier struct
%Solve for control points
%Xi = inv(Ai)*ri;
Xi = pinv(Ai)*ri;

%construct remaining control points
K = W.';
if closed
    nseg=n;
else
    nseg=n-1;
end

Bez(nseg)=struct;
for ii=1:nseg
    Q=zeros(m,G+2);
    Q(:,[1,end]) = [K(ii,:).', K(ii+1,:).'];
    for idx=2:size(Q,2)-1
        Q(:,idx) = Xi((ii-1)*G+(idx-1),:).';
    end
    
    %store in struct
    Bez(ii).Q=Q; %control points for segment
    Bez(ii).w=ones(1,size(Q,2)); %control points for segment
end


end

% function R=Rot(theta)
% R=[cos(theta), -sin(theta); sin(theta), cos(theta)];
% end