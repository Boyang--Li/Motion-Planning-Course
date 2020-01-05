clear variables


%Test repeated points
% W=[0,0; 2,0; 4,0; 4,0; 6,2; 6,4; 6,6].'; %single corner

% %example to compare waypoint distance weighting
% W=[0,0; -0.0,0.98; 0.1,1;  1.2,1.1; 0.35,1.55; 0.3,1.53; -1,1.6].'; %geometric/parametric continuity comparison
% 
% %3D example
W=[0,0,0; -0.0,0.98,0.2; 0.1,1,0.3;  1.2,1.1,1; 0.35,1.55,2; 0.3,1.53,2.1; -1,1.6,1.5].'; %geometric/parametric continuity comparison

%circular/elliptic fits
% theta=linspace(0,2*pi,50)+2*pi/9; %circular fit
% W=[cos(theta);sin(theta)].*[2;1];


G=[3,4,5,6,7]; %array of smoothness

R(numel(G))=struct;
for kk=1:numel(G)
     R(kk).Bez = GnBezierFit(W,G(kk),1); %include weighting
     R(kk).Bez = EvaluateBezierStruct(R(kk).Bez,100);
end

%% plotting
figure('Name','Compare smoothness of fit with higher order Beziers')
ncol=3;

hax1=gobjects(numel(R),ncol);
for kk=1:numel(R)
    hax1((kk-1)*ncol+1)=subplot(numel(R),ncol,(kk-1)*ncol+1); hold on; grid minor; ylabel(['G',num2str(G(kk)),' y[m]'] ); axis equal; %title(['G',num2str(G(kk))]);
    hax1((kk-1)*ncol+2)=subplot(numel(R),ncol,(kk-1)*ncol+2); hold on; grid minor; ylabel('curv [1/m]')
    hax1((kk-1)*ncol+3)=subplot(numel(R),ncol,(kk-1)*ncol+3); hold on; grid minor; ylabel( hax1((kk-1)*ncol+3),'d/ds curv [1/m2]')
    if kk==numel(R)
        xlabel(hax1((kk-1)*ncol+1),'x [m]')
        xlabel(hax1((kk-1)*ncol+2), 's [m]');
        xlabel( hax1((kk-1)*ncol+3),'s [m]');
    end
    
    colormap(hax1((kk-1)*ncol+1),lines(numel(R(kk).Bez)+1))
    cMAP=colormap(hax1((kk-1)*ncol+1));
    
    if size(W,1)==2

        plot(hax1((kk-1)*ncol+1),W(1,:),W(2,:),'k*') %plot waypoints
        for ii=1:numel(R(kk).Bez)
            plot(hax1((kk-1)*ncol+1),R(kk).Bez(ii).X(1,:),R(kk).Bez(ii).X(2,:),'Color',cMAP(ii,:),'Linewidth',2);
            plot(hax1((kk-1)*ncol+1),R(kk).Bez(ii).Q(1,:),R(kk).Bez(ii).Q(2,:),'x-','Color',cMAP(ii,:));
            plot(hax1((kk-1)*ncol+2),R(kk).Bez(ii).Lt,R(kk).Bez(ii).C,'Color',cMAP(ii,:));

            [dkds,dkdu,duds] = Curvederivative(R(kk).Bez(ii)); 

            plot(hax1((kk-1)*ncol+3),R(kk).Bez(ii).Lt, dkds,'Color',cMAP(ii,:));
        end
    elseif size(W,1)==3
         plot3(hax1((kk-1)*ncol+1),W(1,:),W(2,:),W(3,:),'k*') %plot waypoints
        for ii=1:numel(R(kk).Bez)
            plot3(hax1((kk-1)*ncol+1),R(kk).Bez(ii).X(1,:),R(kk).Bez(ii).X(2,:),R(kk).Bez(ii).X(3,:),'Color',cMAP(ii,:),'Linewidth',2);
            plot3(hax1((kk-1)*ncol+1),R(kk).Bez(ii).Q(1,:),R(kk).Bez(ii).Q(2,:),R(kk).Bez(ii).Q(3,:),'x-','Color',cMAP(ii,:));
            
            plot(hax1((kk-1)*ncol+2),R(kk).Bez(ii).Lt,R(kk).Bez(ii).C,'Color',cMAP(ii,:));

            %check the derivative of curvature. 
            [dkds,~,~] = Curvederivative(R(kk).Bez(ii)); 
            plot(hax1((kk-1)*ncol+3),R(kk).Bez(ii).Lt, dkds,'Color',cMAP(ii,:));
        end
        
    end
    linkaxes([hax1((kk-1)*ncol+2), hax1((kk-1)*ncol+3)],'x')
end

%link the view angle for the 3d plots
linkaxes((hax1(1:ncol:end)),'xy')
Link = linkprop((hax1(1:ncol:end)), ...
       {'CameraUpVector', 'CameraPosition', 'CameraTarget','CameraViewAngle'});
setappdata(gcf, 'StoreTheLink', Link);

%% Extra functions
function [varargout] = BezierDeriv(Q,t)
temp=Q;
varargout{nargout-1}=[]; %preallocation
for ii=1:nargout-1
    varargout{ii} =BezierLocal(temp,t);
    temp=Hodograph(temp); %create weights for derivative function
    %varargout{ii} = temp;
end
varargout{nargout} =BezierLocal(temp,t);
end


function [C] = BezierLocal(Q,t)
%check if rational or regular bezier curve. if no info, assume regular
p=size(Q,2)-1; %degree of bezier curve
%m=size(Q,1);   %dimension of space

%construct bernstein basis matrix
Over = @(u,d) factorial(u)./(factorial(d).*(factorial(u-d)));
Bp=zeros(p+1,p+1);
for kk=0:p
    ii=kk:p;
    Bp(kk+1,ii+1) =  (-1).^(ii-kk).*Over(p,ii).*Over(ii,kk) ; %*t^i
end

%evaluate numerically
Up  = t.^((0:p)');   %time power matrix
C   = Q*Bp*Up;

end


function [dkds,dkdu,duds] = Curvederivative(Bez3)
    %function is only valid for regular bezier splines
    colnorm = @(X,pow) sum(abs(X).^pow,1).^(1/pow);
    t=linspace(0,1,numel(Bez3.C));

    [C,Cd,Cdd,Cddd]=BezierDeriv(Bez3.Q,t);
    m=size(C,1);
 
     if m==2
        crossvector = cross([Cd;zeros(1,numel(t))],[Cdd;zeros(1,numel(t))]);
        crossvector2 = cross([Cd;zeros(1,numel(t))],[Cddd;zeros(1,numel(t))]);
        K = crossvector(3,:)./(colnorm(Cd,2).^3); %signed curvature in 2D
        dkdu = crossvector2(3,:)./(colnorm(Cd,2).^3) - 3*sum(Cd.*Cdd,1)./(colnorm(Cd,2).^2).*K;
     elseif m==3
        crossvector = cross(Cd,Cdd);
        crossvector2 = cross(Cd,Cddd);
        K = colnorm(crossvector,2)./(colnorm(Cd,2).^3); %unsigned curvature in 3D
        dkdu = colnorm(crossvector2,2)./(colnorm(Cd,2).^3) - 3*sum(Cd.*Cdd,1)./(colnorm(Cd,2).^2).*K;
     end
    duds = 1./colnorm(Cd,2);
    dkds = dkdu.*duds;
end

