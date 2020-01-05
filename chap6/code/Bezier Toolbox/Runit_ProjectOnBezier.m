clear variables

%point which is projected on bezier curve
%P=[0.5;1];
%P=[1.99;0];
%P=[-0.5;2];
P=[-0.4;0.6];

%waypoints for bezier fitting
Nway = 10; %number of waypoints
theta=linspace(0,2*pi,10);%+2*pi/7; %circular fit
W    =[cos(theta);sin(theta)].*[2;1];


%check 3D functionality
% P=[-0.4;0.6;0];
% W    =[cos(theta);sin(theta);cos(theta).*sin(theta)].*[2;1;1];



%% create Bezier curve
%fit bezier spline through waypoints
G=4;  %desired smoothness property ( number of control points per segment is G+2 )
[Bez] = GnBezierFit(W,G,1); %include weighting

%create rational curve for tests
w  = ones(1,size(Bez(1).Q,2)); %elements in w correspond to columsn of Q
w(ceil(end/2)) = 5; %change a weight so it becomes rational rather then regular
BezR=Bez;
temp=repmat({w},numel(BezR),1);
[BezR.w]=temp{:};

%populate structures from bezier curves at 100 samples
Bez  = EvaluateBezierStruct(Bez,100);  %evaluates the regular curve
BezR = EvaluateBezierStruct(BezR,100); %evaluates the rational curve


%% projection
%normal mode projection test (on single bezier curve), output is not used
%but these lines demonstrate that the algorithm is capable of projecting on
%a single bezier curve
[~,~,~,~] = BezierProjection(Bez(1).Q,P);
[~,~,~,~] = BezierProjection(BezR(1).Q,P,w);

%struct mode projection
[Ctp,theta,tp,dp]     = BezierProjection(Bez,P);  %projection of point on regular curve
[Ctp2,theta2,tp2,dp2] = BezierProjection(BezR,P); %projection of point on rational curve


%% plotting
figure('Name','Demonstrate point projection on Regular and Rational Bezier curves')
hax(1)=subplot(3,2,[1,3]); hold on; grid minor; ylabel(['G',num2str(G),' y[m]'] ); xlabel('x [m]'); axis equal; title('regular');
hax(2)=subplot(3,2,[2,4]); hold on; grid minor; ylabel(['G',num2str(G),' y[m]'] ); xlabel('x [m]'); axis equal; title('rational');
hax(3)=subplot(3,2,5); hold on; grid minor; ylabel('curv [1/m]'); xlabel('arc length s [m]');
hax(4)=subplot(3,2,6); hold on; grid minor; ylabel('curv [1/m]'); xlabel('arc length s [m]');


%plot Beziers
PlotBezier(hax([1,3]),Bez)
PlotBezier(hax([2,4]),BezR)

if size(W,1)==2
    plot(hax(1),W(1,:),W(2,:),'k*') %plot waypoints to demonstrate fit
    plot(hax(2),W(1,:),W(2,:),'k*') %plot waypoints to demonstrate fit
    plot(hax(1),[P(1), Ctp(1)],[P(2), Ctp(2)],'r-x')
    plot(hax(2),[P(1), Ctp2(1)],[P(2), Ctp2(2)],'r-x')
elseif size(W,1)==3
    plot3(hax(1),W(1,:),W(2,:),W(3,:),'k*') %plot waypoints to demonstrate fit
    plot3(hax(2),W(1,:),W(2,:),W(3,:),'k*') %plot waypoints to demonstrate fit
    plot3(hax(1),[P(1), Ctp(1)],[P(2), Ctp(2)],[P(3), Ctp(3)],'r-x')
    plot3(hax(2),[P(1), Ctp2(1)],[P(2), Ctp2(2)],[P(3), Ctp2(3)],'r-x')
end

linkaxes(hax([1,2]),'xy')
Link = linkprop(hax([1,2]), ...
    {'CameraUpVector', 'CameraPosition', 'CameraTarget','CameraViewAngle'});
setappdata(gcf, 'StoreTheLink', Link);
