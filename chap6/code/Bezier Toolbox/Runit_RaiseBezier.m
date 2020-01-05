clear variables


%point
P=[1.6;0.2];

%bezier description
theta=linspace(0,2*pi,10)+2*pi/9; %circular fit
W    =[cos(theta);sin(theta)].*[2;1];


%fit bezier curve
G=4;
[Bez,W] = GnBezierFit(W,G,1); %include weighting
Bez = EvaluateBezierStruct(Bez,100);


%check raising
Bez2 = RaiseBezier(Bez,2);
Bez2 = EvaluateBezierStruct(Bez2,100);

%% Plotting
figure('Name','Raise the order of a Bezier curve')
hax(1)=subplot(2,2,1); hold on; grid minor; axis equal; xlabel('x [m]'); ylabel('y [m]'); title('original curve')
hax(2)=subplot(2,2,3); hold on; grid minor; axis equal; xlabel('x [m]'); ylabel('y [m]'); title('raised curve')
hax(3)=subplot(1,2,2); hold on; grid minor; xlabel('arc length [s]'); ylabel('curv [1/m]'); title('Raised curve dashed')
colormap(hax(1),lines(numel(Bez)+1))
cMAP=colormap(hax(1));

for ii=1:numel(Bez)
   plot(hax(1),Bez(ii).X(1,:),Bez(ii).X(2,:),'Color',cMAP(ii,:),'LineWidth',2)
   plot(hax(1),Bez(ii).Q(1,:),Bez(ii).Q(2,:),'x-','Color',cMAP(ii,:))
   plot(hax(2),Bez2(ii).X(1,:),Bez2(ii).X(2,:),'Color',cMAP(ii,:),'LineWidth',2)   
   plot(hax(2),Bez2(ii).Q(1,:),Bez2(ii).Q(2,:),'-x','Color',cMAP(ii,:))
    
   plot(hax(3),Bez(ii).Lt,Bez(ii).C ,'Color',cMAP(ii,:))
   plot(hax(3),Bez2(ii).Lt,Bez2(ii).C ,'--','Color',cMAP(ii,:),'LineWidth',2)
end



