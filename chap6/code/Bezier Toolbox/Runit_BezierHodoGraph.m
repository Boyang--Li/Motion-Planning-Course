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


%check hodograph
Bezd = Hodograph(Bez);
Bezd = EvaluateBezierStruct(Bezd,100);

%% plotting
figure('Name','Compare Hodograph with numerical gradients')
hax(1)=subplot(1,2,1); hold on; grid minor; xlabel('x'); ylabel('y'); axis equal
hax(2)=subplot(2,2,2); hold on; grid minor; ylabel('dx/dt'); title('Comparison of derivatives with numerical gradient')
hax(3)=subplot(2,2,4); hold on; grid minor; xlabel('t'); ylabel('dy/dt'); 
colormap(hax(1),lines(numel(Bez)+1))
cMAP=colormap(hax(1));

PlotBezier(hax(1),Bez)
for ii=1:numel(Bez)
   plot(hax(2),linspace(0,1,100),gradient(Bez(ii).X(1,:),linspace(0,1,100)) ,'Color',cMAP(ii,:))
   plot(hax(2),linspace(0,1,100),Bezd(ii).X(1,:) ,'--','Color',cMAP(ii,:),'LineWidth',2)
   plot(hax(3),linspace(0,1,100),gradient(Bez(ii).X(2,:),linspace(0,1,100)) ,'Color',cMAP(ii,:))
   plot(hax(3),linspace(0,1,100),Bezd(ii).X(2,:) ,'--','Color',cMAP(ii,:),'LineWidth',2)
end



