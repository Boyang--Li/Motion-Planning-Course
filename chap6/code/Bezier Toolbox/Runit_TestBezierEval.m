clear variables


%% inputs
Q =[0,0;
    80,0;
    100,20;
    100,100].'; 
w1=[1 1 1 1]; %create array of equal weights (regular, weights used but fast evaluation is still used)
w2=[1 1 3 1]; %create second array of weights (rationa)


%generate curve
t=linspace(0,1,100);

[R(1).C, R(1).theta, R(1).K, R(1).L]  = BezierEval(Q,t); %rational
[R(2).C, R(2).theta, R(2).K, R(2).L]  = BezierEval(Q,t,w1); %rational
[R(3).C, R(3).theta, R(3).K, R(3).L]  = BezierEval(Q,t,w2); %rational


%% plotting
figure('Name','Compare rational with regular bezier curve')
hax(1) = subplot(1,2,1); hold on; grid minor; axis equal; xlabel('x [m]'); ylabel('y [m]')
hax(2) = subplot(1,2,2); hold on; grid minor; xlabel('arc length [s]'); ylabel('curv [1/m]')

colormap(hax(1),lines(numel(R)))
cMAP=colormap(hax(1));
plot(hax(1),Q(1,:),Q(2,:),'k-*')
for ii=1:numel(R)
   plot(hax(1), R(ii).C(1,:),R(ii).C(2,:),'Color',cMAP(ii,:),'LineWidth',2)
   plot(hax(2), R(ii).L,R(ii).K,'Color',cMAP(ii,:))
end