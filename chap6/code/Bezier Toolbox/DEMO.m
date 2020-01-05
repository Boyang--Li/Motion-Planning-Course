
%% Fitting
disp('Fit a bezier curve through waypoints (open or closed curve) with arbitrary smoothness')
Runit_FitGnCurve

pause()

%% Derivatives regular bezier curve
disp('Calculate Hodograph for the derivatives of the regular Bezier curve')
Runit_BezierHodoGraph

pause()

%% raise order of regular curves
disp('Raise the order of a bezier curve and maintain its shape')
Runit_RaiseBezier

pause()

%% Evaluate the curves of both regular and rational curves
disp('Evaluate both regular and rational Bezier curves')
Runit_TestBezierEval

pause()

%% Project point on Bezier curve
disp('Compute projection of a point on a Bezier Spline or curve (regular/rational)')
Runit_ProjectOnBezier

pause()

%% Interactively draw Bezier splines
disp('Interactively Draw Regular Bezier Splines')
Runit_interactiveBezier
