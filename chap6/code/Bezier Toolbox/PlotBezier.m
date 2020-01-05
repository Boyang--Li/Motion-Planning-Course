function varargout=PlotBezier(varargin)
% This function can be used to plot a Bezier curve in the axes of the
% handle
%
% Syntax:
%    hbez = PlotBezier(hax,Bez)
%    hbez = PlotBezier(Bez)
%    PlotBezier(hax,Bez)
%    PlotBezier(Bez)
%
% Outputs:
% The Bezier curve described by the Bez struct will be plotted in the
% current axes, or the axes passed to the function. If an array of axes
% handles with two elements is passed to the function, the second axes will
% be used to plot curvature as function of arc length.
%
% Eindhoven University of Technology
% Robbin van Hoek - nov 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%set the axes to plot in
if nargin>1
   hax=varargin{1};
   Bez=varargin{2};
else
   hax=gca;
   Bez=varargin{1};
end

%set colormap a priori
colormap(hax(1),lines(numel(Bez)+2))
cMAP=colormap(hax(1)); %store colormap

%check dimension
m=size(Bez(1).Q,1);

%preallocate grapichs objects
hbez=gobjects(numel(Bez),(numel(hax)+1));
if m==2
    for ii=1:numel(Bez)
        hbez(ii,1)=plot(hax(1),Bez(ii).X(1,:),Bez(ii).X(2,:),'Color',cMAP(ii,:),'Linewidth',2);
        hbez(ii,2)=plot(hax(1),Bez(ii).Q(1,:),Bez(ii).Q(2,:),'x-','Color',cMAP(ii,:));
    end
    
elseif m==3
    for ii=1:numel(Bez)
        hbez(ii,1)=plot3(hax(1),Bez(ii).X(1,:),Bez(ii).X(2,:),Bez(ii).X(3,:),'Color',cMAP(ii,:),'Linewidth',2);
        hbez(ii,2)=plot3(hax(1),Bez(ii).Q(1,:),Bez(ii).Q(2,:),Bez(ii).Q(3,:),'x-','Color',cMAP(ii,:));
    end
end


if numel(hax)==2
    colormap(hax(2),lines(numel(Bez)+2))
    for ii=1:numel(Bez)
        if isfield(Bez(ii),'C')
            hbez(ii,3)=plot(hax(2),Bez(ii).Lt,Bez(ii).C,'Color',cMAP(ii,:),'Linewidth',2);   
        end
    end
end

%set outputs
if nargout==1
   varargout{1}=hbez; 
end

end

% linkaxes(hax,'xy')
% Link = linkprop([hax], ...
%        {'CameraUpVector', 'CameraPosition', 'CameraTarget','CameraViewAngle'});
% setappdata(gcf, 'StoreTheLink', Link);

