function BEZ = InterActiveFit()
% This function can be used to generate a Bezier curve interactively via a
% figure window.
%
% Syntax:
% BEZ = InterActiveFit()
%
% Outputs:
% output Bez is a struct with fields
% Q     are the control points size [size(W,1),(G+2)]
% w     are the weights for rational bezier curve (unrelated to weighting
%       toggle) these are set to an array of ones in this function.
% n     are the parameters at which Bez is evaluated on the interval [0,1]
% X     are the evaluated curve coordinates
% theta are the directions of the tangent vectors at each evaluated point
% C     are the curvatures of the curve at each evaluated point
% L     are the cumulative length of the curve for each individual segment
% Lt    are the cumulative length of the curve for entire spline
%
%
% Available Control:
% scrolling in top axes     zooms in and out
% left click in top axes    adds waypoint
% right click in top axes   removes last waypoint
% space         adds the 1st waypoint at the end of  list (closes loop)
% backspace     removes last waypoints
% uparrow       increases the order (and smoothness) of the bezier curve
% downarrow     decreases the order of the bezier curve
% r             autoscales the top axis to the data
% w             toggles weighting based on distance between waypoints
%
% Eindhoven University of Technology
% Robbin van Hoek - nov 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%reset globally used variables
Bez=[];
W=[];
hbez=[];
hW=[];
G=4;
w=1;
Down=0;
X=[];

%% available controls:
% scrolling in top axes     zooms in and out
% left click in top axes    adds waypoint
% right click in top axes   removes last waypoint
% space         adds the first waypoint at the end of the list (closes loop)
% backspace     removes last waypoints
% uparrow       increases the order (and smoothness) of the bezier curve
% downarrow     decreases the order of the bezier curve
% r             autoscales the top axis to the data
% w             toggles weighting based on distance between waypoints

%% Create figure and run functions

%create figure and axes
fig_h = figure('Units','Normalized','OuterPosition',[0 0 1 1],'Name','Interactively generate bezier curve by click and drag - Robbin van Hoek');
hax(2)=subplot(3,1,3); hold on; grid minor; ylabel('curvature [1/m]'); xlabel('arc length s [m]');
hax(1)=subplot(3,1,[1,2]); hold on; grid minor; ylabel(['G',num2str(G),'-fit  y[m]']); xlabel('x [m]'); axis equal;
TITLETEXT={'L/R CLICK in top axis to add/remove points,  SPACEBAR completes closed curve,  CLICKDRAG for preview';
    'UP and DOWN arrow increase/decrease order,  W-key toggles waypoint distance weighting on and off';
    'SCROLLWHEEL to zoom,   R-key autoscales window to data,   press RETURN to finish'};
title(TITLETEXT);
axis(hax(1),[-20,20,-8,8])

%enable manipulation functions
set(fig_h,'KeyPressFcn', @key_pressed_fcn);           %enable interaction with keyboard (deleting / loop closure)
set(fig_h,'windowbuttonmotionfcn', @mouse_move_fcn2); %enable cursor change and click and drag
set(fig_h,'WindowScrollWheelFcn',@figScroll);         %make sure scrolling with mouse wheel is possible
set(fig_h,'WindowButtondownfcn', @WindowButtonDn);    %enable cursor change
set(fig_h,'WindowButtonupFcn',@WindowButtonUp);       %make sure scrolling with mouse wheel is possible

set(hax(1),'ButtonDownFcn',@ClickDown_fcn);           %make sure points are added when clicking


%make sure axis doesnt update aumatically
set(hax(1),'XLimMode','manual','YLimMode','manual');


%copy global parameter to output
while isvalid(fig_h)
    drawnow
    BEZ=Bez;
end

%make sure that only the clicked coordinates are outputted
if size(W,2)<2
    BEZ=[];
else
    [BEZ] = GnBezierFit(W,G,w);
end
return %make sure the caller function receives the output




%nested functions ensure that we use global variables, but only within this
%function

%% Update Bezier
%update the plot information
    function Hbez=UpdateBezier(W,varargin)
        %get axis where bezier is plotted (needed for keydownfnc not having an
        %axis property
        if nargin==2
            hax=varargin{1};
        else
            HAX= get(hbez,'Parent');
            hax=HAX{1};
        end
        
        %if G does not exist, make it and put it in the label
        if ~exist('G') || isempty(G)
            G=4;
            hlabel=get(hax(1),'YLabel');
            set(hlabel,'String',['G',num2str(G),'-fit  y[m]'])
        end
        
        delete(hW);
        hW=plot(W(1,:),W(2,:),'ko');
        
        
        %fit a bezier curve if sufficient points are available
        if size(W,2)>=2
            %fit bezier spline through selected points
            [Bez] = GnBezierFit(W,G,w); %include weighting
            
            %evaluate the curves
            Bez  = EvaluateBezierStruct(Bez,50);  %evaluates the regular curve
            
            %plot in the requested axes
            delete(hbez)
            hbez=PlotBezier(hax,Bez);
        end
        
        %make sure that if enough waypoints are deleted, the entire curve is
        %deleted
        if size(W,2)==1
            delete(hbez)
            hbez=[];
        end
        
        %copy global for output
        Hbez=hbez;
        
    end

%% Button up and down
    function WindowButtonDn(fig_obj,eventDat)
        Down=1;
    end
    function WindowButtonUp(fig_obj,eventDat)
        Down=0;
        
        %X is created in clickdownfunc
        if ~exist('W')
            W=X;
        else
            W=[W,X];
        end
        
        %get the both axis handles (so curvature can also be updated)
        HAX = get(fig_obj,'Children');
        
        %update the Bezier
        UpdateBezier(W,HAX);
        
    end

%% function to obtain location of clicks
    function ClickDown_fcn(hax,eventDat)
        
        %add or remove waypoints on clicks
        if eventDat.Button==1 %add point
            C = get (hax, 'CurrentPoint');
            X=[C(1,1);C(1,2)];
        elseif eventDat.Button==3 %remove last point
            X=[]; %make sure that on release of button no last point is added via the WindowButtonUp function
            if size(W,2)>0
                W(:,end)=[];
                %disp('rightclick detected: deleted last waypoint')
            else
                %disp('rightclick detected: No waypoints left to delete')
            end
        end
        %disp(eventDat);
        
    end

%% function for zooming with mousewheel

    function figScroll(fig_obj,callbackdata)
        
        %when scrolling, first check if middle mouse button is also pressed
        %set cursor to a crosshair if moving over the correct axes (which in this
        %case is the first axis)
        hax = get(fig_obj,'Children');
        
        hAxes = overobj2(fig_obj,'axes');
        if eq(hAxes,hax(1)) %if correct axes
            
            %check the focus point
            cpt = get (hax(1), 'CurrentPoint');
            pt = cpt(1,1:2).';
            
            %get current limits
            Xlim = get(hax(1),'Xlim');
            Ylim = get(hax(1),'Ylim');
            xrange=Xlim(2)-Xlim(1);
            yrange=Ylim(2)-Ylim(1);
            
            xfrac=(pt(1)-Xlim(1))/xrange;
            yfrac=(pt(2)-Ylim(1))/yrange;
            
            %define the new ranges
            zoom=0.4;
            if callbackdata.VerticalScrollCount > 0
                Xrange=xrange*(1+zoom);
                Yrange=yrange*(1+zoom);
            elseif callbackdata.VerticalScrollCount < 0
                Xrange=xrange*(1-zoom);
                Yrange=yrange*(1-zoom);
            end
            
            %calculate new limits
            XLIM = pt(1)+[-xfrac, (1-xfrac)]*Xrange;
            YLIM = pt(2)+[-yfrac, (1-yfrac)]*Yrange;
            
            %set new limits
            set(hax(1),'Xlim',XLIM,'Ylim',YLIM)
            
        else
            return
        end
        
    end

%% function for handling key presses
    function key_pressed_fcn(fig_obj,eventDat)
        
        CHILD = get(fig_obj,'Children');
        hax=CHILD;
        
        key  = get(fig_obj, 'CurrentKey');
        char = get(fig_obj, 'CurrentCharacter');
        mod  = get(fig_obj, 'CurrentModifier');
        
        %for testing
        %disp(eventDat);
        
        %modify control points
        if strcmp(key,'backspace')
            if size(W,2)>0
                W(:,end)=[];
                %disp('backspace detected: deleted last waypoint')
            else
                %disp('backspace detected: No waypoints left to delete')
            end
        elseif strcmp(key,'space')
            if size(W,2)>2
                W=[W,W(:,1)]; %close the loop on spacebar
                %disp('spacebar detected: added first waypoint for loop closure')
            else
                %disp('spacebar detected:can only close a loop if more then 2 waypoints exist')
            end
        end
        
        %modifies the order of bezier curve
        if strcmp(key,'uparrow')
            G=G+1;
            hlabel=get(hax(1),'YLabel');
            set(hlabel,'String',['G',num2str(G),' y[m]'])
        elseif strcmp(key,'downarrow')
            if G>=2
                G=G-1;
                hlabel=get(hax(1),'YLabel');
                set(hlabel,'String',['G',num2str(G),'-fit  y[m]'])
            end
        end
        
        if strcmp(key,'w')
            w=~w; %change weighting for Bezier fitting
        end
        
        %update the bezier information
        Hbez = UpdateBezier(W,hax);
        
        
        
        %reset view to fit data exactly
        if strcmp(key,'r')
            if ~isempty(Hbez) %check data ranges
                xdata=[get(Hbez(:,1:2),'xdata')];
                ydata=[get(Hbez(:,1:2),'ydata')];
                
                Xmin=min([xdata{:}]);
                Xmax=max([xdata{:}]);
                Ymin=min([ydata{:}]);
                Ymax=max([ydata{:}]);
                
                xrange=Xmax-Xmin;
                yrange=Ymax-Ymin;
                Xlim=[Xmin-xrange*0.1, Xmax+xrange*0.1];
                Ylim=[Ymin-yrange*0.1, Ymax+yrange*0.1];
                set(hax(1),'Xlim',Xlim,'Ylim',Ylim)
            else
                set(hax(1),'Xlim',[-1, 1],'Ylim',[-1, 1])
            end
        end
        
        if strcmp(key,'return')
            close(fig_obj)
        end
        
        
    end

%% mouse type function
    function mouse_move_fcn2(fig_obj,eventDat)
        %set cursor to a crosshair if moving over the correct axes (which in this
        %case is the first axis) and predraw the bezier curve
        CHILD = get(fig_obj,'Children');
        HAX=CHILD; %contains both plots
        
        %disp(eventDat)
        hAxes = overobj2(fig_obj,'axes');
        if eq(hAxes,HAX(1)) %~isempty(hAxes)
            set(gcf,'Pointer','crosshair');
            
            if Down==1 %Down is set by WindowButtonDown function, enables click and drag
                C = get (hax(1), 'CurrentPoint');
                X=[C(1,1);C(1,2)];
                if ~exist('W')
                    Wtemp=X;
                else
                    Wtemp=[W,X];
                end
                
                %update the Bezier
                UpdateBezier(Wtemp,HAX);
            end
        else
            set(gcf,'Pointer','arrow');
        end
        
    end

    function h = overobj2(Hfig,Type)
        % This function is undocumented and will change in a future release
        
        %OVEROBJ Get handle of object the pointer is over.
        %   H = OVEROBJ(TYPE) check searches visible objects of Type TYPE in
        %   the PointerWindow looking for one that is under the pointer.  It
        %   returns the handle to the first object it finds under the pointer
        %   or else the empty matrix.
        %
        %   Notes:
        %   Only works with object types that are children of figure
        %
        %   Example:
        %       axes ;
        %       %after executing the following line place the pointer over the axes
        %       %object or else the overobj function will return empty
        %       pause(2),overobj('axes')
        %
        %   Copyright 1984-2013 The MathWorks, Inc.
        %
        % EDITED TO WORK FOR INTENTED PURPOSES
        %
        fig = Hfig;
        
        % Look for quick exit
        if fig==0
            h = [];
            return
        end
        
        figUnit = get(fig,'Units'); %get original figure units
        set(fig,'Units','Normalized'); %set to normalized
        figPos = get(fig,'Position');      %get figure position
        p1     = get(Hfig,'CurrentPoint'); %get cursor current point
        set(fig,'Units',figUnit) %reset to original units
        
        %ret relative normalized position of cursor in figure
        % x=p1(1)/figPos(3);
        % y=p1(2)/figPos(4);
        x=p1(1);%/figPos(3);
        y=p1(2);%/figPos(4);
        
        %compare position with each of the children
        c = findobj(get(fig,'Children'),'flat','Type',Type,'Visible','on');
        for h = c'
            hUnit = get(h,'Units');
            set(h,'Units','norm')
            r = get(h,'Position');
            set(h,'Units',hUnit)
            if ( (x>r(1)) && (x<r(1)+r(3)) && (y>r(2)) && (y<r(2)+r(4)) )
                return
            end
        end
        h = [];
    end

end
