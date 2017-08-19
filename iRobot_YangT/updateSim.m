function updateSim(timerSim,event,obj,handlesGUI)
% updateSim(timerSim,event,obj,handlesGUI)
% Update the simulation of the robot.  Use current position and movement
% information to plot the next step and update the robot information
%
% Input:
% timerSim - Timer object (required argument for timers)
% event - Structure, contains information about timer call
%   (required argument for timers)
% obj - CreateRobot object, contains information about robot
% handlesGUI - Structure, handles to all GUI objects for updating plot

% updateSim.m
% Copyright (C) 2011 Cornell University
% This code is released under the open-source BSD license.  A copy of this
% license should be provided with the software.  If not, email:
% CreateMatlabSim@gmail.com
    
    robotRad = .16;                 % Should be equal to obj.radius,
                                    % which cannot be accessed from here.

    % Get time since last update
    tStep= timerSim.InstantPeriod;
    if isnan(tStep)                 % First function call
        tStep= timerSim.Period;     % Assume period is correct
    end
    
    % Use a multiplier on the time step based on simulation speed
    if get(handlesGUI.radio_speed2,'Value')
        tStep= tStep*2;
    elseif get(handlesGUI.radio_speed3,'Value')
        tStep= tStep*3;
    end
    
    % Extract values
    state= getState(obj);
    x= state(1);
    y= state(2);
    v= state(4:5);
    
    % Check for collisions with walls
    collPts= findCollisions(obj);
    
    % Depending on if walls are hit, update position differently
    if isempty(collPts)         % No walls
        driveNormal(obj,tStep)
    elseif size(collPts,1) == 1 % One wall
        if ~collPts(4)          % No corner
            drive1Wall(obj,tStep,collPts)
        else                    % One corner
            driveCorner(obj,tStep,collPts)
        end
    else                        % Two walls
        if ~any(collPts(:,4))   % No corners
            drive2Wall(obj,tStep,collPts)
        elseif xor(collPts(1,4),collPts(2,4))   % One corner
            collPts= collPts(find(~collPts(:,4)),:);
            drive1Wall(obj,tStep,collPts)       % Only look at wall
        else                    % Two corners
            % Only look at corner that is closest to the trajectory
            vec1= [collPts(1,1)-x collPts(1,2)-y]/...
                sqrt((collPts(1,1)-x)^2+(collPts(1,2)-y)^2);
            vec2= [collPts(2,1)-x collPts(2,2)-y]/...
                sqrt((collPts(2,1)-x)^2+(collPts(2,2)-y)^2);
            [closest closeIdx]= max([dot(v,vec1) dot(v,vec2)]);
            collPts= collPts(closeIdx,:);
            driveCorner(obj,tStep,collPts)
        end
    end
    
    % Update the output data
    updateOutput(obj)
    
    % Extract updated state values
    oldstate= state;
    state= getState(obj);
    x= state(1);
    y= state(2);
    th= state(3);
    
    % Update odometry values
    updateOdom(obj,oldstate,state)
    
    % If in robot-centric view mode move plot focal point
    if get(handlesGUI.radio_centric,'Value')
        curr_xlimit= get(handlesGUI.axes_map,'XLim');
        curr_ylimit= get(handlesGUI.axes_map,'YLim');
        half_xdist= (curr_xlimit(2)-curr_xlimit(1))/2;
        half_ydist= (curr_ylimit(2)-curr_ylimit(1))/2;
        set(handlesGUI.axes_map,'XLim',[x-half_xdist x+half_xdist])
        set(handlesGUI.axes_map,'YLim',[y-half_ydist y+half_ydist])
    end
    
    % Line approximation of the robot
    circ_numPts= 21;    % Estimate circle as circ_numPts-1 lines
    circ_ang=linspace(0,2*pi,circ_numPts);
    circ_rad=ones(1,circ_numPts)*robotRad;
    [circ_x circ_y]= pol2cart(circ_ang,circ_rad);
    circ_x=circ_x+x;
    circ_y=circ_y+y;
    
    % Update robot circle and direction line
    handles_robot= get(handlesGUI.figure_simulator,'UserData');
    set(handles_robot(1),'XData',circ_x)
    set(handles_robot(1),'YData',circ_y)
    set(handles_robot(2),'XData',[x x+1.5*robotRad*cos(th)])
    set(handles_robot(2),'YData',[y y+1.5*robotRad*sin(th)])
    
    % Update sensors visualization
    handles_sensors= get(handlesGUI.axes_map,'UserData');
    updateSensorVisualization(obj,handlesGUI,handles_sensors)
end