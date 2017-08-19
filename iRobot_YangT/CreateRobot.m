classdef CreateRobot < handle
% Creates definition of class, instances of which will include all
% properties and controls associated with a single CreateRobot robot

% CreateRobot.m
% Copyright (C) 2011 Cornell University
% This code is released under the open-source BSD license.  A copy of this
% license should be provided with the software.  If not, email:
% CreateMatlabSim@gmail.com
    
    properties(Constant, GetAccess= 'private')
    % Values cannot be changed
    % All units are kg/m/s/rad unless otherwise noted
        radius = 0.16;          % Radius of the robot (model as a circle)
        wheelbase = 0.258       % Distance between wheels of iRobot Create
        rangeIR= 0.1;           % Linear range of the infrared sensor
        rangeSonar= 3;          % Linear range of all sonar sensors
        rangeMinSonar= 0.02;    % Minimum linear range of sonar sensor
        rangeLidar= 4;          % Linear range of LIDAR sensor
        rangeMinLidar= 0.02;    % Minimum linear range of LIDAR sensor
        angRangeLidar= pi*240/180;      % Angular range of LIDAR sensor
        numPtsLidar= 681;       % Number of points in LIDAR sensing range
        rangeCamera = 6;        % Linear range of the camera for blob detection
        angRangeCamera = pi*60/180; % Angular range of camera (each side)
        cameraDisplace = 0.13;  % Position of camera along robot's x-axis      
        frictionKin= 0.35;    %  Coefficient of kinetic friction of robot and wall
    end
    
    properties(GetAccess= 'private', SetAccess= 'private')
    % Values require get and set methods to view or manipulate
        % Sensor variables
        odomDist;   % Distance traveled since last check from odometry
                    % Format: double
        odomAng;    % Angle turned since last check from odometry
                    % Format: double
        noise;      % Contains noise data for sensors used
                    % Format: structure, fieldnames are sensor names and
                    %   field values are sensor noise [mean standard_dev]
        comDelay;   % Communication delay of commands to robot
                    % Format: double
        
        % Robot state variables
        posAbs;     % Position in absolute coordinates
                    % Format: vector of doubles, [x y]
        velAbs;     % Velocity in absolute coordinates
                    % Format: vector of doubles, [x y]
        thAbs;      % Yaw angle relative to positive x-axis
                    % Format: double (-pi < thAbs <= pi)
        wAbs;       % Angular velocity
                    % Format: double (positive counter-clockwise)
        velInt;     % Forward velocity intended by motor commands
                    % Format: double
        wInt;       % Angular velocity intended by motor commands
                    % Format: double
        autoEnable; % Enabled/Disabled state of autonomous control
                    % Format: boolean
        dataHist;   % Time history of position and function calls for output
                    % Format: cell array, columns
                    %   [time x y th fcn_called arg_val]
                    %   time - Double, time relative to start of autonomous code
                    %   x - Double, x-coordinate of center of robot
                    %   y - Double, y-coordinate of center of robot
                    %   th - Double, angle of robot relative to pos x-axis
                    %   fcn_called - String or cell array of strings, 
                    %     name(s) of the function(s) called in that time step
                    %   arg_val - Vector of doubles or cell array of vectors,
                    %     value(s) for the input OR output argument(s);
                    %     the robot object argument is ignored
        
        % Environment variables
        timeElap;   % Time of the start of autonomous code execution
                    % Format: unsigned 64 bit integer
                    % time syntax - to be used with toc function
        handlesGUI; % Handles to all GUI objects
                    % Format: structure containing handle numbers
                    %   See SimulatorGUI.fig and SimulatorGUI.m for more
                    %   format information on the contents of the structure
        mapStart;   % Contains robot start position/orientation information
                    % Format: vector of doubles, [x y th]
        mapWalls;   % Contains obstacle (wall) start and end points
                    % Format: matrix of doubles, columns [x1 y1 x2 y2]
        mapLines;   % Contains line start and endpoints
                    % Format: matrix of doubles, columns [x1 y1 x2 y2]
        mapBeacs;   % Containing beacon location, color, and ID information
                    % Format: cell array of doubles, 
                    %   columns {x y red green blue ID}
        mapVWalls;  % Contains virtual wall location, direction, and 
                    %   strength information
                    % Format: matrix of doubles, columns [x y th strength]
    end
    
    methods(Static, Access= 'public')
    % Functions that do not need the object defined to operate
        
        function newAng= wrap2pi(oldAng)
        % newAng = wrap2pi(oldAng)
        % Wrap angle in radians to [-pi pi]
        % Replace wrapToPi to avoid dependence on mapping toolbox
        %
        % Input:
        % oldAng - Angle to be wrapped within limits (rad)
        %
        % Output:
        % newAng - Output angle within limits (rad)
            
            % Increase if to low
            while oldAng < -pi
                oldAng= oldAng+2*pi;
            end
            
            % Decrease if too high
            while oldAng > pi
                oldAng= oldAng-2*pi;
            end
            newAng= oldAng;
        end
    end
    
    methods(Access= 'public')
    % Functions available to call from controller function or other files
    
    % Constructor Function
        function obj= CreateRobot(varargin)
        % obj = CreateRobot
        % Creates instance of the user-defined class Create Robot and
        % initializes all properties. Note that if CreateRobot is not
        % called by SimulatorGUI (with handlesGUI argument), full
        % functionality impossible.
        %
        % obj = CreateRobot(handlesGUI)
        % Format of function call from SimulatorGUI. Passes along structure
        % of handles for the GUI to allow full functionality
        %
        % Input:
        % handlesGUI - Structure of handles to GUI objects
        %   e.g. handlesGUI.push_adv - handle for advance button
        %
        % Output:
        % obj - Instance of class CreateRobot with all fields initialized
            
            % Deal with input argument
            % Will be handles to SimulatorGUI if called by simulator
            if ~isempty(varargin) && isstruct(varargin{1})
                obj.handlesGUI= varargin{1};
            else
                obj.handlesGUI= [];
            end
            
            % Assign properties
            obj.mapStart= [0 0 0];  % Default start position at origin
            obj.mapWalls= [];
            obj.mapLines= [];
            obj.mapBeacs= {};
            obj.mapVWalls= [];
            obj.timeElap= [];
            obj.dataHist= {};
            obj.noise= struct;      % Empty structure to store noise data
            obj.comDelay= 0;        % Default instantaneous communication
            obj.posAbs= obj.mapStart(1:2); % Initial position
            obj.velAbs= [0 0]; % Assume robot is stationary to start
            obj.thAbs= obj.mapStart(3);
            obj.wAbs= 0;
            obj.velInt= 0;
            obj.wInt= 0;
            obj.autoEnable= false;  % Start in manual control mode
            obj.odomDist= 0;        % Start odometry at zero
            obj.odomAng= 0;
        end
        
    % SimulatorGUI Control Functions
        function [rad rIR rSon rLid angRLid numPtsLid]= getConstants(obj)
        % [rad rIR rSon rLid angRLid numPtsLid] = getConstants(obj)
        % Output constant properties of the robot for simulator usage
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % rad - Double, radius of robot (m)
        % rIR - Double, range of infrared wall sensor (m)
        % rLid - Double, linear range of LIDAR sensor (m)
        % angRLid - Double, angular range of LIDAR sensor (rad)
        % numPtsLid - Double, number of points used by LIDAR sensor
            
            rad= obj.radius;
            rIR= obj.rangeIR;
            rSon= obj.rangeSonar;
            rLid= obj.rangeLidar;
            angRLid= obj.angRangeLidar;
            numPtsLid= obj.numPtsLidar;
        end
        
        function [start walls lines beacs vwalls]= getMap(obj)
        % [start walls lines beacs vwalls] = getMap(obj)
        % Output the obstacle and robot map data for plotting
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % start - Vector containing robot start position information
        % walls - Matrix containing obstacle information
        % lines - Matrix containing line information
        % beacs - Cell array containing beacon information
        % vwalls - Matrix containing virtual wall information
        %   See the properties specification for format information
            
            % Assign variables from object values
            start= obj.mapStart;
            walls= obj.mapWalls;
            lines= obj.mapLines;
            beacs= obj.mapBeacs;
            vwalls= obj.mapVWalls;
        end
        
        function setMap(obj,walls,lines,beacs,vwalls)
        % setMap(obj,walls,lines,beacs,vwalls)
        % Change the obstacle map data in the robot object
        %
        % Input:
        % obj - Instance of class CreateRobot
        % walls - % Matrix containing obstacle information
        % lines - % Matrix containing line information
        % beacs - % Cell array containing beacon information
        % vwalls - % Matrix containing virtual wall information
        %   See properties specification for format information
            
            % Save map information to object
            obj.mapWalls= walls;
            obj.mapLines= lines;
            obj.mapBeacs= beacs;
            obj.mapVWalls= vwalls;
        end
        
        function setMapStart(obj,origin)
        % setMapStart(obj,origin)
        % Change the mapStart data in the robot object. Move the robot to
        % that position.
        %
        % Input:
        % obj - Instance of class CreateRobot
        % origin - Vector of doubles, containing information about the
        %   robot start position in the format [x y th]
            
            obj.mapStart= origin;
            obj.posAbs= origin(1:2);
            obj.thAbs= origin(3);
        end
        
        function setNoise(obj,noiseStruct)
        % setNoise(obj,noiseStruct)
        % Change the noise data for the sensors in the robot object
        %
        % Input:
        % obj - Instance of class CreateRobot
        % noise - Structure, containing noise information
        %   Fieldnames are sensor names with values of [mean standard_dev]
        %   eg. noise.lidar = [0.003 0.012]
            
            % Set property value
            obj.noise= noiseStruct;
        end
        
        function setComDelay(obj,tDelay)
        % setComDelay(obj,tDelay)
        % Change the communication delay property in the robot object
        % This delay will occur on every function call from the
        % autonomous program
        %
        % Input:
        % obj - Instance of class CreateRobot
        % tDelay - Double, communication delay (s)
            
            obj.comDelay= tDelay;
        end
        
        function manualKeyboard(obj,velChange)
        % manualKeyboard(obj,velChange)
        % Change the velocity of the robot according to manual controls
        %
        % Input:
        % obj - Instance of class CreateRobot
        % velChange - Vector of doubles [linear angular], controlling
        %   velocity; positive increases, negative decreases, zero leaves 
        %   it as is, and NaN will zero the velocity
            
            % Change robot object property values
            if any(isnan(velChange))
                obj.velInt= 0;
                obj.wInt= 0;
            else
                % Find intended velocity
                FwdVel= obj.velInt+velChange(1);
                AngVel= obj.wInt+velChange(2);
                
                % Limit individual wheel velocities
                wheelRight= FwdVel+AngVel*obj.wheelbase/2;
                wheelLeft= FwdVel-AngVel*obj.wheelbase/2;
                if abs(wheelRight) > 0.5 || abs(wheelLeft) > 0.5
                    wheelRight= min(max(wheelRight,-0.5),0.5);
                    wheelLeft= min(max(wheelRight,-0.5),0.5);
                    disp(['Warning: desired velocity combination '...
                        'exceeds limits'])
                    FwdVel= (wheelRight+wheelLeft)/2;
                    AngVel= (wheelRight-wheelLeft)/obj.wheelbase;
                end
                obj.velInt= FwdVel;
                obj.wInt= AngVel;
            end
        end
        
        function setAutoEnable(obj,state)
        % setAutoEnable(obj,state)
        % Imports new state for autoEnable property
        %
        % Input:
        % obj - Instance of class CreateRobot
        % state - Boolean, true if autonomous control is enabled
            
            obj.autoEnable= state;
        end
        
        function autoCheck(obj)
        % autoCheck(obj)
        % Checks if autonomous control program is allowed to execute
        % If it is not allowed to execute, it will throw an error to exit
        % all currently running functions until the error is caught
        %
        % Input:
        % obj - Instance of class CreateRobot
            
            % Check if autonomous control is disabled
            if ~obj.autoEnable
                % Throw exception to exit autonomous control program
                error('SIMULATOR:AutonomousDisabled',...
                    ['Autonomous code execution is disabled.  '...
                    'SimulatorGUI must be open and it must call the '...
                    'control program for autonomous mode to enable'])
            end
        end
        
        function updateSensorVisualization(obj,handlesGUI,handles_sensors)
        % updateSensorVisualization(obj,handlesGUI,handles_sensors)
        % Updates the data used to plot the sensors visual representation
        %
        % Input:
        % obj - Instance of class CreateRobot
        % handlesGUI - Structure of doubles, handles to all GUI objects
        % handles_sensors - Vector of doubles, handles to all sensor
        %   visualization plots;
        %   order specified by creation in SimulatorGUI_OpeningFcn
            
            % Wall sensor
            if get(handlesGUI.chkbx_wall,'Value')
                % Calculate position of sensor
                sensAng= -0.225*pi; % Sensor placement relative to front
                x_sensor= obj.posAbs(1)+obj.radius*cos(obj.thAbs+sensAng);
                y_sensor= obj.posAbs(2)+obj.radius*sin(obj.thAbs+sensAng);
                sensAngDir= -0.5*pi;    % Sensor looks to the right
                
                % Find if wall is detected
                wall= genIR(obj);
                if wall     % Plot full range if wall is detected
                    set(handles_sensors(1),'XData',[x_sensor x_sensor+...
                        obj.rangeIR*cos(obj.thAbs+sensAngDir)])
                    set(handles_sensors(1),'YData',[y_sensor y_sensor+...
                        obj.rangeIR*sin(obj.thAbs+sensAngDir)])
                    set(handles_sensors(1),'Visible','on')
                else
                    set(handles_sensors(1),'Visible','off')
                end
            end

            % Sonar sensors
            if get(handlesGUI.chkbx_sonar,'Value')
                distSonar= genSonar(obj);
                for i= 1:4
                    th_sensor= obj.thAbs+(i-1)*pi/2;
                    x_sensor= obj.posAbs(1)+obj.radius*cos(th_sensor);
                    y_sensor= obj.posAbs(2)+obj.radius*sin(th_sensor);
                    set(handles_sensors(1+i),'XData',...
                        [x_sensor x_sensor+distSonar(i)*cos(th_sensor)])
                    set(handles_sensors(1+i),'YData',...
                        [y_sensor y_sensor+distSonar(i)*sin(th_sensor)])
                end
            end

            % Bump sensors
            if get(handlesGUI.chkbx_bump,'Value')
                bump= genBump(obj);
                x_robot= obj.posAbs(1);
                y_robot= obj.posAbs(2);
                th_robot= obj.thAbs;
                if bump(1)      % Right bump sensor
                    set(handles_sensors(6),'XData',...
                        [x_robot+obj.radius*cos(th_robot-pi/2) ...
                        x_robot+obj.radius*cos(th_robot-0.135*pi)])
                    set(handles_sensors(6),'YData',[y_robot+obj.radius*...
                        sin(th_robot-pi/2) y_robot+obj.radius*...
                        sin(th_robot-0.135*pi)])
                    set(handles_sensors(6),'Visible','on')
                else
                    set(handles_sensors(6),'Visible','off')
                end
                if bump(2)      % Front bump sensor
                    set(handles_sensors(7),'XData',...
                        [x_robot+obj.radius*cos(th_robot-0.135*pi) ...
                        x_robot+obj.radius*cos(th_robot+0.135*pi)])
                    set(handles_sensors(7),'YData',...
                        [y_robot+obj.radius*sin(th_robot-0.135*pi) ...
                        y_robot+obj.radius*sin(th_robot+0.135*pi)])
                    set(handles_sensors(7),'Visible','on')
                else
                    set(handles_sensors(7),'Visible','off')
                end
                if bump(3)      % Left bump sensor
                    set(handles_sensors(8),'XData',...
                        [x_robot+obj.radius*cos(th_robot+0.135*pi) ...
                        x_robot+obj.radius*cos(th_robot+pi/2)])
                    set(handles_sensors(8),'YData',...
                        [y_robot+obj.radius*sin(th_robot+0.135*pi) ...
                        y_robot+obj.radius*sin(th_robot+pi/2)])
                    set(handles_sensors(8),'Visible','on')
                else
                    set(handles_sensors(8),'Visible','off')
                end
            end

            % Cliff sensors
            if get(handlesGUI.chkbx_cliff,'Value')
                cliff= genCliff(obj);
                x_robot= obj.posAbs(1);
                y_robot= obj.posAbs(2);
                th_robot= obj.thAbs;
                if cliff(1) <= 5.4      % Right cliff sensor
                    set(handles_sensors(9),'XData',...
                        x_robot+obj.radius*cos(th_robot-pi/3))
                    set(handles_sensors(9),'YData',...
                        y_robot+obj.radius*sin(th_robot-pi/3))
                    set(handles_sensors(9),'Visible','on')
                else
                    set(handles_sensors(9),'Visible','off')
                end
                if cliff(2) <= 5.4      % Front-right cliff sensor
                    set(handles_sensors(10),'XData',...
                        x_robot+obj.radius*cos(th_robot-pi/10))
                    set(handles_sensors(10),'YData',...
                        y_robot+obj.radius*sin(th_robot-pi/10))
                    set(handles_sensors(10),'Visible','on')
                else
                    set(handles_sensors(10),'Visible','off')
                end
                if cliff(3) <= 5.4      % Front-left cliff sensor
                    set(handles_sensors(11),'XData',...
                        x_robot+obj.radius*cos(th_robot+pi/10))
                    set(handles_sensors(11),'YData',...
                        y_robot+obj.radius*sin(th_robot+pi/10))
                    set(handles_sensors(11),'Visible','on')
                else
                    set(handles_sensors(11),'Visible','off')
                end
                if cliff(4) <= 5.4      % Left cliff sensor
                    set(handles_sensors(12),'XData',...
                        x_robot+obj.radius*cos(th_robot+pi/3))
                    set(handles_sensors(12),'YData',...
                        y_robot+obj.radius*sin(th_robot+pi/3))
                    set(handles_sensors(12),'Visible','on')
                else
                    set(handles_sensors(12),'Visible','off')
                end
            end

            % LIDAR sensor
            if get(handlesGUI.chkbx_lidar,'Value')
                x_sensor= obj.posAbs(1)+obj.radius*cos(obj.thAbs);
                y_sensor= obj.posAbs(2)+obj.radius*sin(obj.thAbs);
                
                % Get noise parameters
                if isfield(obj.noise,'lidar')
                    noiseAvg= obj.noise.lidar(1);
                    noiseStDev= obj.noise.lidar(2);
                else
                    noiseAvg= 0;
                    noiseStDev= 0;
                end
                
%                 % First line
%                 th_sensor= obj.thAbs-obj.angRangeLidar/2;
%                 dist= findDist(obj,x_sensor,y_sensor,...
%                     obj.rangeLidar,th_sensor)+noiseAvg+noiseStDev*randn;
%                 set(handles_sensors(13),'XData',...
%                     [x_sensor x_sensor+dist*cos(th_sensor)])
%                 set(handles_sensors(13),'YData',...
%                     [y_sensor y_sensor+dist*sin(th_sensor)])
%                 
%                 % Second line
%                 th_sensor= obj.thAbs+obj.angRangeLidar/2;
%                 dist= findDist(obj,x_sensor,y_sensor,...
%                     obj.rangeLidar,th_sensor)+noiseAvg+noiseStDev*randn;
%                 set(handles_sensors(14),'XData',...
%                     [x_sensor x_sensor+dist*cos(th_sensor)])
%                 set(handles_sensors(14),'YData',...
%                     [y_sensor y_sensor+dist*sin(th_sensor)])
                
                % Sensor lines in LIDAR range
                for i= 0:4
                    th_sensor= obj.thAbs+obj.angRangeLidar*(i-2)/4;
                    dist= findDist(obj,x_sensor,y_sensor,obj.rangeLidar,...
                        th_sensor)+noiseAvg+noiseStDev*randn;
                    set(handles_sensors(13+i),'XData',...
                        [x_sensor x_sensor+dist*cos(th_sensor)])
                    set(handles_sensors(13+i),'YData',...
                        [y_sensor y_sensor+dist*sin(th_sensor)])
                end
            end
        end
        
        function startTimeElap(obj)
        % startTimeElap(obj)
        % Start the timer to determine time steps for data output
        %
        % Input:
        % obj - Instance of class CreateRobot
            
            obj.timeElap= tic;
        end
        
        function updateOutput(obj)
        % updateOutput(obj)
        % Update the data history to be outputted after autonomous code
        % execution is done
        %
        % Input:
        % obj - Instance of class CreateRobot
            
            % Add in state variable information, leave function empty
            if obj.autoEnable   % Autonomous mode
                obj.dataHist= [obj.dataHist ; {toc(obj.timeElap)} ...
                    {obj.posAbs(1)} {obj.posAbs(2)} {obj.thAbs} {[]}];
            end
        end
        
        function addFcnToOutput(obj,fcn_called)
        % addFcnToOutput(obj,fcn_called)
        % Add the string representing a function call to the output data in
        % the current index
        %
        % Input:
        % obj - Instance of class CreateRobot
        % fcn_called - String representation of the function call including
        %   all arguments in the forms of (e.g.):
        %   RoombaInit - zero argument functions
        %   [1 0]= ButtonReaderRoomba - output argument functions
        %   travelDist(0.200,1.500) - input argument functions
            
            incell= obj.dataHist{end,5};    % In function name cell
            if isempty(incell)              % No functions called yet
                obj.dataHist{end,5}= fcn_called;
            else                            % Function(s) have been called
                if ischar(incell)           % Single string
                    obj.dataHist{end,5}= ...
                        {incell ; fcn_called};  % Create cell array
                else                        % Cell array of strings
                    incell{end+1}= fcn_called;  % Add to cell array
                    obj.dataHist{end,5}= incell;
                end
            end
        end
        
        function saveOutput(obj)
        % saveOutput(obj)
        % Save the output data to a .mat file in the current directory
        %
        % Input:
        % obj - Instance of class CreateRobot
            
            % Extract autonomous data immediately
            datahistory= obj.dataHist;
            
            % Set default filename to unused file
            filename= 'SimulatorOutputData_';
            i= 1;
            while exist([filename num2str(i) '.mat'],'file')
                i= i+1;     % Increment number to assign after filename
            end
            filename= [filename num2str(i)];
            
            % Pull up dialogue box for saving file with default name
            filename= inputdlg('Filename (no extension):',...
                'Save Autonomous Data',1,{filename});
            if ~isempty(filename)
                filename= [filename{1} '.mat'];
                
                % Check if file is already in existence
                button= 'Yes';
                if exist(filename,'file')
                    button= questdlg('A file by that name exists.  Overwrite?',...
                        'Verify Overwrite','Yes','Cancel','Cancel');
                end
                % Note that 'Cancel' will not allow file to be saved
                
                % Save file if desired
                if strcmp(button,'Yes')
                    save(filename,'datahistory')
                end
            end
        end
        
        function resetOutput(obj)
        % resetOutput(obj)
        % Reset the data to be outputted after autonomous execution
        %
        % Input:
        % obj - Instance of class CreateRobot
            
            % Clear the robot object property
            obj.dataHist= {};
        end
        
    % Sensor Functions
        function bump= genBump(obj)
        % bump = genBump(obj)
        % Generates a reading for all bump sensors
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % bump - Vector of Boolean doubles [right front left],
        %   1 indicates the sensor is activated, 0 is not activated
            
            % Get intersections with walls
            collPts= findCollisions(obj);
            % Check that there are intersections
            bump= [0 0 0];    % Initialize to no bumps
            if ~isempty(collPts)
                % Change intersection points to local coordinates
                x_int= collPts(:,1)-obj.posAbs(1);
                y_int= collPts(:,2)-obj.posAbs(2);
                
                % Get angle relative to robot front
                th_int= obj.wrap2pi(atan2(y_int,x_int)-obj.thAbs);
                
                % Find sensor activated, if any
                for i= 1:length(th_int)
                    if th_int(i) < -0.135*pi && th_int(i) > -pi/2
                        bump(1)= 1;  % Right sensor
                    elseif th_int(i) < 0.135*pi && th_int(i) > -0.135*pi
                        bump(2)= 1;  % Front sensor (both left and right)
                    elseif th_int(i) < pi/2 && th_int(i) > 0.135*pi
                        bump(3)= 1;  % Left sensor
                    end
                end
                
                % Ensure that front sensor doesn't read with other two
                if bump(2)
                    bump(1)= 0;
                    bump(3)= 0;
                elseif bump(1) && bump(3)
                    bump(1)= 0;
                    bump(2)= 1;
                    bump(3)= 0;
                end
            end
        end
        
        function cliff= genCliff(obj)
        % cliff = genCliff(obj)
        % Generates a reading for the cliff sensors
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % cliff - Vector of doubles [right front-right front-left left],
        %   high values if there are no lines, low values for lines
            
            % Create lines representing robot to look for intersections
            % Get robot information
            x_rob= obj.posAbs(1);
            y_rob= obj.posAbs(2);
            th_rob= obj.thAbs;
            rad= obj.radius;
            
            % Check against every obstacle (individually)
            x_int= [];              % Position of intersections
            y_int= [];
            for i= 1:size(obj.mapLines,1)% Count variable for obstacles
                % Get line data
                x1= obj.mapLines(i,1);
                y1= obj.mapLines(i,2);
                x2= obj.mapLines(i,3);
                y2= obj.mapLines(i,4);
                
                % Find intersection points on infinite lines
                m= (y2-y1)/(x2-x1);     % Slope of the line
                if isinf(m)             % Infinite slope
                    x_posInt= [x1 x2];  % Possible intersection points
                    y_posInt(1)= y_rob+sqrt(rad^2-(x1-x_rob)^2);
                    y_posInt(2)= y_rob-sqrt(rad^2-(x1-x_rob)^2);
                else
                    b= y1-m*x1;
                    % Quadratic equation of form: Ax^2+Bx+C = 0
                    A= m^2+1;
                    B= 2*m*b-2*x_rob-2*m*y_rob;
                    C= x_rob^2+b^2-2*b*y_rob+y_rob^2-rad^2;
                    x_posInt(1)= (-B+sqrt(B^2-4*A*C))/(2*A);
                    x_posInt(2)= (-B-sqrt(B^2-4*A*C))/(2*A);
                    y_posInt(1)= m*x_posInt(1)+b;
                    y_posInt(2)= m*x_posInt(2)+b;
                end
                
                % Check that line intersects circle
                if ~all(isreal(x_posInt)) || ~all(isreal(y_posInt))
                    x_posInt= [];       % Eliminate all points
                    y_posInt= [];
                else
                    elim= [];           % Elements to eliminate
                    for j= 1:length(x_posInt)   % Check both points
                        % Check that they are on the finite line
                        if(x_posInt(j) < min([x1 x2]) || ...
                                x_posInt(j) > max([x1 x2]) || ...
                                y_posInt(j) <  min([y1 y2]) || ...
                                y_posInt(j) > max([y1 y2]))
                            elim= [elim j];
                        end
                    end
                    x_posInt(elim)= [];
                    y_posInt(elim)= [];
                end
                x_int= [x_int ; x_posInt];
                y_int= [y_int ; y_posInt];
            end
            
            % Check that there are intersections
            cliff= ones(1,4);   % Initialize to no lines
            if ~isempty(x_int)
                % Change intersection points to local coordinates
                x_int= x_int-x_rob;
                y_int= y_int-y_rob;
                
                % Get angle relative to robot front
                th_int= obj.wrap2pi(atan2(y_int,x_int)-th_rob);
                
                % Find sensor activated, if any
                for i= 1:length(th_int)
                    if th_int(i) >= -0.37*pi && th_int(i) <= -0.295*pi
                        cliff(1)= 0;  % Right sensor
                    elseif th_int(i) >= -0.105*pi && th_int(i) <= -0.04*pi
                        cliff(2)= 0;  % Front right sensor
                    elseif th_int(i) >= 0.04*pi && th_int(i) <= 0.105*pi
                        cliff(3)= 0;  % Front left sensor
                    elseif th_int(i) >= 0.295*pi && th_int(i) <= 0.37*pi
                        cliff(4)= 0;  % Left sensor
                    end
                end
            end
            
            % Get noise to change the reading of the sensors
            if isfield(obj.noise,'cliff')
                noiseAvg= obj.noise.cliff(1);
                noiseStDev= obj.noise.cliff(2);
                noiseVal= noiseAvg+noiseStDev*randn;
            else
                noiseVal= 0;
            end
            
            % Put in values for sensor reading
            cliff= 20*cliff+1.5+noiseVal;
        end
        
        function wall= genIR(obj)
        % wall = genIR(obj)
        % Generates a reading for the infrared wall sensor
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % wall - Boolean double, 1 if a wall is within range on the right
        %   of the robot
            
            % Calculate position of sensor
            sensAng= -0.225*pi; % Sensor placement relative to front of robot
            x_sensor= obj.posAbs(1)+obj.radius*cos(obj.thAbs+sensAng);
            y_sensor= obj.posAbs(2)+obj.radius*sin(obj.thAbs+sensAng);
            
            % Solve for distance using general function
            sensAngDir= -0.5*pi;    % Sensor looks to the right
            distIR= findDist(obj,x_sensor,y_sensor,...
                obj.rangeIR,obj.thAbs+sensAngDir);
            
            % Get noise to change the effective range of the infrared
            if isfield(obj.noise,'wall')
                noiseAvg= obj.noise.wall(1);
                noiseStDev= obj.noise.wall(2);
                noiseVal= noiseAvg+noiseStDev*randn;
            else
                noiseVal= 0;
            end
            
            % Generate reading
            wall= distIR < obj.rangeIR+noiseVal;
        end
        
        function vwall= genVWall(obj)
        % vwall = genVWall(obj)
        % Generates a reading for the virtual wall sensor
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % vwall - Boolean double, 1 if the robot is within the field of a
        % virtual wall, or the "halo" surrounding the emitter
            
            % Get sensor position
            x_sensor= obj.posAbs(1)+obj.radius*cos(obj.thAbs);
            y_sensor= obj.posAbs(2)+obj.radius*sin(obj.thAbs);
            
            % Define virtual wall emitter constants
            halo_rad= 0.45;     % Radius of the halo around the emitter
            range_short= 2.13;  % Range of the wall on the 0-3' setting
            ang_short= 0.33;    % Angular range on the 0-3' setting
            range_med= 5.56;    % Range of the wall on the 4'-7' setting
            ang_med= 0.49;      % Angular range on the 4'-7' setting
            range_long= 8.08;   % Range of the wall on the 8'+ setting
            ang_long= 0.61;     % Angular range on the 8'+ setting
            
            % Check against all virtual wall ranges
            vwall= 0;   % Initialize output to no-wall
            for i= 1:size(obj.mapVWalls,1)
                % Get emitter position
                x_emit= obj.mapVWalls(i,1);
                y_emit= obj.mapVWalls(i,2);
                
                % Check if sensor is within halo
                if sqrt((x_sensor-x_emit)^2+(y_sensor-y_emit)^2) < halo_rad
                    vwall_seen= 1;
                else
                    % Get more emitter constants
                    th= obj.mapVWalls(i,3);
                    if obj.mapVWalls(i,4) == 1
                        range= range_short;
                        ang= ang_short;
                    elseif obj.mapVWalls(i,4) == 2
                        range= range_med;
                        ang= ang_med;
                    else
                        range= range_long;
                        ang= ang_long;
                    end
                    
                    % Find points that define boundary of virtual wall
                    x_1= x_emit+range*cos(th+ang/2);
                    y_1= y_emit+range*sin(th+ang/2);
                    x_2= x_emit+range*cos(th-ang/2);
                    y_2= y_emit+range*sin(th-ang/2);
                    
                    % Find if sensor is within virtual wall triangle
                    % Use determinant definition of the area of a triangle
                    area_vwall= 0.5*abs(det([x_1 y_1 1 ; x_2 y_2 1 ; ...
                        x_emit y_emit 1]));
                    area_1= 0.5*abs(det([x_1 y_1 1 ; x_2 y_2 1 ; ...
                        x_sensor y_sensor 1]));
                    area_2= 0.5*abs(det([x_emit y_emit 1 ; x_2 y_2 1 ; ...
                        x_sensor y_sensor 1]));
                    area_3= 0.5*abs(det([x_1 y_1 1 ; x_emit y_emit 1 ; ...
                        x_sensor y_sensor 1]));
                    area_tot= area_1+area_2+area_3;
                    vwall_seen= abs(area_tot-area_vwall) < 0.001;
                end
                
                % Check for walls between robot and emitter
                dist_emit= sqrt((x_emit-x_sensor)^2+(y_emit-y_sensor)^2);
                dist_vwall= findDist(obj,x_sensor,y_sensor,dist_emit,...
                    atan2(y_emit-y_sensor,x_emit-x_sensor));
                if vwall_seen && dist_vwall == dist_emit
                    vwall= 1;
                end
            end
        end
        
        function distSonar= genSonar(obj)
        % distSonar = genSonar(obj)
        % Generates a reading for all sonar sensors
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % distSonar - Vector of doubles [front left back right],
        %   distance along each line of sight to nearest obstacle
            
            % Get noise parameters
            if isfield(obj.noise,'sonar')
                noiseAvg= obj.noise.sonar(1);
                noiseStDev= obj.noise.sonar(2);
            else
                noiseAvg= 0;
                noiseStDev= 0;
            end
            
            % Cycle through all sensors
            distSonar= obj.rangeSonar*ones(1,4);    % Preallocate for speed
            for i= 1:4
                % Calculate position and orientation of the sensor
                % Assume sensors are at the edge of the robot
                th_sensor= obj.thAbs+(i-1)*pi/2;
                x_sensor= obj.posAbs(1); %+obj.radius*cos(th_sensor);
                y_sensor= obj.posAbs(2); %+obj.radius*sin(th_sensor);
                
                % Get noise value to change reading of sonar
                noiseVal= noiseAvg+noiseStDev*randn;
                
                % Solve for distance using general function
                distSonar(i)= findDist(obj,x_sensor,y_sensor,...
                    obj.rangeSonar+obj.radius,th_sensor);
                
                distSonar(i) = distSonar(i)-obj.radius; 
                % Do not add sensor noise if sensor saturated to max value
                % Or if it is less than the minimum range
                if (distSonar(i)<obj.rangeSonar)&&(distSonar(i)>obj.rangeMinSonar)
                   % Compute bounds on the sensor noise such that noise 
                   % cannot cause sensor saturation in either direction
                   % (max or min).  
                   % NOTE: This results in noise that is not purely Gaussian
                   
                   maxNoiseVal = obj.rangeSonar - distSonar(i) - 0.0001;
                   minNoiseVal = obj.rangeMinSonar - distSonar(i) + 0.0001;
                   
                   % Saturate noisey sonar measurement between these values
                   noiseVal = min(max(noiseVal,minNoiseVal),maxNoiseVal);
                   
                   distSonar(i) = distSonar(i)+noiseVal;
                end
            end
        end
        
        function distLidar= genLidar(obj)
        % distLidar = genLidar(obj)
        % Generates a reading for the LIDAR sensor
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % distLidar - Vector of doubles of length obj.numPtsLidar
        %   Distances correspond to angles [-angRangeLidar/2 angRangeLidar/2]
        %   Remember that angles are defined positive counter-clockwise
            
            % Calculate position of sensor (same for all angles)
            x_sensor= obj.posAbs(1)+obj.radius*cos(obj.thAbs);
            y_sensor= obj.posAbs(2)+obj.radius*sin(obj.thAbs);
            
            % Get noise parameters
            if isfield(obj.noise,'lidar')
                noiseAvg= obj.noise.lidar(1);
                noiseStDev= obj.noise.lidar(2);
            else
                noiseAvg= 0;
                noiseStDev= 0;
            end
            
            % Cycle through all points of sensing
            distLidar= obj.rangeLidar*ones(1,obj.numPtsLidar);
            for i= 1:obj.numPtsLidar
                % Find orientation of line of sight
                th_sensor= obj.thAbs+(i-1)*obj.angRangeLidar/...
                    (obj.numPtsLidar-1)-obj.angRangeLidar/2;
                
                % Get noise value to change reading of LIDAR
                noiseVal= noiseAvg+noiseStDev*randn;
                
                % Solve for distance using general function
                distLidar(i)= findDist(obj,x_sensor,y_sensor,...
                    obj.rangeLidar,th_sensor)+noiseVal;
                
                % Set any readings below minimum to the minimum
                distLidar(i)= max(distLidar(i),obj.rangeMinLidar);
            end
        end
        
        function [ang dist color id]= genCamera(obj)
        % [ang dist color id] = genCamera(obj)
        % Generates the output from the blob detection on the camera,
        % detects only beacons
        % Camera is located at 'cameraDisplace' distance (+0.13m) 
        % along the robot's x-axis. Same as lab setup.
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % ang - Vector of doubles, each the angle relative to robot at 
        %   which beacon is detected
        % dist - Vector of doubles, each the distance of beacon from camera
        % color - Matrix of doubles of width three (color vector), 
        %   each row the color of beacon detected
        % id  - AR tag ID
            
            % Get robot position and orientation
            x_r= obj.posAbs(1);
            y_r= obj.posAbs(2);
            th_r= obj.thAbs;
            
            % Get camera position and orientation
            x_c = x_r + obj.cameraDisplace*cos(th_r);
            y_c = y_r + obj.cameraDisplace*sin(th_r);
            th_c = th_r;
            
            % Check each beacon against camera ranges
            ang= [];
            dist= [];
            color= [];
            id = [];
            
            for i= 1:size(obj.mapBeacs,1)   % Go through all the beacons
                % Get beacon position
                x_b   = obj.mapBeacs{i,1};
                y_b   = obj.mapBeacs{i,2};
                clr_b = [obj.mapBeacs{i,3},obj.mapBeacs{i,4},obj.mapBeacs{i,5}];
                id_b  = str2num(obj.mapBeacs{i,6});
                
                % Find heading and distance from camera to beacon
                ang_b= obj.wrap2pi(atan2(y_b-y_c,x_b-x_c)-th_c);
                dist_b= sqrt((x_b-x_c)^2+(y_b-y_c)^2);
                
                % See if there is a wall in the way (blocks field-of-view)
                walldist= findDist(obj,x_c,y_c,dist_b, ...
                        obj.wrap2pi(ang_b+th_c));
                
                % If camera can see beacon, then save beacon information
                if abs(ang_b) <= obj.angRangeCamera && ...
                        dist_b <= obj.rangeCamera && ...
                        walldist == dist_b
                    ang= [ang ; ang_b];
                    dist= [dist ; dist_b];
                    color= [color ; clr_b];
                    id = [id;id_b];
                end
            end
            
            % Add noise
            if ~isempty(ang)
                if isfield(obj.noise,'camera')
                    noiseAvg= obj.noise.camera(1);
                    noiseStDev= obj.noise.camera(2);
                else
                    noiseAvg= 0;
                    noiseStDev= 0;
                end
                ang= ang+noiseAvg+noiseStDev*randn(length(ang),1);
                dist= dist+noiseAvg+noiseStDev*randn(length(ang),1);
            end
        end
        
        function dist= genOdomDist(obj)
        % dist = genOdomDist(obj)
        % Determines how far the robot has traveled since the last call
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % dist - Double, distance traveled since last call from odometry
            
            % Extract property
            % Noise is already added
            dist= obj.odomDist;
            
            % Wrap to limits
            if dist < -32.768
                dist= -32.768;
                disp('Simulator:overflow')
                disp('Return value capped at minimum')
            elseif dist > 32.768
                dist= 32.768;
                disp('Simulator:overflow')
                disp('Return value capped at maximum')
            end
            
            % Reset sensor to record distance since this call
            obj.odomDist= 0;
        end
        
        function ang= genOdomAng(obj)
        % ang = genOdomAng(obj)
        % Determines how far the robot has turned since the last call
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % ang - Double, angle turned since last call from odometry,
        %   positive counter-clockwise
            
            % Extract property
            % Noise is already added
            ang= obj.odomAng;
            
            % Wrap to limits
            if ang < -571
                ang= -571;
                disp('Simulator:overflow')
                disp('Return value capped at minimum')
            elseif ang > 571
                ang= 571;
                disp('Simulator:overflow')
                disp('Return value capped at maximum')
            end
            
            % Reset sensor to record angle since this call
            obj.odomAng= 0;
        end
        
        function updateOdom(obj,oldstate,newstate)
        % updateOdom(obj,oldstate,newstate)
        % Updates the odometry properties in the robot object
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Input:
        % oldstate - Vector of doubles [x y th v w], values of state from
        %   previous time step
        % newstate - Vector of doubles [x y th v w], values of current state
            
            % Extract values
            x_o= oldstate(1);
            y_o= oldstate(2);
            th_o= oldstate(3);
            v_o= oldstate(4:5);
            w_o= oldstate(6);
            x_n= newstate(1);
            y_n= newstate(2);
            th_n= newstate(3);
            
            % Get noise parameters
            if isfield(obj.noise,'odometry')
                noiseAvg= obj.noise.odometry(1);
                noiseStDev= obj.noise.odometry(2);
            else
                noiseAvg= 0;
                noiseStDev= 0;
            end
            
            % Distance sensor
            dist= sqrt((x_n-x_o)^2+(y_n-y_o)^2);    % Distance traveled
            dir= sign(dot(v_o,[cos(th_o) sin(th_o)]));  % For/Backwards
            noiseVal= (noiseAvg+noiseStDev*randn)*dist;
            obj.odomDist= obj.odomDist+dir*dist+noiseVal;
            
            % Angle sensor
            % Assume small turning angles, and account for cases such as 
            % turning from -pi to pi
            turn= min(abs(th_n-th_o),abs(th_n+th_o));
            dir= sign(w_o);
            noiseVal= (noiseAvg+noiseStDev*randn)*turn;
            obj.odomAng= obj.odomAng+dir*turn+noiseVal;
        end
        
        function [x y th]= genOverhead(obj)
        % [x y th] = genOverhead(obj)
        % Generate the output of the overhead localization system
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % x - x-coordinate of the robot
        % y - y-coordinate of the robot
        % th - angle of the robot relative to positive x-axis
            
            % Extract values
            x= obj.posAbs(1);
            y= obj.posAbs(2);
            th= obj.thAbs;
        end
        
    % Computational Functions
        function dist= findDist(obj,x_sensor,y_sensor,range,th)
        % dist = findDist(obj,x_sensor,y_sensor,range,th)
        % Finds the distance a sensor is measuring at a certain angle
        %
        % Input:
        % obj - Instance of class CreateRobot
        % x_sensor - X-coordinate of the sensor position
        % y_sensor - Y-coordinate of the sensor position
        % range - Linear range of the sensor
        % th - Angle the sensor is investigating in absolute coordinates
        
        % Output:
        % dist - Linear distance along the line of sight of the sensor to
        % 	the closest obstacle
                
            % Create line of sight
            x_range= x_sensor+range*cos(th);   % Range of sensor
            y_range= y_sensor+range*sin(th);
            
            % Find line equation for sensor line
            m_sensor= (y_range-y_sensor)/(x_range-x_sensor);
            if m_sensor > 1e14
                m_sensor= inf;
            elseif abs(m_sensor) < 1e-14
                m_sensor= 0;
            elseif m_sensor < -1e14
                m_sensor= -inf;
            end
            b_sensor= y_sensor-m_sensor*x_sensor;
            
            % Check against every obstacle (individually)
            j= 1;                   % Count variable for intersections
            x_int= [];              % Position of intersections
            y_int= [];
            for i= 1:size(obj.mapWalls,1)% Count variable for obstacles
                % Find line equations for wall lines
                m_wall= (obj.mapWalls(i,4)-obj.mapWalls(i,2))/...
                    (obj.mapWalls(i,3)-obj.mapWalls(i,1));
                if m_wall > 1e14
                    m_wall= inf;
                elseif abs(m_wall) < 1e-14
                    m_wall= 0;
                elseif m_wall < -1e14
                    m_wall= -inf;
                end
                b_wall= obj.mapWalls(i,2)-m_wall*obj.mapWalls(i,1);
                
                % Find intersection of infinitely long walls
                if ~(m_sensor == m_wall)    % Not parallel lines
                    if isinf(m_sensor)      % Vertical sensor line
                        x_hit= x_sensor;
                        y_hit= m_wall*x_hit+b_wall;
                    elseif isinf(m_wall)    % Vertical wall line
                        x_hit= obj.mapWalls(i,1);
                        y_hit= m_sensor*x_hit+b_sensor;
                    else                    % Normal conditions
                        x_hit= (b_wall-b_sensor)/(m_sensor-m_wall);
                        y_hit= m_sensor*x_hit+b_sensor;
                    end
                    
                    % Verify that intersection is on finite lines
                    % Use tolerances to account for rounding errors on
                    % vertical or horizontal lines
                    if x_hit-min(x_sensor,x_range) > -0.001 && ...
                            x_hit-max(x_sensor,x_range) < 0.001 && ...
                            y_hit-min(y_sensor,y_range) > -0.001 && ...
                            y_hit-max(y_sensor,y_range) < 0.001 && ...
                            x_hit-min(obj.mapWalls(i,[1 3])) > -0.001 && ...
                            x_hit-max(obj.mapWalls(i,[1 3])) < 0.001 && ...
                            y_hit-min(obj.mapWalls(i,[2 4])) > -0.001 && ...
                            y_hit-max(obj.mapWalls(i,[2 4])) < 0.001
                        x_int(j)= x_hit;
                        y_int(j)= y_hit;
                        j= j+1;
                    end
                end
            end
            
            % Find closest wall on sensor line
            dist= range;    % Initialize to max range
            if ~isempty(x_int)
                distVec= sqrt((x_int-x_sensor).^2+(y_int-y_sensor).^2);
                dist= min(distVec);  % Find shortest distance to intersections
            end
        end
        
        function collPts= findCollisions(obj)
        % collPts = findCollisions(obj)
        % Check if the robot intersects any walls
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % collPts - Matrix of doubles [x y i f]
        %   x - x-coordinate of closest point to the center of the robot
        %   y - y-coordinate of closest point to the center of the robot
        %   i - Index of wall, used as obj.mapWalls(i,:)
        %   f - Corner flag, is 1 if intersection point is a corner
        %   An empty matrix means no collisions
            
            % Extract variables
            xR= obj.posAbs(1);  % Position of the robot center
            yR= obj.posAbs(2);
            rad= obj.radius; % Radius of the robot
            
            % Find nearest point on every wall
            collPts= [];
            for i= 1:size(obj.mapWalls,1)
                % Extract wall data
                x1= obj.mapWalls(i,1);
                y1= obj.mapWalls(i,2);
                x2= obj.mapWalls(i,3);
                y2= obj.mapWalls(i,4);
                
                % Assume wall is infinitely long
                m= (y2-y1)/(x2-x1);         % Slope of wall
                if isinf(m)                 % Vertical wall
                    x0= x1;
                    y0= yR;
                elseif m == 0               % Horizontal wall
                    x0= xR;
                    y0= y1;
                else                        % Normal conditions of wall
                    b= y1-m*x1;             % Intercept of wall
                    c= yR+xR/m; % Intercept of perpendicular line through robot
                    
                    % Calculate intersection point of two lines
                    x0= (c-b)/(m+1/m);
                    y0= (b-c)/(m^2+1)+c;
                end
                
                % Check if intersection point is not on finite wall
                endPt= 0;
                if x0 > max(x1,x2) || x0 < min(x1,x2) || ...
                        y0 > max(y1,y2) || y0 < min(y1,y2)
                    % Closest point will be nearest endpoint
                    dist1= sqrt((x1-xR)^2+(y1-yR)^2);
                    dist2= sqrt((x2-xR)^2+(y2-yR)^2);
                    if dist1 <= dist2
                        x0= x1;
                        y0= y1;
                    else
                        x0= x2;
                        y0= y2;
                    end
                    endPt= 1;   % Set corner flag
                end
                
                % Check if intersection point is within robot circle
                if sqrt((x0-xR)^2+(y0-yR)^2) <= rad
                    collPts= [collPts ; x0 y0 i endPt];
                end
            end
            
            % Only keep closest two collision points
            if size(collPts,1) > 2
                [distances distIdx]= sort(sqrt((collPts(:,1)-xR).^2+...
                    (collPts(:,2)-yR).^2));
                collPts= collPts(distIdx(1:2),:);
            end
        end
        
    % State Manipulator Functions
        function state= getState(obj)
        % state = getState(obj)
        % Extracts current state properties for the simulation program
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % state - Vector of doubles [x y th v w], values of current state
            
            % Extract variables
            x= obj.posAbs(1);
            y= obj.posAbs(2);
            th= obj.thAbs;
            v= obj.velAbs;
            w= obj.wAbs;
            
            % Put in output format
            state= [x y th v w];
        end
        
        function setState(obj,state)
        % setState(obj,state)
        % Imports new state properties from the simulation program
        %
        % Input:
        % obj - Instance of class CreateRobot
        % state - Vector of doubles [x y th v w], values of new state
            
            % Update robot object
            obj.posAbs= state(1:2);
            obj.thAbs= state(3);
            obj.velAbs= state(4:5);
            obj.wAbs= state(6);
        end
        
        function driveNormal(obj,tStep)
        % driveNormal(obj,tStep)
        % Updates the new position based on the current position and
        % velocities when no walls affect the robot
        %
        % Input:
        % obj - Instance of class CreateRobot
        % tStep - Double, time since the previous state update
            
            % Get important values
            x= obj.posAbs(1);
            y= obj.posAbs(2);
            th= obj.thAbs;
            v= obj.velInt;
            w= obj.wInt;
            
            % Check zero-velocity cases to avoid inf and NaN values
            % The following code is taken from function
            % SimRobot by Jason Hardy and Francis Havlak
            if w == 0       % Straight path case
                x_new= x+v*cos(th)*tStep;
                y_new= y+v*sin(th)*tStep;
                th_new= th;
                vx= v*cos(th);
                vy= v*sin(th);
            elseif v == 0   % Turning only
                x_new= x;
                y_new= y;
                th_new= th+w*tStep;
                vx= v*cos(th);
                vy= v*sin(th);
            else            % Compute position along arc trajectory
                sign_v= sign(v);
                th_new= th+w*tStep;
                dth= th_new-th;
                dir= sign(dth);
                motionRad = v/w;
                %get relative motion
                l_chord= motionRad*2*sin(dth/2);
                y_rel= -dir*l_chord^2/2/motionRad;
                x_rel= sign_v*sqrt(l_chord^2-y_rel^2);
                %translate into global chords 
                R= [cos(th) -sin(th) ; sin(th) cos(th)];
                pos_new= [x ; y]+R*[x_rel ; y_rel];
                x_new= pos_new(1);
                y_new= pos_new(2);
                vx= v*cos(th_new);
                vy= v*sin(th_new);
            end
            % End of code from SimRobot
            
            % Update position
            th_new= obj.wrap2pi(th_new);
            obj.posAbs= [x_new y_new];
            obj.thAbs= th_new;
            obj.velAbs= [vx vy];
            obj.wAbs= w;
        end
        
        function drive1Wall(obj,tStep,collPts)
        % drive1Wall(obj,tStep,collPts)
        % Updates the new position based on the current position and
        % velocities when one wall affects the robot
        %
        % Input:
        % obj - Instance of class CreateRobot
        % tStep - Double, time since the previous state update
        % collPts - Vector of doubles [x y i f]
        %   x - x-coordinate of closest point to the center of the robot
        %   y - y-coordinate of closest point to the center of the robot
        %   i - Index of wall, used as obj.mapWalls(i,:)
        %   f - Corner flag, is 1 if intersection point is a corner
        %   To be used in this function, collPts must have exactly 1 row
            
            % Get important values
            r= obj.radius;
            x= obj.posAbs(1);
            y= obj.posAbs(2);
            th= obj.thAbs;
            v_int= obj.velInt*[cos(th) sin(th)];
            w_int= obj.wInt;
            muK= obj.frictionKin;
            i_wall= collPts(3);
            
            % Get wall data with x1 <= x2
            [x1,wall_idx]= min(obj.mapWalls(i_wall,[1 3]));
            if wall_idx == 1
                y1= obj.mapWalls(i_wall,2);
                x2= obj.mapWalls(i_wall,3);
                y2= obj.mapWalls(i_wall,4);
            else
                y1= obj.mapWalls(i_wall,4);
                x2= obj.mapWalls(i_wall,1);
                y2= obj.mapWalls(i_wall,2);
            end
            
            % Get tangential vector to the wall in the correct direction
            % That is, counter-clockwise around the robot
            if collPts(2) <= y   % Wall is beneath robot
                tV= [x2-x1 y2-y1]/sqrt((x2-x1)^2+(y2-y1)^2);
            else
                tV= [x1-x2 y1-y2]/sqrt((x1-x2)^2+(y1-y2)^2);
            end
            
            % Get normal vector from wall to robot
            nV= [x-collPts(1) y-collPts(2)]/...
                sqrt((x-collPts(1))^2+(y-collPts(2))^2);
            
            % Put intended velocity into tangential and normal directions
            v_t_int= dot(v_int,tV);
            v_n_int= dot(v_int,nV);
            
            % Compute true normal velocity
            v_n= (v_n_int > 0)*v_n_int; % Make it zero if intended normal 
                                        % velocity is negative (towards wall)
            v_n_int= (v_n_int <= 0)*v_n_int;    % Make zero or negative for
                                                % friction computation
            
%%%%%%%%%%%%% Check if robot is sliding or is in pure rolling%%%%%%%%%%
            
            % Compute angular and tangential velocity
            % Assume normal force ~ normal velocity
            if -w_int*r > v_t_int
                v_t= v_t_int+v_n_int*muK;
                w= w_int+sign(w_int)*v_n_int*muK/r;
            else
                v_t= v_t_int-v_n_int*muK;
                w= w_int-sign(w_int)*v_n_int*muK/r;
            end
            if sign(v_t_int) ~= sign(v_t)   % Motion opposite of intended
                v_t= 0;     % Friction should only resists motion
            end
            if sign(w_int) ~= sign(w)
                w= 0;       % Friction should only resist motion
            end
            
            % Compute cartesian components of velocity
            v_x= v_t*tV(1)+v_n*nV(1);
            v_y= v_t*tV(2)+v_n*nV(2);
            
            % Update position
            obj.posAbs= [x+v_x*tStep  y+v_y*tStep];
            obj.thAbs= obj.wrap2pi(th+w*tStep);
            obj.velAbs= [v_x v_y];
            obj.wAbs= w;
        end
        
        function drive2Wall(obj,tStep,collPts)
        % drive2Wall(obj,tStep,collPts)
        % Updates the new position based on the current position and
        % velocities when two walls affect the robot
        %
        % Input:
        % obj - Instance of class CreateRobot
        % tStep - Double, time since the previous state update
        % collPts - Matrix of doubles, columns [x y i f]
        %   x - x-coordinate of closest point to the center of the robot
        %   y - y-coordinate of closest point to the center of the robot
        %   i - Index of wall, used as obj.mapWalls(i,:)
        %   f - Corner flag, is 1 if intersection point is a corner
        %   To be used in this function, collPts must have exactly 2 rows
            
            % Get important values
            x= obj.posAbs(1);
            y= obj.posAbs(2);
            th= obj.thAbs;
            v_int= obj.velInt*[cos(th) sin(th)];
            w_int= obj.wInt;
            muK= obj.frictionKin;
            i_wall= collPts(:,3);
            
            % Get wall data with x1 <= x2
            x1= zeros(length(i_wall),1);    % Preallocate for speed
            y1= zeros(length(i_wall),1);
            x2= zeros(length(i_wall),1);
            y2= zeros(length(i_wall),1);
            tV= zeros(length(i_wall),2);
            nV= zeros(length(i_wall),2);
            for j= 1:length(i_wall)
                % Check that wall is not vertical
                if obj.mapWalls(i_wall(j),1) ~= obj.mapWalls(i_wall(j),3)
                    [x1(j) wall_idx]= min(obj.mapWalls(i_wall(j),[1 3]));
                    if wall_idx == 1
                        y1(j)= obj.mapWalls(i_wall(j),2);
                        x2(j)= obj.mapWalls(i_wall(j),3);
                        y2(j)= obj.mapWalls(i_wall(j),4);
                    else
                        y1(j)= obj.mapWalls(i_wall(j),4);
                        x2(j)= obj.mapWalls(i_wall(j),1);
                        y2(j)= obj.mapWalls(i_wall(j),2);
                    end
                    
                    % Get tangential vector to wall in the correct direction
                    % That is, counter-clockwise around the robot
                    if collPts(j,2) < y     % Wall is beneath robot
                        tV(j,:)= [x2(j)-x1(j) y2(j)-y1(j)]/...
                            sqrt((x2(j)-x1(j))^2+(y2(j)-y1(j))^2);
                    else
                        tV(j,:)= [x1(j)-x2(j) y1(j)-y2(j)]/...
                            sqrt((x1(j)-x2(j))^2+(y1(j)-y2(j))^2);
                    end
                else    % Vertical wall
                    [y1(j) wall_idx]= min(obj.mapWalls(i_wall(j),[2 4]));
                    if wall_idx == 1
                        x1(j)= obj.mapWalls(i_wall(j),1);
                        x2(j)= obj.mapWalls(i_wall(j),3);
                        y2(j)= obj.mapWalls(i_wall(j),4);
                    else
                        x1(j)= obj.mapWalls(i_wall(j),3);
                        x2(j)= obj.mapWalls(i_wall(j),1);
                        y2(j)= obj.mapWalls(i_wall(j),2);
                    end
                    
                    % Get tangential vector to wall in the correct direction
                    % That is, counter-clockwise around the robot
                    if collPts(j,1) < x     % Wall is left of robot
                        tV(j,:)= [x1(j)-x2(j) y1(j)-y2(j)]/...
                            sqrt((x1(j)-x2(j))^2+(y1(j)-y2(j))^2);
                    else
                        tV(j,:)= [x2(j)-x1(j) y2(j)-y1(j)]/...
                            sqrt((x2(j)-x1(j))^2+(y2(j)-y1(j))^2);
                    end
                end
                
                % Get normal vector from wall to robot
                nV(j,:)= [x-collPts(j,1) y-collPts(j,2)]/...
                    sqrt((x-collPts(j,1))^2+(y-collPts(j,2))^2);
            end
            
            % Find normal intended velocity components
            v_n1_int= dot(v_int,nV(1,:));
            v_n2_int= dot(v_int,nV(2,:));
            
            % Find if the robot is driving into the corner
            done= false;    % Signal that other sim function was called
            if dot(tV(1,:),tV(2,:)) <= 0    % Walls at acute or right angle
                if (v_n1_int <= 0 && v_n2_int <= 0) || (v_n1_int <= 0 ...
                        && xor(dot(v_int,tV(1,:)) <= 0, ...
                        dot(tV(1,:),nV(1,:)+nV(2,:)) < 0)) || ...
                        (v_n2_int <= 0 && xor(dot(v_int,tV(2,:)) <= 0, ...
                        dot(tV(2,:),nV(1,:)+nV(2,:)) < 0))
                    v= [0 0];   % Stuck in the corner
                    w= w_int+sign(w_int)*muK*(v_n1_int+v_n2_int);
                    if sign(w) ~= sign(w_int)
                        w= 0;   % Friction should only resist motion
                    end
                elseif dot(v_int,nV(1,:)) <= 0
                    collPts= collPts(1,:);  % Drive towards wall 1
                    drive1Wall(obj,tStep,collPts)
                    done= true;
                elseif dot(v_int,nV(2,:)) <= 0
                    collPts= collPts(2,:);  % Drive towards wall 2
                    drive1Wall(obj,tStep,collPts)
                    done= true;
                else                        % Drive away from walls
                    v= v_int;
                    w= w_int;
                end
            else                        % Walls at obtuse
                if v_n1_int > 0 && v_n2_int > 0 % Drive away from walls
                    v= v_int;
                    w= w_int;
                elseif sign(dot(v_int,tV(1,:))) == sign(dot(v_int,tV(2,:)))
                    if v_n1_int < v_n2_int      % Drive towards wall 1
                        collPts= collPts(1,:);
                        drive1Wall(obj,tStep,collPts)
                    else                        % Drive towards wall 2
                        collPts= collPts(2,:);
                        drive1Wall(obj,tStep,collPts)
                    end
                    done= true;
                else                            % Stuck in the corner
                    v= [0 0];   % Stuck in the corner
                    w= w_int+sign(w_int)*muK*(v_n1_int+v_n2_int);
                    if abs(w) > abs(w_int)
                        w= 0;   % Friction should only resist motion
                    end
                end
            end
            
            if ~done    % drive1Wall hasn't been called to update everything
                % Update position
                % Don't use arc calculation to avoid errors
                obj.posAbs= [x+v(1)*tStep  y+v(2)*tStep];
                obj.thAbs= obj.wrap2pi(th+w*tStep);
                obj.velAbs= v;
                obj.wAbs= w;
            end
        end
        
        function driveCorner(obj,tStep,collPts)
        % driveCorner(obj,tStep,collPts)
        % Updates the new position based on the current position and
        % velocities when one corner affects the robot
        %
        % Input:
        % obj - Instance of class CreateRobot
        % tStep - Double, time since the previous state update
        % collPts - Matrix of doubles, columns [x y i f]
        %   x - x-coordinate of closest point to the center of the robot
        %   y - y-coordinate of closest point to the center of the robot
        %   i - Index of wall, used as obj.mapWalls(i,:)
        %   f - Corner flag, is 1 if intersection point is a corner
        %   To be used in this function, collPts must have exactly 1 row
            
            % Get important values
            r= obj.radius;
            x= obj.posAbs(1);
            y= obj.posAbs(2);
            th= obj.thAbs;
            v_int= obj.velInt*[cos(th) sin(th)];
            w_int= obj.wInt;
            muK= obj.frictionKin;
            
            % Get normal vector from corner to robot
            nV= [x-collPts(1) y-collPts(2)]/...
                sqrt((x-collPts(1))^2+(y-collPts(2))^2);
            
            if dot(v_int,nV) >= 0   % Moving away from the corner
                driveNormal(obj,tStep)
            else
                % Assume rolling only motion around the corner
                tV= cross([nV 0],[0 0 1]);  % Vector tangential to path
                tV= tV(1:2);                % clockwise around corner
                a= -dot(v_int,tV)*tStep/r;  % Angle to travel
                dnV= (cos(a)-1)*r*nV;   % Movement in normal direction
                dtV= -sin(a)*r*tV;      % Movement in tangential direction
                w= w_int+sign(w_int)*dot(v_int,nV)*muK; % Angular velocity
                if sign(w) ~= sign(w_int) 
                    w= 0;           % Friction should only resist motion
                end
                
                % Update position
                obj.posAbs= [x+dnV(1)+dtV(1)  y+dnV(2)+dtV(2)];
                obj.thAbs= obj.wrap2pi(th-a+w*tStep);
                obj.velAbs= (dnV+dtV)/tStep;
                obj.wAbs= w;
            end
        end
        
    % Translator Functions
        function obj= RoombaInit(obj)
        % obj = RoombaInit(obj)
        % Outputs robot object for use in calling other functions
        % This function serves no purpose when used with only the
        % simulator, but it is necessary when controlling the real robot
        %
        % Input:
        % obj - Instance of class CreateRobot
        %   This argument must be the input argument to the control program
        %   (usually the COM port number when dealing with the real robot)
        %
        % Output:
        % obj - Instance of class CreateRobot
        %   Identical to the input argument in the simulator only
            
            % Check for valid input
            if ~isa(obj,'CreateRobot')
                error('Simulator:invalidInput',...
                    ['Input to RoombaInit must have class CreateRobot.'...
                    '  Input argument should be the input argument '...
                    'to the control program'])
            else
                try
                    % Check that autonomous is enabled and quit if not
                    autoCheck(obj)
                    
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % Add the translator function call to output data
                    fcn_called= 'RoombaInit';
                    addFcnToOutput(obj,fcn_called)
                    
                    % Display messages that toolbox runs
                    disp('Establishing connection to Roomba...')
                    disp('Opening connection to Roomba...')
                    disp('Setting Roomba to Control Mode...')
                    disp('I am alive if my two outboard lights came on')
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError:')
                        disp(['Error in function RoombaInit.  Caused '...
                            'by standard simulator function calls.'])
                    end
                    rethrow(me)
                end
            end
        end
        
        function [BumpRight BumpLeft BumpFront Wall virtWall CliffLft ...
                CliffRgt CliffFrntLft CliffFrntRgt LeftCurrOver ...
                RightCurrOver DirtL DirtR ButtonPlay ButtonAdv Dist ...
                Angle Volts Current Temp Charge Capacity pCharge]= ...
                AllSensorsReadRoomba(obj)
        % [BumpRight BumpLeft BumpFront Wall virtWall CliffLft 
        %   CliffRgt CliffFrntLft CliffFrntRgt LeftCurrOver ...
        %   RightCurrOver DirtL DirtR ButtonPlay ButtonAdv Dist ...
        %   Angle Volts Current Temp Charge Capacity pCharge] = ...
        %   AllSensorsReadRoomba(obj)
        % Read most of the sensors on the robot at once
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % BumpRight - Boolean double, 1 if right bump sensor is triggered
        % BumpLeft - Boolean double, 1 if left bump sensor is triggered
        % BumpFront - Boolean double, 1 if both left and right bump sensors
        %   are triggered, BumpLeft and BumpRight will return 0 for this
        % Wall - Boolean double, 1 is a wall is within range of the wall
        %   sensor
        % virtWall - Boolean double, 1 if a virtual wall is detected
        % CliffLft - Boolean double, 1 if a cliff or line is detected on
        %   left
        % CliffRgt - Boolean double, 1 if a cliff or line is detected on
        %   right
        % CliffFrntLft - Boolean double, 1 if a cliff or line is detected 
        %   on the front left
        % CliffFrntRgt - Boolean double, 1 if a cliff or line is detected
        %   on the front right
        % LeftCurrOver - Boolean double, 1 if left wheel receives over 
        %   1.0 A
        % RightCurrOver - Boolean double, 1 if right wheel receives over 
        %   1.0 A
        % DirtL - Boolean double, 1 if dirt is detected on the left
        % DirtR - Boolean double, 1 if dirt is detected on the right
        % ButtonPlay - Boolean double, 1 if Play button is currently 
        %   pressed
        % ButtonAdv - Boolean double, 1 if Advance button is currently
        %   pressed
        % Dist - Double, distance traveled since last call from odometry (m)
        % Angle - Double, angle turned since last call from odometry (rad)
        % Volts - Double, voltage of battery (V)
        % Current - Double, current flowing in or out of battery (A)
        % Temp - Double, temperature of battery (degrees C)
        % Charge - Double, charge remaining in battery (mA-hours)
        % Capacity - Double, total possible charge of battery (mA-hours)
        % pCharge - Double, charge in battery as percentage of the capacity (%)

            % Check for valid input
            if ~isa(obj,'CreateRobot')
                error('Simulator:invalidInput',...
                    ['Input to AllSensorsReadRoomba must have class '...
                    'CreateRobot.  Input argument should be the input '...
                    'argument to the control program'])
            else
                try
                    % Check that autonomous is enabled and quit if not
                    autoCheck(obj)

                    % Pause for communication delay
                    pause(obj.comDelay)

                    % Read bump sensors
                    bump= genBump(obj);
                    BumpRight= bump(1);
                    BumpFront= bump(2);
                    BumpLeft= bump(3);

                    % Read infrared wall sensor
                    Wall= genIR(obj);

                    % Read cliff sensors
                    cliff= genCliff(obj);
                    CliffRgt= cliff(1);
                    CliffFrntRgt= cliff(2);
                    CliffFrntLft= cliff(3);
                    CliffLft= cliff(4);
                    
                    % Read the state of the buttons
                    [ButtonAdv ButtonPlay]= ButtonsSensorRoomba(obj);

                    % Read odometry
                    Dist= DistanceSensorRoomba(obj);
                    Angle= AngleSensorRoomba(obj);

                    % Read IR receiver for virtual wall signal
                    virtWall= genVWall(obj);

                    % Unused in simulation
                    LeftCurrOver= 0;    % Assume no overcurrent
                    RightCurrOver= 0;
                    DirtL= 0;           % Dirt sensors always 0
                    DirtR= 0;
                    Volts= BatteryVoltageRoomba(obj);
                    Current= CurrentTesterRoomba(obj);
                    Temp= 25;               % Assume room temperature
                    [Charge Capacity pCharge]= ...
                        BatteryChargeReaderRoomba(obj);

                    % Add the translator function call to output data
                    fcn_called= sprintf(...
                        ['[%.0f %.0f %.0f %.0f %.0f %.0f %.0f %.0f '...
                        '%.0f %.0f %.0f %.0f %.0f %.0f %.0f %.3f %.3f '...
                        '%.3f %.3f %.3f %.3f %.3f %.3f]= '...
                        'AllSensorsReadRoomba'],BumpRight,BumpLeft,...
                        BumpFront,Wall,virtWall,CliffLft,CliffRgt,...
                        CliffFrntLft,CliffFrntRgt,LeftCurrOver,...
                        RightCurrOver,DirtL,DirtR,ButtonPlay,ButtonAdv,...
                        Dist,Angle,Volts,Current,Temp,Charge,Capacity,...
                        pCharge);
                    addFcnToOutput(obj,fcn_called)
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simualtor:unknownError')
                        disp('Error in function AllSensorsReadRoomba.')
                    end
                    rethrow(me)
                end
            end
        end
        
        function AngleR= AngleSensorRoomba(obj)
        % AngleR = AngleSensorRobot(obj)
        % Determines how far the robot has turned since the last call
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % AngleR - Double, angle turned since last call from odometry,
        %   positive counter-clockwise (rad)
            
            % Check for valid input
            if ~isa(obj,'CreateRobot')
                error('Simulator:invalidInput',...
                    ['Input to AngleSensorRoomba must have class '...
                    'CreateRobot.  Input argument should be the input '...
                    'argument to the control program'])
            else
                try
                    % Check that autonomous is enabled and quit if not
                    autoCheck(obj)
                    
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % Generate a reading for the odometry and reset it
                    AngleR= genOdomAng(obj);
                    
                    % Add the translator function call to output data
                    fcn_called= sprintf('%.3f= AngleSensorRoomba',AngleR);
                    addFcnToOutput(obj,fcn_called)
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp('Error in function AngleSensorRoomba.')
                    end
                    rethrow(me)
                end
            end
        end
        
        function [Charge Capacity Percent]= BatteryChargeReaderRoomba(obj)
        % [Charge Capacity Percent] = BatteryChargeReaderRoomba(obj)
        % Reads the charge remaining in the battery
        % This is not important to the functionality of the simulator
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % Charge - Double, charge remaining in battery (mA-hours)
        % Capacity - Double, total possible charge of battery (mA-hours)
        % Percent - Double, charge in battery as percentage of the capacity (%)
            
            % Check for valid input
            if ~isa(obj,'CreateRobot')
                error('Simulator:invalidInput',...
                    ['Input to BatteryChargeReaderRoomba must have class '...
                    'CreateRobot.  Input argument should be the input '...
                    'argument to the control program'])
            else
                try
                    % Check that autonomous is enabled and quit if not
                    autoCheck(obj)
                    
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % Return a fully charged battery for simulator
                    Charge= 3000;
                    Capacity= 3000;
                    Percent= 100*Charge/Capacity;
                    
                    % Add the translator function call to output data
                    fcn_called= sprintf(...
                        '[%.3f %.3f %.3f]= BatteryChargeReaderRoomba',...
                        Charge,Capacity,Percent);
                    addFcnToOutput(obj,fcn_called)
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp('Error in function BatteryChargeReaderRoomba.')
                    end
                    rethrow(me)
                end
            end
        end
        
        function Voltage= BatteryVoltageRoomba(obj)
        % Voltage = BatteryVoltageRoomba(obj)
        % Reads the voltage of the battery
        % This is not important to the functionality of the simulator
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % Voltage - Double, voltage of the battery (V)
            
            % Check for valid input
            if ~isa(obj,'CreateRobot')
                error('Simulator:invalidInput',...
                    ['Input to BatteryVoltageRoomba must have class '...
                    'CreateRobot.  Input argument should be the input '...
                    'argument to the control program'])
            else
                try
                    % Check that autonomous is enabled and quit if not
                    autoCheck(obj)
                    
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % Return a fully charged battery for simulator
                    Voltage= 17.2;
                    
                    % Add the translator function call to output data
                    fcn_called= sprintf('%.3f= BatteryVoltageRoomba',...
                        Voltage);
                    addFcnToOutput(obj,fcn_called)
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp('Error in function BatteryVoltageRoomba.')
                    end
                    rethrow(me)
                end
            end
        end
        
        function [BumpRight BumpLeft WheDropRight WheDropLeft ...
                WheDropCaster BumpFront] = BumpsWheelDropsSensorsRoomba(obj)
        %[BumpRight BumpLeft WheDropRight WheDropLeft WheDropCaster 
        %   BumpFront] = BumpsWheelDropsSensorsRoomba(obj)
        % Reads status of bump and wheel drop sensors
        % The wheel drop sensors are not important to the simulator since
        % there is no functionality for cliffs
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % BumpRight - Boolean double, 1 if right bump sensor is triggered
        % BumpLeft - Boolean double, 1 if left bump sensor is triggered
        % WheDropRight - Boolean double, 1 if right wheel is released and
        %   in an extended position
        % WheDropLeft - Boolean double, 1 if left wheel is released and
        %   in an extended position
        % WheDropCaster - Boolean double, 1 if front wheel is released and
        %   in an extended position
        % BumpFront - Boolean double, 1 if both left and right bump sensors
        %   are triggered, BumpLeft and BumpRight will return 0 for this
            
            % Check for valid input
            if ~isa(obj,'CreateRobot')
                error('Simulator:invalidInput',...
                    ['Input to BumpsWheelDropsSensorsRoomba must have '...
                    'class CreateRobot.  Input argument should be the '...
                    'input argument to the control program'])
            else
                try
                    % Check that autonomous is enabled and quit if not
                    autoCheck(obj)
                    
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % Read bump sensors
                    bump= genBump(obj);
                    BumpRight= bump(1);
                    BumpFront= bump(2);
                    BumpLeft= bump(3);
                    
                    % Assume retracted wheels for simulation purposes
                    WheDropRight= 0;
                    WheDropLeft= 0;
                    WheDropCaster= 0;
                    
                    % Add the translator function call to output data
                    fcn_called= sprintf(['[%.0f %.0f %.0f %.0f %.0f '...
                        '%.0f]= BumpsWheelDropsSensorsRoomba'],...
                        BumpRight,BumpLeft,WheDropRight,WheDropLeft,...
                        WheDropCaster,BumpFront);
                    addFcnToOutput(obj,fcn_called)
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp(['Error in function '...
                            'BumpsWheelDropsSensorsRoomba.'])
                    end
                    rethrow(me)
                end
            end
        end
        
        function [ButtonAdv ButtonPlay]= ButtonsSensorRoomba(obj)
        % [ButtonAdv ButtonPlay] = ButtonsSensorRoomba(obj)
        % Reads state of the buttons
        % Note that the buttons are toggle buttons in the simulator, unlike
        %   on the real Create where the buttons must be held down
        % This function will display useful output only when the robot 
        %   object has been created by SimulatorGUI (and was passed the 
        %   handles structure)
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % ButtonAdv - Boolean double, 1 if Advance button is currently
        %   pressed
        % ButtonPlay - Boolean double, 1 if Play button is currently
        %   pressed
            
            % Check for valid input
            if ~isa(obj,'CreateRobot')
                error('Simulator:invalidInput',...
                    ['Input to ButtonsSensorRoomba must have '...
                    'class CreateRobot.  Input argument should be the '...
                    'input argument to the control program'])
            else
                try
                    % Check that autonomous is enabled and quit if not
                    autoCheck(obj)
                    
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % Check values if called from simulator
                    if ~isempty(obj.handlesGUI)
                        ButtonAdv= get(obj.handlesGUI.push_adv,'Value');
                        ButtonPlay= get(obj.handlesGUI.push_play,'Value');
                    else    % Not called from simulator so assume 0
                        ButtonAdv= 0;
                        ButtonPlay= 0;
                    end
                    
                    % Add the translator function call to output data
                    fcn_called= sprintf('[%.0f %.0f]= ButtonsSensorRoomba'...
                        ,ButtonAdv,ButtonPlay);
                    addFcnToOutput(obj,fcn_called)
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp('Error in function ButtonsSensorRoomba.')
                    end
                    rethrow(me)
                end
            end
        end
        
        function state= CliffFrontLeftSensorRoomba(obj)
        % state = CliffFrontLeftSensorRoomba(obj)
        % Reads state of the front left cliff sensor
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % state - Boolean double, 1 if sensor is over line or cliff
            
            % Check for valid input
            if ~isa(obj,'CreateRobot')
                error('Simulator:invalidInput',...
                    ['Input to CliffFrontLeftSensorRoomba must have '...
                    'class CreateRobot.  Input argument should be the '...
                    'input argument to the control program'])
            else
                try
                    % Check that autonomous is enabled and quit if not
                    autoCheck(obj)
                    
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % State is 0 since no cliffs in the simulator
                    state= 0;
                    
                    % Add the translator function call to output data
                    fcn_called= sprintf(...
                        '%.0f= CliffFrontLeftSensorRoomba',state);
                    addFcnToOutput(obj,fcn_called)
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp('Error in function CliffFrontLeftSensorRoomba.')
                    end
                    rethrow(me)
                end
            end
        end
        
        function state= CliffFrontRightSensorRoomba(obj)
        % state = CliffFrontRightSensorRoomba(obj)
        % Reads state of the front right cliff sensor
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % state - Boolean double, 1 if sensor is over line or cliff
            
            % Check for valid input
            if ~isa(obj,'CreateRobot')
                error('Simulator:invalidInput',...
                    ['Input to CliffFrontRightSensorRoomba must have '...
                    'class CreateRobot.  Input argument should be the '...
                    'input argument to the control program'])
            else
                try
                    % Check that autonomous is enabled and quit if not
                    autoCheck(obj)
                    
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % State is 0 since no cliffs in the simulator
                    state= 0;
                    
                    % Add the translator function call to output data
                    fcn_called= sprintf(...
                        '%.0f= CliffFrontRightSensorRoomba',state);
                    addFcnToOutput(obj,fcn_called)
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp('Error in function CliffFrontRightSensorRoomba.')
                    end
                    rethrow(me)
                end
            end
        end
        
        function state= CliffLeftSensorRoomba(obj)
        % state = CliffLeftSensorRoomba(obj)
        % Reads state of the left cliff sensor
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % state - Boolean double, 1 if sensor is over line or cliff
            
            % Check for valid input
            if ~isa(obj,'CreateRobot')
                error('Simulator:invalidInput',...
                    ['Input to CliffLeftSensorRoomba must have '...
                    'class CreateRobot.  Input argument should be the '...
                    'input argument to the control program'])
            else
                try
                    % Check that autonomous is enabled and quit if not
                    autoCheck(obj)
                    
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % State is 0 since no cliffs in the simulator
                    state= 0;
                    
                    % Add the translator function call to output data
                    fcn_called= sprintf('%.0f= CliffLeftSensorRoomba',...
                        state);
                    addFcnToOutput(obj,fcn_called)
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp('Error in function CliffLeftSensorRoomba.')
                    end
                    rethrow(me)
                end
            end
        end
        
        function state= CliffRightSensorRoomba(obj)
        % state = CliffRightSensorRoomba(obj)
        % Reads state of the right cliff sensor
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % state - Boolean double, 1 if sensor is over line or cliff
            
            % Check for valid input
            if ~isa(obj,'CreateRobot')
                error('Simulator:invalidInput',...
                    ['Input to CliffRightSensorRoomba must have '...
                    'class CreateRobot.  Input argument should be the '...
                    'input argument to the control program'])
            else
                try
                    % Check that autonomous is enabled and quit if not
                    autoCheck(obj)
                    
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % State is 0 since no cliffs in the simulator
                    state= 0;
                    
                    % Add the translator function call to output data
                    fcn_called= sprintf('%.0f= CliffRightSensorRoomba',...
                        state);
                    addFcnToOutput(obj,fcn_called)
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp('Error in function CliffRightSensorRoomba.')
                    end
                    rethrow(me)
                end
            end
        end
        
        function strg= CliffFrontLeftSignalStrengthRoomba(obj)
        % strg = CliffFrontLeftSignalStrengthRoomba(obj)
        % Reads strength of the front left cliff sensor signal
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % strg - uint16, lower value if sensor is over line or cliff (%)
            
            % Check for valid input
            if ~isa(obj,'CreateRobot')
                error('Simulator:invalidInput',...
                    ['Input to CliffFrontLeftSignalStrengthRoomba must '...
                    'have class CreateRobot.  Input argument should be '...
                    'the input argument to the control program'])
            else
                try
                    % Check that autonomous is enabled and quit if not
                    autoCheck(obj)
                    
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % Signal strength based on if a line is there or not
                    cliff= genCliff(obj);
                    strg= cliff(3);
                    
                    % Add the translator function call to output data
                    fcn_called= sprintf(...
                        '%.3f= CliffFrontLeftSignalStrengthRoomba',strg);
                    addFcnToOutput(obj,fcn_called)
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp(['Error in function '...
                            'CliffFrontLeftSignalStrengthRoomba.'])
                    end
                    rethrow(me)
                end
            end
        end
        
        function strg= CliffFrontRightSignalStrengthRoomba(obj)
        % strg = CliffFrontRightSignalStrengthRoomba(obj)
        % Reads strength of the front right cliff sensor signal
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % strg - uint16, lower value if sensor is over line or cliff (%)
            
            % Check for valid input
            if ~isa(obj,'CreateRobot')
                error('Simulator:invalidInput',...
                    ['Input to CliffFrontRightSignalStrengthRoomba must '...
                    'have class CreateRobot.  Input argument should be '...
                    'the input argument to the control program'])
            else
                try
                    % Check that autonomous is enabled and quit if not
                    autoCheck(obj)
                    
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % Signal strength based on if a line is there or not
                    cliff= genCliff(obj);
                    strg= cliff(2);
                    
                    % Add the translator function call to output data
                    fcn_called= sprintf(...
                        '%.3f= CliffFrontRightSignalStrengthRoomba',strg);
                    addFcnToOutput(obj,fcn_called)
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp(['Error in function '...
                            'CliffFrontRightSignalStrengthRoomba'])
                    end
                    rethrow(me)
                end
            end
        end
        
        function strg= CliffLeftSignalStrengthRoomba(obj)
        % strg = CliffLeftSignalStrengthRoomba(obj)
        % Reads strength of the left cliff sensor signal
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % strg - uint16, lower value if sensor is over line or cliff (%)
            
            % Check for valid input
            if ~isa(obj,'CreateRobot')
                error('Simulator:invalidInput',...
                    ['Input to CliffLeftSignalStrengthRoomba must '...
                    'have class CreateRobot.  Input argument should be '...
                    'the input argument to the control program'])
            else
                try
                    % Check that autonomous is enabled and quit if not
                    autoCheck(obj)
                    
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % Signal strength based on if a line is there or not
                    cliff= genCliff(obj);
                    strg= cliff(4);
                    
                    % Add the translator function call to output data
                    fcn_called= ...
                        sprintf('%.3f= CliffLeftSignalStrengthRoomba',strg);
                    addFcnToOutput(obj,fcn_called)
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownFunction')
                        disp(['Error in function '...
                            'CliffLeftSignalStrengthRoomba.'])
                    end
                    rethrow(me)
                end
            end
        end
        
        function strg= CliffRightSignalStrengthRoomba(obj)
        % strg = CliffRightSignalStrengthRoomba(obj)
        % Reads strength of the right cliff sensor signal
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % strg - uint16, lower value if sensor is over line or cliff (%)
            
            % Check for valid input
            if ~isa(obj,'CreateRobot')
                error('Simulator:invalidInput',...
                    ['Input to CliffRightSignalStrengthRoomba must '...
                    'have class CreateRobot.  Input argument should be '...
                    'the input argument to the control program'])
            else
                try
                    % Check that autonomous is enabled and quit if not
                    autoCheck(obj)
                    
                    pause(obj.comDelay)
                    
                    % Signal strength based on if a line is there or not
                    cliff= genCliff(obj);
                    strg= cliff(1);
                    
                    % Add the translator function call to output data
                    fcn_called= sprintf(...
                        '%.3f= CliffRightSignalStrengthRoomba',strg);
                    addFcnToOutput(obj,fcn_called)
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp(['Error in function '...
                            'CliffRightSignalStrengthRoomba.'])
                    end
                    rethrow(me)
                end
            end
        end
        
        function Current= CurrentTesterRoomba(obj)
        % Current = CurrentTesterRoomba(obj)
        % Reads current flowing in or out of the battery
        % This is not important to the functionality of the simulator
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % Current - Double, positive current signifies a charging battery,
        %   negative current is a discharging battery (A)
            
            % Check for valid input
            if ~isa(obj,'CreateRobot')
                error('Simulator:invalidInput',...
                    ['Input to CurrentTesterRoomba must '...
                    'have class CreateRobot.  Input argument should be '...
                    'the input argument to the control program'])
            else
                try
                    % Check that autonomous is enabled and quit if not
                    autoCheck(obj)
                    
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % Current will always be negative for simulator
                    Current= int16(-3);
                    % 3 Amps is an estimate of a common battery current
                    
                    % Add the translator function call to output data
                    fcn_called= sprintf('%.3f= CurrentTesterRoomba',...
                        Current);
                    addFcnToOutput(obj,fcn_called)
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp('Error in function CurrentTesterRoomba.')
                    end
                    rethrow(me)
                end
            end
        end
        
        function DemoCmdsCreate(obj,DemoNum)
        % DemoCmdsCreate(obj,DemoNum)
        % Performs the specified demo with the robot
        % This is not important to the functionality of the simulator
        %
        % Input:
        % obj - Instance of class CreateRobot
        % DemoNum - Double, corresponds to the demo to be run
        %   -1 - Abort current demo
        %   0 - Cover a room using a combination of behaviors
        %   1 - Cover a room unless signal from Home Base is found
        %       Home base will not be found in simulation
        %   2 - Spirals outward then inward
        %   3 - Searches for a wall, then drives along the wall
        %   4 - Drives in a figure-8
        %   5 - Drives forward when pushed while avoiding obstacles
        %       Cannot be pushed in simulation
        %   6 - Drives towards a virtual wall
        %   7 - Drives between virtual walls
        %   8 - Plays Pachelbel's Cannon when cliff sensors are triggered
        %   9 - Plays a different note for each cliff and bump sensor
            
            % Check for valid input
            if ~isa(obj,'CreateRobot')
                error('Simulator:invalidInput',...
                    ['The first input argument to DemoCmdsCreate must '...
                    'have class CreateRobot.  Input argument should be '...
                    'the input argument to the control program'])
            elseif ~isnumeric(DemoNum) || isempty(DemoNum)
                error('Simulator:invalidInput',...
                    ['The second input argument to DemoCmdsCreate must '...
                    'be numeric.  The number must be in the range [-1 9]'])
            elseif DemoNum < -1 || DemoNum > 9
                error('Simulator:invalidInput',...
                    ['The second input argument to DemoCmdsCreate must '...
                    'be between -1 and 9 to select valid demos'])
            else
                try
                    % Check that autonomous is enabled and quit if not
                    autoCheck(obj)
                    
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % Add the translator function call to output data
                    fcn_called= sprintf('DemoCmdsCreate(%.0f)',DemoNum);
                    addFcnToOutput(obj,fcn_called)
                    
                    % Placeholder until demo is programmed
                    fprintf('Demo number %.0f selected.\n',DemoNum)
                    disp(['WARNING:Currently no simulator demo '...
                        'functionality.'])
                    %disp('Performing Demo')
                    
                    % Use sound or soundsc to play music
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp('Error in function DemoCmdsCreate.')
                    end
                    rethrow(me)
                end
            end
        end
        
        function Distance= DistanceSensorRoomba(obj)
        % Distance = DistanceSensorRoomba(obj)
        % Determines how far the robot has traveled since the last call
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % Distance - Double, distance traveled since last call from odometry (m)
            
            % Check for valid input
            if ~isa(obj,'CreateRobot')
                error('Simulator:invalidInput',...
                    ['Input to DistanceSensorRoomba must '...
                    'have class CreateRobot.  Input argument should be '...
                    'the input argument to the control program'])
            else
                try
                    % Check that autonomous is enabled and quit if not
                    autoCheck(obj)
                    
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % Generate a reading for the odometry and reset it
                    Distance= genOdomDist(obj);
                    
                    % Display message as the toolbox would
                    if Distance > 32 || Distance < -32
                        disp('Warning:  May have overflowed')
                    end
                    
                    % Add the translator function call to output data
                    fcn_called= sprintf('%.3f= DistanceSensorRoomba',...
                        Distance);
                    addFcnToOutput(obj,fcn_called)
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp('Error in function DistanceSensorRoomba.')
                    end
                    rethrow(me)
                end
            end
        end
        
        function BeepRoomba(obj)
        % BeepRoomba(obj)
        % Cause the robot to beep
        % This is not important to the functionality of the simulator
        %
        % Input:
        % obj - Instance of class CreateRobot
            
            % Check for valid input
            if ~isa(obj,'CreateRobot')
                error('Simulator:invalidInput',...
                    ['Input to BeepRoomba must '...
                    'have class CreateRobot.  Input argument should be '...
                    'the input argument to the control program'])
            else
                try
                    % Check that autonomous is enabled and quit if not
                    autoCheck(obj)
                    
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % Use the built in function to call the default beep
                    beep on
                    beep
                    
                    % Add the translator function call to output data
                    fcn_called= 'BeepRoomba';
                    addFcnToOutput(obj,fcn_called)
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simualtor:unknownError')
                        disp('Error in function BeepRoomba.')
                    end
                    rethrow(me)
                end
            end
        end
        
        function SetLEDsRoomba(obj,LED,pColor,pIntensity)
        % SetLEDsRoomba(obj,LED,pColor,pIntensity)
        % Control the lighting of the LEDs on the robot
        % If SimulatorGUI set up the robot object and passed it the handles
        %   structure, then this function will change the color of the LED
        %   representations in the GUI, otherwise it will print the change
        %   to the command window
        %
        % Input:
        % obj - Instance of class CreateRobot
        % LED - Double, controls states of the Play and Advance LEDs
        %   0 - Both off
        %   1 - Advance on
        %   2 - Play on
        %   3 - Both on
        %   Allowable values 0, 1, 2, or 3
        % pColor - Double, controls color of the Power LED (%)
        %   0 - Pure green
        %   100 - Pure red
        %   Allowable values [0 100]
        % pIntensity - Double, controls brightness of the Power LED (%)
        %   Allowable values [1 100]
            
            % Check for valid input
            if ~isa(obj,'CreateRobot')
                error('Simulator:invalidInput',...
                    ['The first input argument to SetLEDsRoomba must '...
                    'have class CreateRobot.  Input argument should be '...
                    'the input argument to the control program'])
            elseif ~isnumeric(LED) || isempty(LED)
                error('Simulator:invalidInput',...
                    ['The second input argument to SetLEDsRoomba must '...
                    'be numeric; one of 0, 1, 2, or 3'])
            elseif ~any(LED == [0 1 2 3])
                error('Simulator:invalidInput',...
                    ['The second input argument to SetLEDsRoomba must '...
                    'be one of 0, 1, 2, or 3'])
            elseif ~isnumeric(pColor) || isempty(pColor)
                error('Simulator:invalidInput',...
                    ['The third input argument to SetLEDsRoomba must '...
                    'be numeric, in the range [0 100]'])
            elseif pColor < 0 || pColor > 100
                error('Simulator:invalidInput',...
                    ['The third input argument to SetLEDsRoomba must '...
                    'be between 0 and 100'])
            elseif ~isnumeric(pIntensity) || isempty(pIntensity)
                error('Simulator:invalidInput',...
                    ['The fourth input argument to SetLEDsRoomba must '...
                    'be numeric, in the range [1 100]'])
            elseif pIntensity < 1 || pIntensity > 100
                error('Simulator:invalidInput',...
                    ['The fourth input argument to SetLEDsRoomba must '...
                    'be between 1 and 100'])
            else
                try
                    % Check that autonomous is enabled and quit if not
                    autoCheck(obj)
                    
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % Add the translator function call to output data
                    fcn_called= sprintf('SetLEDsRoomba(%.0f,%.0f,%.0f)',...
                        LED,pColor,pIntensity);
                    addFcnToOutput(obj,fcn_called)
                    
                    % Convert power LED input from percent to decimal
                    pColor= pColor/100;
                    pIntensity= pIntensity/100;
                    
                    % Set LED representations in simulator window
                    disp('LEDs Changing')
                    if ~isempty(obj.handlesGUI)
                        if LED == 0
                            play= 0;
                            adv= 0;
                        elseif LED == 1
                            play= 0;
                            adv= 1;
                        elseif LED == 2
                            play= 1;
                            adv= 0;
                        elseif LED == 3
                            play= 1;
                            adv= 1;
                        end
                        set(obj.handlesGUI.text_play,'BackgroundColor',...
                            play*[0 1 0])
                        set(obj.handlesGUI.text_adv,'BackgroundColor',...
                            adv*[0 1 0])
                        set(obj.handlesGUI.text_power,'BackgroundColor',...
                            pIntensity*[pColor 1-pColor 0])
                    else    % No simulator so just display messages
                        disp('LEDs Changing')
                        if LED == 0
                            disp('Play Off, Advance Off')
                        elseif LED == 1
                            disp('Play Off, Advance On')
                        elseif LED == 2
                            disp('Play On, Advance Off')
                        elseif LED == 3
                            disp('Play On, Advance On')
                        end
                        fprintf('Power LED set to color: [%d %d 0]\n',...
                            pColor,100-pColor)
                        fprintf('Power LED set to intensity: %d\n',...
                            pIntensity)
                    end
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp('Error in function SetLEDsRoomba.')
                    end
                    rethrow(me)
                end
            end
        end
        
        function SetFwdVelRadiusRoomba(obj,FwdVel,Radius)
        % SetFwdVelRadiusRoomba(obj,FwdVel,Radius)
        % Controls the forward velocity and turning radius of the robot
        %
        % Input:
        % obj - Instance of class CreateRobot
        % FwdVel - Double, forward velocity in range [-0.5 0.5] (m/s)
        % Radius - Double, turning radius in range [-2 -eps],[eps 2] (m)
        %   Turning is positive counter-clockwise, special cases are:
        %   inf - Essentially straight movement, exception to range rule
        %   eps - zero-point turn counter-clockwise
        %   -eps - zero-point turn clockwise
            
            % Check for valid input
            if ~isa(obj,'CreateRobot')
                error('Simulator:invalidInput',...
                    ['The first input argument to SetFwdVelRadiusRoomba '...
                    'must have class CreateRobot.  Input argument '...
                    'should be the input argument to the control program'])
            elseif ~isnumeric(FwdVel) || isempty(FwdVel)
                error('Simulator:invalidInput',...
                    ['The second input argument to SetFwdVelRadiusRoomba '...
                    'must be numeric, in the range [-0.5 0.5]'])
            elseif ~isnumeric(Radius) || isempty(Radius)
                error('Simulator:invalidInput',...
                    ['The third input argument to SetFwdVelRadiusRoomba '...
                    'must be numeric, in the range [-2 2]'])
            else
                try
                    % Check that autonomous is enabled and quit if not
                    autoCheck(obj)
                    
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % Add the translator function call to output data
                    fcn_called= sprintf('SetFwdVelRadiusRoomba(%.3f,%.3f)',...
                        FwdVel,Radius);
                    addFcnToOutput(obj,fcn_called)
                    
                    % Check for input within range
                    if abs(FwdVel) > 0.5
                        FwdVel= sign(FwdVel)*0.5;
                        disp('Simulator:invalidInput')
                        disp(['Input value FwdVel for function '...
                            'SetFwdVelRadiusRoomba must be in range '...
                            '[-0.5 0.5]'])
                    end
                    
                    % Adjust turning radius to robot parameters
                    if isinf(Radius)
                        Radius= 32.768;
                    elseif Radius == eps
                        Radius= 0.001;
                    elseif Radius == -eps
                        Radius= -0.001;
                    elseif abs(Radius) > 2
                        Radius= sign(Radius)*2;
                        disp('Simulator:invalidInput')
                        disp(['Input value Radius for function '...
                            'SetFwdVelRadiusRoomba must be in range '...
                            '[-2 2]'])
                    elseif Radius == 0
                        Radius= 0.001;
                        %%%% What is the actual behavior?? %%%%
                        disp('Simulator:invalidInput')
                        disp(['Input value Radius for function '...
                            'SetFwdVelRadiusRoomba should not be zero'])
                        disp('Use eps or -eps to specify turning direction')
                    end
                    AngVel= FwdVel/Radius;
                    
                    % Limit individual wheel velocities
                    wheelRight= FwdVel+AngVel*obj.wheelbase/2;
                    wheelLeft= FwdVel-AngVel*obj.wheelbase/2;
                    if abs(wheelRight) > 0.5 || abs(wheelLeft) > 0.5
                        wheelRight= min(max(wheelRight,-0.5),0.5);
                        wheelLeft= min(max(wheelRight,-0.5),0.5);
                        disp('Simulator:invalidInput')
                        disp(['Each wheel can only move at 0.5 m/s. '...
                            'Choose a velocity combination that does '...
                            'not exceed these limits. Values set to max.'])
                        FwdVel= (wheelRight+wheelLeft)/2;
                        AngVel= (wheelRight-wheelLeft)/obj.wheelbase;
                    end
                    %%%% What is the actual behavior?? %%%%
                    
                    % Change object parameters to new values
                    obj.velInt= FwdVel;
                    obj.wInt= AngVel;
                    disp('moving!')
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp('Error in function SetFwdVelRadiusRoomba.')
                    end
                    rethrow(me)
                end
            end
        end
        
        function SetDriveWheelsCreate(obj,rightWheel,leftWheel)
        % SetDriveWheelsCreate(obj,rightWheel,leftWheel)
        % Controls the forward velocity of each wheel individually
        %
        % Input:
        % obj - Instance of class CreateRobot
        % rightWheel - Double, velocity of right wheel
        % leftWheel - Double, velocity of left wheel
        %   Both values in range [-0.5 0.5] (m/s)
            
            % Check for valid input
            if ~isa(obj,'CreateRobot')
                error('Simulator:invalidInput',...
                    ['The first input argument to SetDriveWheelsCreate '...
                    'must have class CreateRobot.  Input argument '...
                    'should be the input argument to the control program'])
            elseif ~isnumeric(rightWheel) || isempty(rightWheel)
                error('Simulator:invalidInput',...
                    ['The second input argument to SetDriveWheelsCreate '...
                    'must be numeric in the range [-0.5 0.5]'])
            elseif ~isnumeric(leftWheel) || isempty(leftWheel)
                error('Simulator:invalidInput',...
                    ['The third input argument to SetDriveWheelsCreate '...
                    'must be numeric in the range [-0.5 0.5]'])
            else
                try
                    % Check that autonomous is enabled and quit if not
                    autoCheck(obj)
                    
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % Add the translator function call to output data
                    fcn_called= sprintf('SetDriveWheelsCreate(%.3f,%.3f)',...
                        rightWheel,leftWheel);
                    addFcnToOutput(obj,fcn_called)
                    
                    % Check for input within range
                    if rightWheel < -0.5
                        rightWheel= -0.5;
                       disp('Simulator:invalidInput')
                       disp(['Input value rightWheel for function '...
                            'SetDriveWheelsCreate too low. Must be in '...
                            'range [-0.5 0.5]. Value reset to min.'])
                    elseif rightWheel > 0.5
                        rightWheel= 0.5;
                        disp('Simulator:invalidInput')
                        disp(['Input value rightWheel for function '...
                            'SetDriveWheelsCreate too high. Must be in '...
                            'range [-0.5 0.5]. Value reset to max.'])
                    end
                    if leftWheel < -0.5
                        leftWheel= -0.5;
                        disp('Simulator:invalidInput')
                        disp(['Input value leftWheel for function '...
                            'SetDriveWheelsCreate too low. Must be in '...
                            'range [-0.5 0.5]. Value reset to min.'])
                    elseif leftWheel > 0.5
                        leftWheel= 0.5;
                        disp('Simulator:invalidInput')
                        disp(['Input value leftWheel for function '...
                            'SetDriveWheelsCreate too high. Must be in '...
                            'range [-0.5 0.5]. Value reset to max.'])
                    end
                    
                    % Change object parameters to new values
                    obj.velInt= (rightWheel+leftWheel)/2;  % Average
                    obj.wInt= (rightWheel-leftWheel)/obj.wheelbase;
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp('Error in function SetDriveWheelsCreate.')
                    end
                    rethrow(me)
                end
            end
        end
        
        function SetFwdVelAngVelCreate(obj,FwdVel,AngVel)
        % SetFwdVelAngVelCreate(obj,FwdVel,AngVel)
        % Controls the forward and angular velocity of the robot
        %
        % Input:
        % obj - Instance of class CreateRobot
        % FwdVel - Double, forward velocity in range [-0.5 0.5] (m/s)
        % AngVel - Double, angular velocity in range [-2.5 2.5] (rad/s)
        %   Note that the combination of velocities should not cause each
        %   individual wheel to exceed their limits [-0.5 0.5] (m/s)
            
            % Check for valid input
            if ~isa(obj,'CreateRobot')
                error('Simulator:invalidInput',...
                    ['The first input argument to SetFwdVelAngVelCreate '...
                    'must have class CreateRobot.  Input argument '...
                    'should be the input argument to the control program'])
            elseif ~isnumeric(FwdVel) || isempty(FwdVel)
                error('Simulator:invalidInput',...
                    ['The second input argument to SetFwdVelAngVelCreate '...
                    'must be numeric in the range [-0.5 0.5]'])
            elseif ~isnumeric(AngVel) || isempty(AngVel)
                error('Simulator:invalidInput',...
                    ['The third input argument to SetFwdVelAngVelCreate '...
                    'must be numeric in the range [-2.5 2.5]'])
            else
                try
                    % Check that autonomous is enabled and quit if not
                    autoCheck(obj)
                    
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % Add the translator function call to output data
                    fcn_called= sprintf('SetFwdVelAngVelCreate(%.3f,%.3f)',...
                        FwdVel,AngVel);
                    addFcnToOutput(obj,fcn_called)
                    
                    % Limit individual wheel velocities
                    wheelRight= FwdVel+AngVel*obj.wheelbase/2;
                    wheelLeft= FwdVel-AngVel*obj.wheelbase/2;
                    if abs(wheelRight) > 0.5 || abs(wheelLeft) > 0.5
                        wheelRight= min(max(wheelRight,-0.5),0.5);
                        wheelLeft= min(max(wheelRight,-0.5),0.5);
                        disp(['Warning: desired velocity combination '...
                            'exceeds limits.'])
                        disp('Simulator:invalidInput')
                        disp(['Each wheel can only move at 0.5 m/s. '...
                            'Choose a velocity combination that does '...
                            'not exceed these limits. Values set to max.'])
                        % Recalculate fwd and ang velocities
                        FwdVel= (wheelRight+wheelLeft)/2;
                        AngVel= (wheelRight-wheelLeft)/obj.wheelbase;
                    end
                    
                    % Change object parameters to new values
                    obj.velInt  = FwdVel;
                    obj.wInt    = AngVel;
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp('Error in function SetFwdVelAngVelCreate.')
                    end
                    rethrow(me)
                end
            end
        end
        
        function travelDist(obj,speed,distance)
        % travelDist(obj,speed,distance)
        % Commands robot to move specified distance at specified speed in a
        %   straight line, then stop
        %
        % Input:
        % obj - Instance of class CreateRobot
        % speed - abs(Velocity) within range [0.025 0.5] (m/s)
        % distance - Distance to travel (m)
        %   Direction is controlled by distance (negative distance moves
        %     the robot backwards)
        %   Negative values for speed will be changed to positive
            
            % Check for valid input
            if ~isa(obj,'CreateRobot')
                error('Simulator:invalidInput',...
                    ['The first input argument to travelDist '...
                    'must have class CreateRobot.  Input argument '...
                    'should be the input argument to the control program'])
            elseif ~isnumeric(speed) || isempty(speed)
                error('Simulator:invalidInput',...
                    ['The second input argument to travelDist '...
                    'must be numeric in the range [0.025 0.5]'])
            elseif ~isnumeric(distance) || isempty(distance)
                error('Simulator:invalidInput',...
                    ['The third input argument to travelDist '...
                    'must be numeric'])
            else
                try
                    % Check that autonomous is enabled and quit if not
                    autoCheck(obj)
                    
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % Add the translator function call to output data
                    fcn_called= sprintf('travelDist(%.3f,%.3f)',speed,distance);
                    addFcnToOutput(obj,fcn_called)
                    
                    % Check for input within range
                    if speed < 0
                        speed= abs(speed);
                        disp(['WARNING: Speed inputted is negative. '...
                            'Should be positive. Taking the absolute '...
                            'value'])
                        disp('Simulator:invalidInput')
                        disp(['Input value speed for function '...
                            'travelDist should not be negative. Sign '...
                            'is changed'])
                    elseif speed < 0.025
                        speed= 0.025;
                        disp(['WARNING: Speed inputted is too low. '...
                            'Setting speed to minimum, .025 m/s'])
                        disp('Simulator:invalidInput')
                        disp(['Input value speed for function '...
                            'travelDist too low. Value set to min.'])
                    end
                    if speed > 0.5
                        speed= 0.5;
                        disp('Simulator:invalidInput')
                        disp(['Input value speed for function '...
                            'travelDist too high. Value set to max.'])
                    end
                    
                    % Change object parameters to new values
                    obj.wInt= 0;
                    obj.velInt= sign(distance)*speed;
                    
                    % Use odometry to travel to end point
                    olddist= obj.odomDist;
                    while obj.autoEnable && ...
                            abs(obj.odomDist-olddist) < abs(distance)
                        pause(0.05)
                    end
                    obj.velInt= 0;     % Stop when done
                    disp('Done travelDist')
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp('Error in function travelDist')
                    end
                    rethrow(me)
                end
            end
        end
        
        function turnAngle(obj,speed,angle)
        % turnAngle(obj,speed,angle)
        % Commands robot to turn specified angle at a speed, then stop
        %
        % Input:
        % obj - Instance of class CreateRobot
        % speed - abs(Angular Velocity) within range [0 0.2] (rad/s)
        % angle - Angle to turn within range [-360 360] (deg)
        %   Note the unit difference: angle is in DEGREES
        %   Direction is controlled by angle based on shortest path
        %     angle in [0 180] or [-180 -360] turns counter-clockwise
        %     angle in [180 360] or [0 -180] turns clockwise
        %   Negative values for speed will be changed to positive
            
            % Check for valid input
            if ~isa(obj,'CreateRobot')
                error('Simulator:invalidInput',...
                    ['Input to RoombaInit must have class CreateRobot.'...
                    '  Input argument should be the input argument '...
                    'to the control program'])
            else
                try
                    % Check that autonomous is enabled and quit if not
                    autoCheck(obj)
                    
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % Add the translator function call to output data
                    fcn_called= sprintf('turnAngle(%.3f,%.3f)',speed,angle);
                    addFcnToOutput(obj,fcn_called)
                    
                    % Check for input within range
                    if speed < 0
                        speed= abs(speed);
                        disp(['WARNING: Speed inputted is negative. '...
                            'Should be positive. Taking the absolute '...
                            'value'])
                        disp('Simulator:invalidInput')
                        disp(['Input value speed for function '...
                            'turnAngle should not be negative. Sign '...
                            'is changed.'])
                    elseif speed < 0.025
                        speed= 0.025;
                        disp(['WARNING: Speed inputted is too low. '...
                            'Setting speed to minimum, .025 m/s']);
                        disp('Simulator:invalidInput')
                        disp(['Input value speed for function '...
                            'turnAngle too low. Value set to min.'])
                    end
                    if speed > 0.2
                        speed= 0.2;
                        disp('Simulator:invalidInput')
                        disp(['Input value speed for function '...
                            'turnAngle too high. Value set to max.'])
                    end
                    if angle < -360
                        disp('Setting angle to be between +/- 360 degrees')
                        disp('Simulator:invalidInput')
                        disp(['Input value angle for function '...
                            'turnAngle too low. Value set to equivalent '...
                            'angle in range [-360 360].'])
                    elseif angle > 360
                        disp('Setting angle to be between +/- 360 degrees')
                        disp('Simulator:invalidInput')
                        disp(['Input value angle for function '...
                            'turnAngle too high. Value set to equivalent '...
                            'angle in range [-360 360].'])
                    end
                    angle= obj.wrap2pi(pi*angle/180);
                    
                    % Choose correct turning direction for shortest path
                    if angle > 0
                        disp(['Setting turn path to shortest route. '...
                            'Going to turn counter-clockwise'])
                    else
                        disp(['Setting turn path to shortest route. '...
                            'Going to turn clockwise'])
                        speed= -speed;
                    end
                    
                    % Change object parameters to new values
                    obj.velInt= 0;
                    obj.wInt= speed;
                    
                    % Use odometry to travel to end point
                    oldang= obj.odomAng;
                    while obj.autoEnable && ...
                            abs(obj.odomAng-oldang) < abs(angle)
                        pause(0.05)
                    end
                    obj.wInt= 0;   % Stop when done
                    disp('Done turnAngle')
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp('Error in function turnAngle.')
                    end
                    rethrow(me)
                end
            end
        end
        
        function state= VirtualWallSensorCreate(obj)
        % state = VirtualWallSensorCreate(obj)
        % Reads the state of the virtual wall sensor
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % state - Boolean double, 1 if the robot detects a virtual wall
            
            % Check for valid input
            if ~isa(obj,'CreateRobot')
                error('Simulator:invalidInput',...
                    ['Input to VirtualWallSensorCreate must '...
                    'have class CreateRobot.  Input argument should be '...
                    'the input argument to the control program'])
            else
                try
                    % Check that autonomous is enabled and quit if not
                    autoCheck(obj)
                    
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % Find out if robot is within range of any virtual wall
                    state= genVWall(obj);
                    
                    % Add the translator function call to output data
                    fcn_called= sprintf('%.0f= VirtualWallSensorCreate',...
                        state);
                    addFcnToOutput(obj,fcn_called)
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp('Error in function VirtualWallSensorCreate.')
                    end
                    rethrow(me)
                end
            end
        end
        
        function distance= ReadSonar(obj,varargin)
        % distance = ReadSonar(obj)
        % Reads the distance returned by the front sonar sensor
        % Functionality has changed from the WOOSH software
        %
        % distance = ReadSonar(obj,sonarNum)
        % Reads the distance returned by the specified sonar sensor
        % Identical to ReadSonarMultiple
        %
        %
        % Input:
        % obj - Instance of class CreateRobot
        % sonarNum - Number corresponding to sonar to be read
        %   The simulator assumes this sonar setup on the WOOSH board:
        %       1 - Right
        %       2 - Front
        %       3 - Left
        %       4 - Back
        %
        % Output:
        % distance - Double, distance to nearest obstacle in front of 
        %   the robot, or the range of the sonar if no obstacle in range
            
            % Check for valid input
            if ~isa(obj,'CreateRobot')
                error('Simulator:invalidInput',...
                    ['The first input argument to ReadSonar must have '...
                    'class CreateRobot.  Input argument should be the '...
                    'input argument to the control program'])
            elseif nargin > 2
                error('Simulator:invalidInput',...
                    ['There can be no more than 2 input arguments to '...
                    'ReadSonar.'])
            elseif ~isnumeric(varargin{1}) || ~(length(varargin{1}) == 1)
                error('Simulator:invalidInput',...
                    ['The second input argument to ReadSonar must be a '...
                    'single number.'])
            elseif varargin{1} < 1 || varargin{1} > 4
                error('Simulator:invalidInput',...
                    ['The second input argument to ReadSonar must be a '...
                    'number between 1 and 4.'])
            else
                try
                    % Check that autonomous is enabled and quit if not
                    autoCheck(obj)
                    
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % Generate reading for sensor
                    distSonar= genSonar(obj);
                    
                    % Extract value for output
                    if isempty(varargin)
                        distance= distSonar(2);
                    else
                        sonarNum= varargin{1};
                        % Extract value for output
                        % Note difference between order of output of 
                        % genSonar and order of indices for sonarNum
                        if sonarNum == 1
                            distIdx= 4;
                        else
                            distIdx= sonarNum-1;
                        end
                        distance= distSonar(distIdx);
                    end
                    
                    % Check that distance is within limits
                    % Sonar functions output empty vector otherwise
                    if distance <= obj.rangeMinSonar || ...
                        distance >= obj.rangeSonar
                        distance= [];
                    end
                    
                    % Add the translator function call to output data
                    if isempty(distance)
                        fcn_called= sprintf('[]= ReadSonar');
                    else
                        fcn_called= sprintf('%.3f= ReadSonar',distance);
                    end
                    addFcnToOutput(obj,fcn_called)
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp('Error in function ReadSonar.')
                    end
                    rethrow(me)
                end
            end
        end
        
        function distance= ReadSonarMultiple(obj,sonarNum)
        % distance = ReadSonarMultiple(obj,sonarNum)
        % Reads the distance returned by the specified sonar sensor
        %
        % Input:
        % obj - Instance of class CreateRobot
        % sonarNum - Number corresponding to sonar to be read
        %   The simulator assumes this sonar setup on the WOOSH board:
        %       1 - Right
        %       2 - Front
        %       3 - Left
        %       4 - Back
        %
        % Output:
        % distance - Double, distance to nearest obstacle in the path of 
        %   the specified sonar, or an empty vector no obstacle 
        %   in range
            
            % Check for valid input
            if ~isa(obj,'CreateRobot')
                error('Simulator:invalidInput',...
                    ['Input to ReadSonarMultiple must have class '...
                    'CreateRobot.  Input argument should be the input '...
                    'argument to the control program'])
            else
                try
                    % Check that autonomous is enabled and quit if not
                    autoCheck(obj)
                    
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % Generate reading for sensor
                    distSonar= genSonar(obj);
                    
                    % Extract value for output
                    % Note difference between order of output of genSonar
                    % and order of indices for sonarNum
                    if sonarNum == 1
                        distIdx= 4;
                    else
                        distIdx= sonarNum-1;
                    end
                    distance= distSonar(distIdx);
                    
                    % Check that distance is within limits
                    % Sonar functions output empty vector otherwise
                    if distance < obj.rangeMinSonar || ...
                        distance > obj.rangeSonar
                        distance= [];
                    end
                    
                    % Add the translator function call to output data
                    if isempty(distance)
                        fcn_called= sprintf('[]= ReadSonarMultiple(%.0f)',...
                            sonarNum);
                    else
                        fcn_called= sprintf(['%.3f= ReadSonarMultiple'...
                            '(%.0f)'],distance,sonarNum);
                    end
                    addFcnToOutput(obj,fcn_called)
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp('Error in function ReadSonarMultiple.')
                    end
                    rethrow(me)
                end
            end
        end
        
        function distScan= LidarSensorCreate(obj)
        % distScan = LidarSensorCreate(obj)
        % Reads the range of distances from the LIDAR sensor
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % distScan - Array of doubles, of length numPtsLidar with the
        %   first value corresponding to the right-most reading and the
        %   last corresponding to the left-most reading on the LIDAR, the
        %   readings will be the distance to the nearest obstacle, or the
        %   range of the LIDAR if no obstacle is in range
            
            % Check for valid input
            if ~isa(obj,'CreateRobot')
                error('Simulator:invalidInput',...
                    ['Input to LidarSensorCreate must have class '...
                    'CreateRobot.  Input argument should be the input '...
                    'argument to the control program'])
            else
                try
                    % Check that autonomous is enabled and quit if not
                    autoCheck(obj)
                    
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % Generate reading for sensor
                    distScan= genLidar(obj);
                    
                    % Add the translator function call to output data
                    fcn_called= sprintf(['[%.3f %.3f %.3f %.3f %.3f]= '...
                        'LidarSensorCreate'],distScan(1),...
                        distScan(ceil(obj.numPtsLidar/4)),...
                        distScan(ceil(obj.numPtsLidar/2)),...
                        distScan(ceil(3*obj.numPtsLidar/4)),...
                        distScan(end));
                    addFcnToOutput(obj,fcn_called)
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp('Error in function LidarSensorCreate.')
                    end
                    rethrow(me)
                end
            end
        end
        
        function [X Y Z ROT Ntag] = ReadBeacon(obj)
        % [X Y Z ROT Ntag] = ReadBeacon(obj)
        % Reads the ARtag detection camera and reports 3-D cartesian
        % position in the camera reference frame, as well as the rotation
        % about the tag center
        %
        % Camera coordinate frame is defined as (updated May 2014):
        % x - axis points to right
        % y - axis points down
        % z - axis points out of camera (depth)
        %
        % Input:
        % obj - Instance of the class CreateRobot
        % 
        % Output: 
        % X   - column vector of x coordinates in camera coordinates
        % y   - column vector of y coordinates in camera coordinates
        % z   - column vector of z coordinates in camera coordinates
        % rot - column vector of rotation of tag around its center point
        % Ntag- column vector of AR tag numbers detected in camera frame
        
         % Check for valid input
            if ~isa(obj,'CreateRobot')
                error('Simulator:invalidInput',...
                    ['Input to ReadBeacon must have class '...
                    'CreateRobot.  Input argument should be the input '...
                    'argument to the control program'])
            else
                try
                    % Check that autonomous is enabled and quit if not
                    autoCheck(obj)
                    
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % Generate reading for sensor
                    [angle, dist,color, Ntag]= genCamera(obj);

                    X = -dist.*sin(angle);      % Minus, because x-axis right
                    Y = zeros(numel(angle),1);  % Assume tags are in same plane as the camera
                    Z = +dist.*cos(angle);
                    ROT = Y;                    % Assume tags are oriented upright 
                    
                    % Add the translator function call to output data
                    if isempty(angle)   % No beacons detected
                        fcn_called= '[]= ReadBeacon';
                    else                % Only output first beacon found
                        fcn_called= sprintf(['[%.3f %.3f %.3f %.3f %.3f]'...
                            ' = ReadBeacon'],X(1),Y(1),Z(1),ROT(1),Ntag(1));
                    end
                    addFcnToOutput(obj,fcn_called)
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp('Error in function ReadBeacon.')
                    end
                    rethrow(me)
                end
            end
        end
        
        function [angle dist color]= CameraSensorCreate(obj)
        % [angle dist color] = CameraSensorCreate(obj)
        % Reads the output from the blob detection on the camera,
        %   detects only beacons
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % angle - Vector of doubles, each the angle relative to robot at 
        %   which beacon is detected
        % dist - Vector of doubles, each the distance of beacon from camera
        % color - Matrix of doubles of width three (color vector), 
        %   each row the color of beacon detected
            
            % Check for valid input
            if ~isa(obj,'CreateRobot')
                error('Simulator:invalidInput',...
                    ['Input to CameraSensorCreate must have class '...
                    'CreateRobot.  Input argument should be the input '...
                    'argument to the control program'])
            else
                try
                    % Check that autonomous is enabled and quit if not
                    autoCheck(obj)
                    
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % Generate reading for sensor
                    [angle dist color]= genCamera(obj);
                    
                    % Add the translator function call to output data
                    if isempty(angle)   % No beacons detected
                        fcn_called= '[]= CameraSensorCreate';
                    else                % Only output first beacon found
                        fcn_called= sprintf(['[%.3f %.3f [%.3f %.3f '...
                            '%.3f]]= CameraSensorCreate'],angle(1),...
                            dist(1),color(1,1),color(1,2),color(1,3));
                    end
                    addFcnToOutput(obj,fcn_called)
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp('Error in function CameraSensorCreate.')
                    end
                    rethrow(me)
                end
            end
        end
        function [fvel wvel] = getRobotvel(obj)
            fvel = obj.velInt;
            wvel = obj.wInt;
        end
        function [x y angle]= OverheadLocalizationCreate(obj)
        % [x y angle] = OverheadLocalizationCreate(obj)
        % Read the output of the overhead localization system
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % x - x-coordinate of the robot
        % y - y-coordinate of the robot
        % angle - angle of the robot relative to positive x-axis
            
            % Check for valid input
            if ~isa(obj,'CreateRobot')
                error('Simulator:invalidInput',...
                    ['Input to OverheadLocalizationCreate must have '...
                    'class CreateRobot.  Input argument should be the '...
                    'input argument to the control program'])
            else
                try
                    % Check that autonomous is enabled and quit if not
                    autoCheck(obj)
                    
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % Generate reading for sensor
                    [x y angle]= genOverhead(obj);
                    
                    % Add the translator function call to output data
                    fcn_called= sprintf(['[%.3f %.3f %.3f]]= '...
                        'OverheadLocalizationCreate'],x,y,angle);
                    addFcnToOutput(obj,fcn_called)
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp('Error in function OverheadLocalizationCreate.')
                    end
                    rethrow(me)
                end
            end
        end
    end
end