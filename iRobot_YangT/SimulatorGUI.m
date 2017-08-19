function varargout = SimulatorGUI(varargin)
% SimulatorGUI M-file for SimulatorGUI.fig
% Simulator for the iRobot Create
% To run, enter "SimulatorGUI" into the command window with not input or
%   output arguments.  This will bring up the GUI window for the simulator,
%   and initialize a CreateRobot object to hold simulation data and perform
%   the required functions.

% SimulatorGUI.m
% Copyright (C) 2011 Cornell University
% This code is released under the open-source BSD license.  A copy of this
% license should be provided with the software.  If not, email:
% CreateMatlabSim@gmail.com

% Edit the above text to modify the response to help SimulatorGUI

% Last Modified by GUIDE v2.5 26-Jan-2011 22:23:42

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @SimulatorGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @SimulatorGUI_OutputFcn, ...
                   'gui_LayoutFcn',  [], ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
   gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before SimulatorGUI is made visible.
function SimulatorGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   unrecognized PropertyName/PropertyValue pairs from the
%            command line (see VARARGIN)
% UIWAIT makes SimulatorGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);

% Choose default command line output for SimulatorGUI
% Output all handles for use with updating
handles.output = handles;

% Store robot object for access to functions
% Store in UserData of title block because no where else to put it
obj= CreateRobot(handles);
set(handles.text_title,'UserData',obj)

% Get constant properties for use in plotting
[rad rIR rSon rLid angRLid numPtsLid]= getConstants(obj);

% Plot robot in default position and store plot handles for updating
axes(handles.axes_map)
circ_numPts= 21;    % Estimate circle as circ_numPts-1 lines
circ_ang=linspace(0,2*pi,circ_numPts);
circ_rad=ones(1,circ_numPts)*rad;
[circ_x circ_y]= pol2cart(circ_ang,circ_rad);
handle_circ= plot(circ_x,circ_y,'b-','LineWidth',1.5);
handle_line= plot([0 1.5*rad],[0 0],'b-','LineWidth',1.5);
set(handles.figure_simulator,'UserData',[handle_circ ; handle_line])

% Plot initial sensors visualization and make invisible
% These plot coordinates are rough to reduce computation here, but it is
%   required that something is plotted
handle_wallIR= plot([rad rad],[0 -rIR],'r-','LineWidth',2);
handle_sonarF= plot([rad rad+rSon],[0 0],'g-','LineWidth',2);
handle_sonarL= plot([0 0],[rad rad+rSon],'g-','LineWidth',2);
handle_sonarB= plot([-rad -(rad+rSon)],[0 0],'g-','LineWidth',2);
handle_sonarR= plot([0 0],[-rad -(rad+rSon)],'g-','LineWidth',2);
handle_bumpR= plot([0 rad],[-rad 0],'m-','LineWidth',2);
handle_bumpF= plot([rad rad],[-rad/2 rad/2],'m-','LineWidth',2);
handle_bumpL= plot([0 rad],[rad 0],'m-','LineWidth',2);
handle_cliffR= plot(0,rad,'c*');
handle_cliffFR= plot(rad,-rad/3,'c*');
handle_cliffFL= plot(rad,rad/3,'c*');
handle_cliffL= plot(0,rad,'c*');
handle_lidarR= plot([rad rLid*cos(-angRLid/2)],...
    [0 rLid*sin(-angRLid/2)],'Color',[1 0.8 0],'LineWidth',2);
handle_lidarFR= plot([rad rLid*cos(-angRLid/4)],...
    [0 rLid*sin(-angRLid/4)],'Color',[1 0.8 0],'LineWidth',2);
handle_lidarF= plot([rad rLid],[0 0],'Color',[1 0.8 0],'LineWidth',2);
handle_lidarFL= plot([rad rLid*cos(angRLid/4)],...
    [0 rLid*sin(angRLid/4)],'Color',[1 0.8 0],'LineWidth',2);
handle_lidarL= plot([rad rLid*cos(angRLid/2)],...
    [0 rLid*sin(angRLid/2)],'Color',[1 0.8 0],'LineWidth',2);
handles_sensors= [handle_wallIR handle_sonarF handle_sonarL ...
    handle_sonarB handle_sonarR handle_bumpR handle_bumpF handle_bumpL ...
    handle_cliffR handle_cliffFR handle_cliffFL handle_cliffL ...
    handle_lidarR handle_lidarFR handle_lidarF handle_lidarFL ...
    handle_lidarL]';
set(handles_sensors,'Visible','off')
set(handles.axes_map,'UserData',handles_sensors)

% Set up timer to control simulation updates
timerSim= timer;
timerSim.BusyMode= 'drop';
timerSim.ExecutionMode= 'fixedSpacing';
%%%%%%%%%%Change?%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
timerSim.Name= 'CreateSim';    % To specify timer for deletion
timerSim.ObjectVisibility= 'on';
%%%%%%%%%%Change?%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
timerSim.Period= 0.1;
timerSim.TasksToExecute= inf;
timerSim.TimerFcn= {@updateSim,obj,handles};
start(timerSim)

% Add in axes toolbar
set(hObject,'toolbar','figure');

% Update handles structure
guidata(hObject, handles);


% --- Outputs from this function are returned to the command line.
function varargout = SimulatorGUI_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes when user attempts to close figure_simulator.
function figure_simulator_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure_simulator (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Delete the timer to prevent further updates
timerList= timerfindall('Name','CreateSim');
if ~isempty(timerList)
    stop(timerList)
    delete(timerList)
end

% Use conditional in case figure is opened without running simulator
if ~isempty(handles)
    % Get robot object
    obj= get(handles.text_title,'UserData');
    
    % Disable autonomous control
    setAutoEnable(obj,false)
end

% Close the figure
delete(hObject);


% --- Executes on button press in push_map.
function push_map_Callback(hObject, eventdata, handles)
% hObject    handle to push_map (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Import map file
[filename pathname filter]= uigetfile('*.txt','Import Map File');
if filename                         % Make sure cancel was not pressed
    fid= fopen([pathname filename]);% Get file handle for parsing
    
    % Parse the file and extract relevant information
    walls= [];
    lines= [];
    beacs= {};
    vwalls= [];
    while ~feof(fid)
        fline= fgetl(fid);
        line= lower(fline);     % Convert to lowercase if necessary
        line= strtrim(line);    % Delete leading and trailing whitespace
        lineWords= {};          % To keep track of line entries
        while ~isempty(line) && ~strcmp(line(1),'%')% End of line or comment
            [word line]= strtok(line);  % Get next entry
            line= strtrim(line);        % To be able to detect comments
            lineWords= [lineWords word];
        end
        
        %%% Regular expressions would probably be more efficient %%%
        if length(lineWords) == 5 && strcmp(lineWords{1},'wall') && ...
                ~isnan(str2double(lineWords{2})) && ...
                ~isnan(str2double(lineWords{3})) && ...
                ~isnan(str2double(lineWords{4})) && ...
                ~isnan(str2double(lineWords{5}))
            walls= [walls ; str2double(lineWords{2}) ...
                str2double(lineWords{3}) str2double(lineWords{4}) ...
                str2double(lineWords{5})];
        elseif length(lineWords) == 5 && strcmp(lineWords{1},'line') && ...
                ~isnan(str2double(lineWords{2})) && ...
                ~isnan(str2double(lineWords{3})) && ...
                ~isnan(str2double(lineWords{4})) && ...
                ~isnan(str2double(lineWords{5}))
            lines= [lines ; str2double(lineWords{2}) ...
                str2double(lineWords{3}) str2double(lineWords{4}) ...
                str2double(lineWords{5})];
        elseif length(lineWords) == 7 && strcmp(lineWords{1},'beacon') ...
                && ~isnan(str2double(lineWords{2})) && ...
                ~isnan(str2double(lineWords{3})) && ...
                length(lineWords{4}) >= 2 && ...
                strcmp(lineWords{4}(1),'[') && ...
                ~isnan(str2double(lineWords{4}(2:end))) && ...
                ~isnan(str2double(lineWords{5})) && ...
                length(lineWords{6}) >= 2 && ...
                strcmp(lineWords{6}(end),']') && ...
                ~isnan(str2double(lineWords{6}(1:end-1)))
            beacs= [beacs ; str2double(lineWords{2}) ...
                str2double(lineWords{3}) ...
                str2double(lineWords{4}(2:end)) ...
                str2double(lineWords{5}) ...
                str2double(lineWords{6}(1:end-1)) lineWords(7)];
        elseif length(lineWords) == 5 && ...
                strcmp(lineWords{1},'virtwall') && ...
                ~isnan(str2double(lineWords{2})) && ...
                ~isnan(str2double(lineWords{3})) && ...
                ~isnan(str2double(lineWords{4})) && ...
                ~isnan(str2double(lineWords{5})) && ...
                str2double(lineWords{5}) >= 1 && ...
                str2double(lineWords{5}) <= 3
            vwalls= [vwalls ; str2double(lineWords{2}) ...
                str2double(lineWords{3}) str2double(lineWords{4}) ...
                str2double(lineWords{5})];
        elseif ~isempty(lineWords)
            warning('MATLAB:invalidInput',...
                'This line in map file %s is unrecognized:\n\t%s',...
                filename,fline)
        end
    end
    fclose(fid);
    
    % Set map data in robot object
    obj= get(handles.text_title,'UserData');
    setMap(obj,walls,lines,beacs,vwalls)
    
    % Clear old map
    axes(handles.axes_map)
    children= get(gca,'Children');  % All ploted items on axes
    handles_sensors= get(handles.axes_map,'UserData');
    for i= 1:length(handles_sensors)    % Keep sensors visualization
        children(find(handles_sensors(i) == children))= [];
    end
    if ~isempty(children)
        delete(children(1:end-2))	% Delete all but robot from axes
    end
    
    % Plot walls
    for i= 1:size(walls,1)
        plot(walls(i,[1 3]),walls(i,[2 4]),'k-','LineWidth',1)
    end
    
    % Plot lines
    for i= 1:size(lines,1)
        plot(lines(i,[1 3]),lines(i,[2 4]),'k--','LineWidth',1)
    end
    
    % Plot beacons
    for i= 1:size(beacs,1)
        plot(beacs{i,1},beacs{i,2},...
            'Color',cell2mat(beacs(i,3:5)),'Marker','o')
        text(beacs{i,1},beacs{i,2},['  ' beacs{i,6}])
    end
    
    % Plot virtual walls
    % Define virtual wall emitter constants
    halo_rad= 0.45;     % Radius of the halo around the emitter
    range_short= 2.13;  % Range of the wall on the 0-3' setting
    ang_short= 0.33;    % Angular range on the 0-3' setting
    range_med= 5.56;    % Range of the wall on the 4'-7' setting
    ang_med= 0.49;      % Angular range on the 4'-7' setting
    range_long= 8.08;   % Range of the wall on the 8'+ setting
    ang_long= 0.61;     % Angular range on the 8'+ setting
    
    % Get points to map virtual walls
    for i= 1:size(vwalls,1)
        x_vw= vwalls(i,1);
        y_vw= vwalls(i,2);
        th_vw= vwalls(i,3);
        if vwalls(i,4) == 1
            range_vw= range_short;
            ang_vw= ang_short;
        elseif vwalls(i,4) == 2
            range_vw= range_med;
            ang_vw= ang_med;
        else
            range_vw= range_long;
            ang_vw= ang_long;
        end
        x_1= x_vw+range_vw*cos(th_vw+ang_vw/2);
        y_1= y_vw+range_vw*sin(th_vw+ang_vw/2);
        x_2= x_vw+range_vw*cos(th_vw-ang_vw/2);
        y_2= y_vw+range_vw*sin(th_vw-ang_vw/2);
        
        % Plot halo around emitter and range triangle
        circ_numPts= 21;    % Estimate circle as circ_numPts-1 lines
        circ_ang=linspace(0,2*pi,circ_numPts);
        circ_rad=ones(1,circ_numPts)*halo_rad;
        [circ_x circ_y]= pol2cart(circ_ang,circ_rad);
        circ_x= circ_x+x_vw;
        circ_y= circ_y+y_vw;
        plot(x_vw,y_vw,'g*')
        plot(circ_x,circ_y,'g:','LineWidth',1);
        plot([x_vw x_1 x_2 x_vw],[y_vw y_1 y_2 y_vw],'g:','LineWidth',1)
    end
end


% --- Executes on button press in push_config.
function push_config_Callback(hObject, eventdata, handles)
% hObject    handle to push_config (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Import configuration file
[filename pathname filter]= uigetfile('*.txt','Import Configuration File');
noise= struct;  % Initialize empty structure for noise information
if filename                         % Make sure cancel was not pressed
    fid= fopen([pathname filename]);% Get file handle for parsing
    
    % Get robot object for data entry
    obj= get(handles.text_title,'UserData');
    
    % Parse the file and extract relevant information
    while ~feof(fid)
        fline= fgetl(fid);      % Get line of file
        line= lower(fline);     % Convert to lowercase if necessary
        line= strtrim(line);    % Delete leading and trailing whitespace
        lineWords= {};          % To keep track of line entries
        while ~isempty(line) && ~strcmp(line(1),'%')% End of line or comment
            [word line]= strtok(line);  % Get next entry
            line= strtrim(line);        % To be able to detect comments
            lineWords= [lineWords word];
        end
        
        % Get information from line
        sensors= {'wall' 'cliff' 'odometry' 'sonar' 'lidar' 'camera' 'imu'};
        if length(lineWords) == 2 && strcmp(lineWords{1},'com_delay')
            setComDelay(obj,str2double(lineWords{2}))
        elseif length(lineWords) == 3 && any(strcmp(lineWords{1},sensors))
            % Name the field in the noise structure after the sensor and
            % store the mean and standard deviation of the noise in it
            noise.(lineWords{1})= ...
                [str2double(lineWords{2}) str2double(lineWords{3})];
        elseif ~isempty(lineWords)
            warning('MATLAB:invalidInput',...
                'This line in config file %s is unrecognized:\n\t%s',...
                filename,fline)
        end
    end
    fclose(fid);
    
    % Set noise data in robot object
    setNoise(obj,noise)
end


% --- Executes on button press in push_origin.
function push_origin_Callback(hObject, eventdata, handles)
% hObject    handle to push_origin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Set origin data in robot object
obj= get(handles.text_title,'UserData');
[x y]= ginput(2);   % Accept two mouse clicks
th= atan2(y(2)-y(1),x(2)-x(1)); % Direction is set by second mouse click
origin= [x(1) y(1) th];         % Start point is set by first mouse click
setMapStart(obj,origin)


% --- Executes on button press in chkbx_wall.
function chkbx_wall_Callback(hObject, eventdata, handles)
% hObject    handle to chkbx_wall (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of chkbx_wall

% Toggle visibility of sensor visualization with the checkbox
handles_sensors= get(handles.axes_map,'UserData');
if get(handles.chkbx_wall,'Value')
    set(handles_sensors(1),'Visible','on')
else
    set(handles_sensors(1),'Visible','off')
end


% --- Executes on button press in chkbx_sonar.
function chkbx_sonar_Callback(hObject, eventdata, handles)
% hObject    handle to chkbx_sonar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of chkbx_sonar

% Toggle visibility of sensor visualization with the checkbox
handles_sensors= get(handles.axes_map,'UserData');
if get(handles.chkbx_sonar,'Value')
    set(handles_sensors(2:5),'Visible','on')
else
    set(handles_sensors(2:5),'Visible','off')
end


% --- Executes on button press in chkbx_bump.
function chkbx_bump_Callback(hObject, eventdata, handles)
% hObject    handle to chkbx_bump (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of chkbx_bump

% Toggle visibility of sensor visualization with the checkbox
handles_sensors= get(handles.axes_map,'UserData');
if get(handles.chkbx_bump,'Value')
    set(handles_sensors(6:8),'Visible','on')
else
    set(handles_sensors(6:8),'Visible','off')
end


% --- Executes on button press in chkbx_cliff.
function chkbx_cliff_Callback(hObject, eventdata, handles)
% hObject    handle to chkbx_cliff (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of chkbx_cliff

% Toggle visibility of sensor visualization with the checkbox
handles_sensors= get(handles.axes_map,'UserData');
if get(handles.chkbx_cliff,'Value')
    set(handles_sensors(9:12),'Visible','on')
else
    set(handles_sensors(9:12),'Visible','off')
end


% --- Executes on button press in chkbx_lidar.
function chkbx_lidar_Callback(hObject, eventdata, handles)
% hObject    handle to chkbx_lidar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of chkbx_lidar

% Toggle visibility of sensor visualization with the checkbox
handles_sensors= get(handles.axes_map,'UserData');
if get(handles.chkbx_lidar,'Value')
    set(handles_sensors(13:17),'Visible','on')
else
    set(handles_sensors(13:17),'Visible','off')
end


% --- Executes on button press in push_sensors.
function push_sensors_Callback(hObject, eventdata, handles)
% hObject    handle to push_sensors (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get robot object to query sensors and output to command window
obj= get(handles.text_title,'UserData');
[rad rIR rSon rLid angRLid numPtsLid]= getConstants(obj);

% Bump sensors
bump= genBump(obj);
bumpR= bump(1);
bumpF= bump(2);
bumpL= bump(3);
fprintf('Bump Sensors:\n')
fprintf('\tRight: %.0f\n',bumpR)
fprintf('\tFront: %.0f\n',bumpF)
fprintf('\tLeft: %.0f\n\n',bumpL)

% Cliff sensors
cliffR= 0;
cliffFR= 0;
cliffFL= 0;
cliffL= 0;
cliff= genCliff(obj);
cliffRstr= cliff(1);
cliffFRstr= cliff(2);
cliffFLstr= cliff(3);
cliffLstr= cliff(4);
fprintf('Cliff Sensors:\n')
fprintf('\tRight: State %.0f Strength %%%.3f\n',cliffR,cliffRstr)
fprintf('\tFront-Right: State %.0f Strength %%%.3f\n',cliffFR,cliffFRstr)
fprintf('\tFront-Left: State %.0f Strength %%%.3f\n',cliffFL,cliffFLstr)
fprintf('\tLeft: State %.0f Strength %%%.3f\n\n',cliffL,cliffLstr)

% Infrared wall sensor
wall= genIR(obj);
fprintf('Wall Sensor: %.0f\n\n',wall)

% Virtual Wall sensor
vwall= genVWall(obj);
fprintf('Virtual Wall Sensor: %.0f\n\n',vwall)

% Sonar sensors
distSonar= genSonar(obj);
fprintf('Sonar Sensors:\n')
fprintf('\tFront: %.3f m\n',distSonar(1))
fprintf('\tLeft: %.3f m\n',distSonar(2))
fprintf('\tRear: %.3f m\n',distSonar(3))
fprintf('\tRight: %.3f m\n\n',distSonar(4))

% LIDAR sensor
handleGUI= gcf;     % Get GUI figure handle to switch back to
distLidar= genLidar(obj);
angleData= linspace(pi/2-angRLid/2,pi/2+angRLid/2,numPtsLid);
figure
hold on
polar(linspace(0,2*pi,11),rad*ones(1,11),'b-')
polar([0 pi/2],[0 1.5*rad],'b-')
polar([angleData(1) 0 angleData(end)],[distLidar(1) 0 distLidar(end)],'k-')
polar(angleData,distLidar,'k.')
axis([-10 10 -10 10])
title('LIDAR Data')
xlabel('Distance (m)')
ylabel('Distance (m)')
hold off
figure(handleGUI)   % Switch back to GUI figure
fprintf('LIDAR Sensor: see figure\n\n')

% Odometry
distOdom= genOdomDist(obj);
angOdom= genOdomAng(obj);
fprintf('Odometry Data (since last call):\n')
fprintf('\tDistance: %.3f m\n',distOdom)
fprintf('\tAngle: %.3f m\n\n',angOdom)

% Overhead localization system
[x y th]= genOverhead(obj);
fprintf('Overhead Localization System output:\n')
fprintf('\tX-Coordinate: %.3f m\n',x)
fprintf('\tY-Coordinate: %.3f m\n',y)
fprintf('\tAngle relative to horizontal: %.3f rad\n\n',th)

% Camera
[angCameraBeacon distCameraBeacon colorCam ID]= genCamera(obj);
% Also see ReadBeacon function in CreateRobot.m for details on camera.

X = -distCameraBeacon.*sin(angCameraBeacon);    % Minus, because camera x-axis is to the right
Z = +distCameraBeacon.*cos(angCameraBeacon);    % Camera's positive z-axis points forward (depth)
fprintf('Camera Data:\n')
for i= 1:length(angCameraBeacon)
    fprintf('Beacon at: theta = %.0f deg, R = %.3f m, with color [%.2f %.2f %.2f] \n',...
        rad2deg(angCameraBeacon(i)),distCameraBeacon(i),colorCam(i,1),colorCam(i,2),...
        colorCam(i,3))
    fprintf('           X = %.2f m, Y = %.2f m, Z = %.2f m, rot = %.0f deg, with tag number %.0f \n',...
        X(i),0,Z(i),0,ID(i))
end
fprintf('\n')


% --- Executes on button press in push_fwd.
function push_fwd_Callback(hObject, eventdata, handles)
% hObject    handle to push_fwd (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Increase linear speed in robot object
obj= get(handles.text_title,'UserData');
velChange= [0.1 0];
manualKeyboard(obj,velChange)


% --- Executes on button press in push_back.
function push_back_Callback(hObject, eventdata, handles)
% hObject    handle to push_back (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Decrease linear speed in robot object
obj= get(handles.text_title,'UserData');
velChange= [-0.1 0];
manualKeyboard(obj,velChange)


% --- Executes on button press in push_left.
function push_left_Callback(hObject, eventdata, handles)
% hObject    handle to push_left (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Increase angular speed in robot object
obj= get(handles.text_title,'UserData');
velChange= [0 0.5];
manualKeyboard(obj,velChange)


% --- Executes on button press in push_right.
function push_right_Callback(hObject, eventdata, handles)
% hObject    handle to push_right (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Decrease angular speed in robot object
obj= get(handles.text_title,'UserData');
velChange= [0 -0.5];
manualKeyboard(obj,velChange)


% --- Executes on button press in push_stop.
function push_stop_Callback(hObject, eventdata, handles)
% hObject    handle to push_stop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Zero all velocity in robot object
obj= get(handles.text_title,'UserData');
velChange= [NaN NaN];
manualKeyboard(obj,velChange)


% --- Executes on key press with focus on figure_simulator or any of its controls.
function figure_simulator_WindowKeyPressFcn(hObject, eventdata, handles)
% hObject    handle to figure_simulator (see GCBO)
% eventdata  structure with the following fields (see FIGURE)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)

% Check which key is pressed and call appropriate function
keydown= eventdata.Key;  % Assume only one key pressed at a time
if any(strcmp(keydown,{'uparrow' 'w'})) && ...
        strcmp(get(handles.push_fwd,'Enable'),'on')
    hObject_new= handles.push_fwd;
    push_fwd_Callback(hObject_new,[],handles)
elseif any(strcmp(keydown,{'downarrow' 's'})) && ...
        strcmp(get(handles.push_back,'Enable'),'on')
    hObject_new= handles.push_back;
    push_back_Callback(hObject_new,[],handles)
elseif any(strcmp(keydown,{'leftarrow' 'a'})) && ...
        strcmp(get(handles.push_left,'Enable'),'on')
    hObject_new= handles.push_left;
    push_left_Callback(hObject_new,[],handles)
elseif any(strcmp(keydown,{'rightarrow' 'd'})) && ...
        strcmp(get(handles.push_right,'Enable'),'on')
    hObject_new= handles.push_right;
    push_right_Callback(hObject_new,[],handles)
elseif any(strcmp(keydown,{'space' 'return' 'escape'})) && ...
        strcmp(get(handles.push_stop,'Enable'),'on')
    hObject_new= handles.push_stop;
    push_stop_Callback(hObject_new,[],handles)
end


% --- Executes on button press in push_auto_start.
function push_auto_start_Callback(hObject, eventdata, handles)
% hObject    handle to push_auto_start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get robot object to pass on to control function
obj= get(handles.text_title,'UserData');

% Stop movement from manual control
velChange= [NaN NaN];
manualKeyboard(obj,velChange)

% Choose file to run as autonomous control function
[fileName pathName filter]= ...
    uigetfile('*.m','Load Autonomous Control File');
if fileName
    fileName= fileName(1:end-2);    % Take off '.m' extension
    if ~strcmp(cd,pathName) % If control program isn't in current directory
        addpath(pathName)   % Add the path to it to allow access
    end	% This will not save over to future sessions of MATLAB unless the user
    % has the setting to save the current search path upon exit
    
    % Set autonomous control to be enabled
    setAutoEnable(obj,true)
    
    % Start timer to determine time steps for data output and add first row
    startTimeElap(obj)
    updateOutput(obj)
    
    % Disable manual control buttons
    set(handles.push_map,'Enable','off')
    set(handles.push_config,'Enable','off')
    set(handles.push_origin,'Enable','off')
    %set(handles.push_fwd,'Enable','off')
    %set(handles.push_back,'Enable','off')
    %set(handles.push_left,'Enable','off')
    %set(handles.push_right,'Enable','off')
    %set(handles.push_stop,'Enable','off')
    set(handles.push_auto_start,'Enable','off')
    
    % Call odometry reading functions to zero readings
    genOdomAng(obj);
    genOdomDist(obj);
    
    % Catch error thrown when autonomous control is disabled and control
    % program calls a translator function, in order to halt control program
    try
        % Run autonomous controller function
        feval(fileName,obj);
        
        % Run the Stop button callback to save data and re-enable manual
        % This will only run if Stop is not pushed during autonomous
        hObject_new= handles.push_auto_stop;
        push_auto_stop_Callback(hObject_new,[],handles)
    catch me                    % Catch error and information
        if strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
            manualKeyboard(obj,[NaN NaN])   % Stop robot movement
            return                          % Allow GUI control again
        else                % Not error from control being disabled
            rethrow(me)     % Allow error to propagate to command window
        end
    end
end


% --- Executes on button press in push_auto_stop.
function push_auto_stop_Callback(hObject, eventdata, handles)
% hObject    handle to push_auto_stop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get robot object
obj= get(handles.text_title,'UserData');

% Set autonomous control to be disabled
setAutoEnable(obj,false)
manualKeyboard(obj,[NaN NaN])

% Export and reset the output data
if get(handles.chkbx_auto_save,'Value')
    saveOutput(obj)
end
resetOutput(obj)

% Enable manual control buttons
set(handles.push_map,'Enable','on')
set(handles.push_config,'Enable','on')
set(handles.push_origin,'Enable','on')
set(handles.push_fwd,'Enable','on')
set(handles.push_back,'Enable','on')
set(handles.push_left,'Enable','on')
set(handles.push_right,'Enable','on')
set(handles.push_stop,'Enable','on')
set(handles.push_auto_start,'Enable','on')
