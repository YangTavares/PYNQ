function varargout = ReplayGUI(varargin)
% ReplayGUI M-file for ReplayGUI.fig
% User interface to assist with the debugging of autonomous code by playing
%   back the code execution from previous simulations
% To run, enter "ReplayGUI" into the command window with no input or output
%   arguments.  This will bring up the GUI window that will allow you to
%   import and view the data history of previous simulations

% ReplayGUI.m
% Copyright (C) 2011 Cornell University
% This code is released under the open-source BSD license.  A copy of this
% license should be provided with the software.  If not, email:
% CreateMatlabSim@gmail.com

% Edit the above text to modify the response to help ReplayGUI

% Last Modified by GUIDE v2.5 15-Jul-2010 22:41:18

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ReplayGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @ReplayGUI_OutputFcn, ...
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


% --- Executes just before ReplayGUI is made visible.
function ReplayGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   unrecognized PropertyName/PropertyValue pairs from the
%            command line (see VARARGIN)

% UIWAIT makes ReplayGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);

% Choose default command line output for ReplayGUI
% Output all handles for use with updating
handles.output = handles;

% Plot robot in default position and store plot handles for updating
axes(handles.axes_map)
circ_numPts= 21;    % Estimate circle as circ_numPts-1 lines
circ_ang=linspace(0,2*pi,circ_numPts);
circ_rad=ones(1,circ_numPts)*0.2;
[circ_x circ_y]= pol2cart(circ_ang,circ_rad);
handle_circ= plot(circ_x,circ_y,'b-','LineWidth',1.5);
set(handle_circ,'Visible','off')
handle_line= plot([0 1.5*0.2],[0 0],'b-','LineWidth',1.5);
set(handle_line,'Visible','off')
set(handles.figure_replay,'UserData',[handle_circ ; handle_line])

% Set the current index of the data file to UserData in the text
set(handles.text_IOData,'UserData',0)

% Add in axes toolbar
set(hObject,'toolbar','figure');

% Update handles structure
guidata(hObject, handles);


% --- Outputs from this function are returned to the command line.
function varargout = ReplayGUI_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Stop and delete the timer
% timerList= timerfindall('Name','CreateSim');
% if ~isempty(timerList)
%     stop(timerList)
%     delete(timerList)
% end

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes when user attempts to close figure_replay.
function figure_replay_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure_replay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: delete(hObject) closes the figure

% Clear playback flags to avoid errors
% Don't try to clear them if opened as just a figure, not with program
if ~isempty(handles)
    clearAllFlags(handles)
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
    
    % Clear old map
    axes(handles.axes_map)
    children= get(gca,'Children');  % All ploted items on axes
    handles_robot= get(handles.figure_replay,'UserData');
    for i= 1:length(handles_robot)  % Keep sensors visualization
        children(find(handles_robot(i) == children))= [];
    end
    if ~isempty(children)
        delete(children)            % Delete all but robot from axes
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


% --- Executes on button press in push_data.
function push_data_Callback(hObject, eventdata, handles)
% hObject    handle to push_data (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Import the data from the selected file into user data in the edit box
[filename pathname filter]= uigetfile('*.mat','Import Data File');
if filename                         % Make sure cancel was not pressed
    datastruct= load([pathname filename]);
    if isfield(datastruct,'datahistory')
        datahistory= datastruct.datahistory;
        if ~isempty(datahistory) && iscell(datahistory) && ...
                size(datahistory,2) == 5
            % Store the data
            set(handles.edit_display,'UserData',datahistory)
            
            % Set the robot to the first data point
            idx= get(handles.text_IOData,'UserData');
            stepPlayback(handles,-(idx-1))
            handles_robot= get(handles.figure_replay,'UserData');
            set(handles_robot,'Visible','on')
        else
            warning('MATLAB:invalidInput','Input data file not recognized')
        end
    else
        warning('MATLAB:invalidInput','Input data file not recognized')
    end
end


% --- Executes on button press in push_slow_back.
function push_slow_back_Callback(hObject, eventdata, handles)
% hObject    handle to push_slow_back (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Set the Slow Rewind flag, clear other flags
clearAllFlags(handles)
set(handles.push_slow_back,'UserData',1)

% Get data for timing
datahistory= get(handles.edit_display,'UserData');
idx= get(handles.text_IOData,'UserData');

% Loop while Slow Rewind flag is still set
while exist('hObject','var') && get(handles.push_slow_back,'UserData') ...
        && idx > 1
    % Display data and move robot for the current time step
    slow_factor= 4;
    pause(slow_factor*(datahistory{idx,1}-datahistory{idx-1,1}))
    stepPlayback(handles,-1)
    idx= get(handles.text_IOData,'UserData');
end


% --- Executes on button press in push_slow_fwd.
function push_slow_fwd_Callback(hObject, eventdata, handles)
% hObject    handle to push_slow_fwd (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Set the Play flag, clear other flags
clearAllFlags(handles)
set(handles.push_slow_fwd,'UserData',1)

% Get data for timing
datahistory= get(handles.edit_display,'UserData');
idx= get(handles.text_IOData,'UserData');

% Loop while play flag is still set
while exist('hObject','var') && get(handles.push_slow_fwd,'UserData') ...
        && idx < size(datahistory,1)
    % Display data and move robot for the current time step
    stepPlayback(handles,1)
    idx= get(handles.text_IOData,'UserData');
    slow_factor= 4;
    pause(slow_factor*(datahistory{idx,1}-datahistory{idx-1,1}))
end


% --- Executes on button press in push_stop.
function push_stop_Callback(hObject, eventdata, handles)
% hObject    handle to push_stop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Clear all playback flags to pause playback
clearAllFlags(handles)

% Restart the playback from the beginning
idx= get(handles.text_IOData,'UserData');
stepPlayback(handles,-(idx-1))


% --- Executes on button press in push_play.
function push_play_Callback(hObject, eventdata, handles)
% hObject    handle to push_play (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Set the Play flag, clear other flags
clearAllFlags(handles)
set(handles.push_play,'UserData',1)

% Get data for timing
datahistory= get(handles.edit_display,'UserData');
idx= get(handles.text_IOData,'UserData');

% Loop while play flag is still set
while exist('hObject','var') && get(handles.push_play,'UserData') ...
        && idx < size(datahistory,1)
    % Display data and move robot for the current time step
    stepPlayback(handles,1)
    idx= get(handles.text_IOData,'UserData');
    pause(datahistory{idx,1}-datahistory{idx-1,1})
end


% --- Executes on button press in push_pause.
function push_pause_Callback(hObject, eventdata, handles)
% hObject    handle to push_pause (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Clear all playback flags to pause playback
clearAllFlags(handles)


% --- Executes on key press with focus on figure_replay or any of its controls.
function figure_replay_WindowKeyPressFcn(hObject, eventdata, handles)
% hObject    handle to figure_replay (see GCBO)
% eventdata  structure with the following fields (see FIGURE)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)

% Check which key is pressed and move time step accordingly
%   Left and up arrows step backwards
%   Right and down arrows step forwards
%   Space, enter, and esc pause
keydown= eventdata.Key;  % Assume only one key pressed at a time
if any(strcmp(keydown,{'leftarrow' 'uparrow'}))
    % Pause playback
    clearAllFlags(handles)
    
    % Get data for checking out of bounds
    idx= get(handles.text_IOData,'UserData');
    
    % Rewind one frame
    if idx > 1
        stepPlayback(handles,-1)
    end
elseif any(strcmp(keydown,{'rightarrow' 'downarrow'}))
    % Pause playback
    clearAllFlags(handles)
    
    % Get data for checking out of bounds
    datahistory= get(handles.edit_display,'UserData');
    idx= get(handles.text_IOData,'UserData');
    
    % Foward one frame
    if idx < size(datahistory,1)
        stepPlayback(handles,1)
    end
elseif any(strcmp(keydown,{'space' 'return' 'escape'})) && ...
        strcmp(get(handles.push_stop,'Enable'),'on')
    % Pause playback
    clearAllFlags(handles)
end


% --- Executes during object creation, after setting all properties.
function edit_display_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_display (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function clearAllFlags(handles)
% Set all playback mode flags to off

set(handles.push_slow_back,'UserData',0)
set(handles.push_slow_fwd,'UserData',0)
set(handles.push_play,'UserData',0)


function stepPlayback(handles,stepSize)
% Display data and move the robot corresponding to the current data index
% and the step size to increase or decrease that index

% Input:
% stepSize - Double, number of data points to move,
%   positive for forward movement, negative for backward

% Get important data and update index
datahistory= get(handles.edit_display,'UserData');
idx= get(handles.text_IOData,'UserData')+stepSize;
set(handles.text_IOData,'UserData',idx)
handles_robot= get(handles.figure_replay,'UserData');
str_disp= get(handles.edit_display,'String');

% Get the new robot position
t= datahistory{idx,1};
x= datahistory{idx,2};
y= datahistory{idx,3};
th= datahistory{idx,4};

% Line approximation of the robot
circ_numPts= 21;    % Estimate circle as circ_numPts-1 lines
circ_ang=linspace(0,2*pi,circ_numPts);
circ_rad=ones(1,circ_numPts)*0.2;
[circ_x circ_y]= pol2cart(circ_ang,circ_rad);
circ_x=circ_x+x;
circ_y=circ_y+y;

% Update robot circle and direction line
set(handles_robot(1),'XData',circ_x)
set(handles_robot(1),'YData',circ_y)
set(handles_robot(2),'XData',[x x+1.5*0.2*cos(th)])
set(handles_robot(2),'YData',[y y+1.5*0.2*sin(th)])

% If in robot-centric view mode move plot focal point
if get(handles.radio_centric,'Value')
    curr_xlimit= get(handles.axes_map,'XLim');
    curr_ylimit= get(handles.axes_map,'YLim');
    half_xdist= (curr_xlimit(2)-curr_xlimit(1))/2;
    half_ydist= (curr_ylimit(2)-curr_ylimit(1))/2;
    set(handles.axes_map,'XLim',[x-half_xdist x+half_xdist])
    set(handles.axes_map,'YLim',[y-half_ydist y+half_ydist])
end

% Define function display string
fcn_str= '';
fcn_data= datahistory{idx,5};
if ischar(fcn_data)             % Single function call this time step
    fcn_str= fcn_data;
elseif iscell(fcn_data)         % Multiple function calls this time step
    fcn_str= fcn_data{1};       % Avoid extra line breaks
    for i= 2:length(fcn_data)
        fcn_str= sprintf('%s  %s',fcn_str,fcn_data{i});
    end
end

% Update the data display in accordance with the new index
if stepSize < 0     % Move backwards
    str_disp= str_disp(1:end+stepSize); % Delete lines from end
elseif stepSize > 0 % Move forwards
    str_disp= [str_disp ; ...
        {sprintf('%.2f %.3f %.3f %.3f %s',t,x,y,th,fcn_str)}];
end
set(handles.edit_display,'String',str_disp)

