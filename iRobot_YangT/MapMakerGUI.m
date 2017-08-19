function varargout = MapMakerGUI(varargin)
% MapMakerGUI M-file for MapMakerGUI.fig
% User interface to assist with the making of mapfiles for input into the
%   simulator
% To run, enter "MapMakerGUI" into the command window with no input or
%   output arguments.  This will bring up the GUI window that will allow
%   you to create a map and export the information to a text file.

% MapMakerGUI.m
% Copyright (C) 2011 Cornell University
% This code is released under the open-source BSD license.  A copy of this
% license should be provided with the software.  If not, email:
% CreateMatlabSim@gmail.com

% Edit the above text to modify the response to help MapMakerGUI

% Last Modified by GUIDE v2.5 27-Jan-2011 12:41:11

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @MapMakerGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @MapMakerGUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
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


% --- Executes just before MapMakerGUI is made visible.
function MapMakerGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to MapMakerGUI (see VARARGIN)

% UIWAIT makes MapMakerGUI wait for user response (see UIRESUME)
% uiwait(handles.figure_mapmaker);

% Choose default command line output for MapMakerGUI
handles.output = hObject;

% Add in axes toolbar
set(hObject,'toolbar','figure');

% Initialize field to store map data with comments to improve readability
mapMat= {'% File containing map information' ;
    sprintf('\n') ;
    '% Formatting:' ;
    '% wall x1 y1 x2 y2' ;
    '%   Order does not matter between the points';
    '% line x1 y1 x2 y2' ;
    '% beacon x y [r g b] ID_tag' ;
    '%   [r g b] is the red-green-blue color vector' ;
    '% virtwall x y theta' ;
    '%   Virtual walls emit from a location, not like real walls' ;
    '%   theta is the angle relative to the positive x-axis' ;
    sprintf('\n')};
set(handles.figure_mapmaker,'UserData',mapMat);

% Update handles structure
guidata(hObject, handles);


% --- Outputs from this function are returned to the command line.
function varargout = MapMakerGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in push_clear.
function push_clear_Callback(hObject, eventdata, handles)
% hObject    handle to push_clear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Check that map should be cleared
button= questdlg('Are you sure you want to clear all map data?',...
    'Verify Clear','Yes','Cancel','Cancel');

% If so then clear the plot and delete data in the matrix
if strcmp(button,'Yes')
    children= get(gca,'Children');
    if ~isempty(children)
        delete(children)             % Deletes from plot
        mapMat= {'% File containing map information' ;
            sprintf('\n') ;
            '% Formatting:' ;
            '% wall x1 y1 x2 y2' ;
            '%   Order does not matter between the points';
            '% line x1 y1 x2 y2' ;
            '% beacon x y [r g b] ID_tag' ;
            '%   [r g b] is the red-green-blue color vector' ;
            '% virtwall x y theta' ;
            '%   Virtual walls emit from a location, not like real walls' ;
            '%   theta is the angle relative to the positive x-axis' ;
            sprintf('\n')};
        set(handles.figure_mapmaker,'UserData',mapMat);
    end
end


% --- Executes on button press in push_undo.
function push_undo_Callback(hObject, eventdata, handles)
% hObject    handle to push_undo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Undo most recent plotting operation
children= get(gca,'Children');
if ~isempty(children)
    delete(children(1))             % Deletes from plot
    
    % Delete last line of map matrix
    mapMat= get(handles.figure_mapmaker,'UserData');% Get map data matrix
    last_line= mapMat{end};
    if strcmp(last_line(1:6),'beacon')
        delete(children(2))         % Delete 2-part plotted items
    elseif strcmp(last_line(1:8),'virtwall')
        delete(children(2:3))        % Delete 3-part plotted items
    end
    mapMat= mapMat(1:end-1);
    set(handles.figure_mapmaker,'UserData',mapMat)  % Set map data matrix
end


% --- Executes on button press in push_save.
function push_save_Callback(hObject, eventdata, handles)
% hObject    handle to push_save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Pull up dialogue box to get filename
fileName= inputdlg('Filename (no extension):','Save Map Data');
if ~isempty(fileName)
    fileName= [fileName{1} '.txt']; % Convert cell array and add extension
    
    % Check if file is already in existence
    button= 'Yes';
    if exist(fileName,'file')
        button= questdlg('A file by that name exists.  Overwrite?',...
            'Verify Overwrite','Yes','Cancel','Cancel');
    end
    
    if strcmp(button,'Yes')
        % Write map data to file
        mapMat= get(handles.figure_mapmaker,'UserData');
        fid= fopen(fileName,'wt');
        for i= 1:size(mapMat,1)
            fprintf(fid,'%s\n',mapMat{i});
        end
        fclose(fid);
    end
end


% --- Executes on button press in push_load.
function push_load_Callback(hObject, eventdata, handles)
% hObject    handle to push_load (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Import map file
[filename pathname filter]= uigetfile('*.txt','Import Map File');
if filename                         % Make sure cancel was not pressed
    fid= fopen([pathname filename]);% Get file handle for parsing
    
    % Clear old map
    axes(handles.axes_map)
    children= get(gca,'Children');  % All ploted items on axes
    if ~isempty(children)
        delete(children)	% Delete all but robot from axes
    end
    mapMat= {[]};
    
   % Parse the file and extract relevant information
    walls= [];
    lines= [];
    beacs= {};
    vwalls= [];
    while ~feof(fid)
        line= fgetl(fid);
        line= lower(line);  % Convert to lowercase if necessary
        line= strtrim(line);    % Delete leading and trailing whitespace
        mapMat= [mapMat ; line];% Save to map cell array
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
                filename,line)
        end
    end
    fclose(fid);
    
    % Store map data, replacing old data
    set(handles.figure_mapmaker,'UserData',mapMat)
    
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


% --- Executes on button press in push_wall_line.
function push_wall_line_Callback(hObject, eventdata, handles)
% hObject    handle to push_wall_line (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

mapMat= get(handles.figure_mapmaker,'UserData');    % Get data cell array
[x y]= ginput(2);                   % Accept two mouse clicks
if ~isempty(x) && length(x) == 2    % Check that two points were clicked
    xLimits= xlim(handles.axes_map);
    yLimits= ylim(handles.axes_map);
    if all(x >= xLimits(1)) && all(x <= xLimits(2)) && ...
            all(y >= yLimits(1)) && all(y <= yLimits(2))
        mapMat= [mapMat ; ...
            sprintf('wall %.3f %.3f %.3f %.3f',x(1),y(1),x(2),y(2))];
        axes(handles.axes_map)
        plot(x,y,'k-','LineWidth',1)
        set(handles.figure_mapmaker,'UserData',mapMat)
    else
        warning('MATLAB:ginput',...
            'Point clicked is out of bounds.  Action has been cancelled.')
    end
end


% --- Executes on button press in push_wall_cont.
function push_wall_cont_Callback(hObject, eventdata, handles)
% hObject    handle to push_wall_cont (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Plot continuous (non-breaking) walls
axes(handles.axes_map)
mapMat= get(handles.figure_mapmaker,'UserData');    % Get data cell array
[x1 y1 button]= ginput(1);          % Accept initial point

% End point acceptance when keyboard button is pressed
while ~isempty(button) && button >= 1 && button <= 3
    [x2 y2 button]= ginput(1);      % Accept second point in line
    if ~isempty(button) && button >= 1 && button <= 3
        xLimits= xlim(handles.axes_map);
        yLimits= ylim(handles.axes_map);
        if all([x1 x2] >= xLimits(1)) && all([x1 x2] <= xLimits(2)) && ...
                all([y1 y2] >= yLimits(1)) && all([y1 y2] <= yLimits(2))
            mapMat= [mapMat ; ...
                sprintf('wall %.3f %.3f %.3f %.3f',x1,y1,x2,y2)];
            plot([x1 x2],[y1 y2],'k-','LineWidth',1)
            x1= x2;     % Save second point for next line starting point
            y1= y2;
        else
            warning('MATLAB:ginput',...
                'Point clicked is out of bounds.  Action has been cancelled.')
        end
    end
end
set(handles.figure_mapmaker,'UserData',mapMat)      % Set data cell array


% --- Executes on button press in push_wall_square.
function push_wall_square_Callback(hObject, eventdata, handles)
% hObject    handle to push_wall_square (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get corner data and add to map matrix
mapMat= get(handles.figure_mapmaker,'UserData');    % Get map data matrix
[x y]= ginput(2);   % Accept two mouse clicks
if ~isempty(x) && length(x) == 2    % Check that two points were clicked
    xLimits= xlim(handles.axes_map);
    yLimits= ylim(handles.axes_map);
    if all(x >= xLimits(1)) && all(x <= xLimits(2)) && ...
            all(y >= yLimits(1)) && all(y <= yLimits(2))
        mapMat= [mapMat ; ...
            sprintf('wall %.3f %.3f %.3f %.3f',x(1),y(1),x(1),y(2)) ; ...
            sprintf('wall %.3f %.3f %.3f %.3f',x(1),y(2),x(2),y(2)) ; ...
            sprintf('wall %.3f %.3f %.3f %.3f',x(2),y(2),x(2),y(1)) ; ...
            sprintf('wall %.3f %.3f %.3f %.3f',x(2),y(1),x(1),y(1))];
        axes(handles.axes_map)
        % Split up plot commands so that Undo will only take out one line
        plot([x(1) x(1)],[y(1) y(2)],'k-','LineWidth',1)
        plot([x(1) x(2)],[y(2) y(2)],'k-','LineWidth',1)
        plot([x(2) x(2)],[y(2) y(1)],'k-','LineWidth',1)
        plot([x(2) x(1)],[y(1) y(1)],'k-','LineWidth',1)
        set(handles.figure_mapmaker,'UserData',mapMat)
    else
        warning('MATLAB:ginput',...
            'Point clicked is out of bounds.  Action has been cancelled.')
    end
end


% --- Executes on button press in push_line_line.
function push_line_line_Callback(hObject, eventdata, handles)
% hObject    handle to push_line_line (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

mapMat= get(handles.figure_mapmaker,'UserData');    % Get data cell array
[x y]= ginput(2);                   % Accept two mouse clicks
if ~isempty(x) && length(x) == 2    % Check that two points were clicked
    xLimits= xlim(handles.axes_map);
    yLimits= ylim(handles.axes_map);
    if all(x >= xLimits(1)) && all(x <= xLimits(2)) && ...
            all(y >= yLimits(1)) && all(y <= yLimits(2))
        mapMat= [mapMat ; ...
            sprintf('line %.3f %.3f %.3f %.3f',x(1),y(1),x(2),y(2))];
        axes(handles.axes_map)
        plot(x,y,'k--','LineWidth',1)
        set(handles.figure_mapmaker,'UserData',mapMat)
    else
        warning('MATLAB:ginput',...
            'Point clicked is out of bounds.  Action has been cancelled.')
    end
end


% --- Executes on button press in push_line_cont.
function push_line_cont_Callback(hObject, eventdata, handles)
% hObject    handle to push_line_cont (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Plot continuous (non-breaking) lines
axes(handles.axes_map)
mapMat= get(handles.figure_mapmaker,'UserData');    % Get data cell array
[x1 y1 button]= ginput(1);          % Accept initial point

% End point acceptance when keyboard button is pressed
while ~isempty(button) && button >= 1 && button <= 3
    [x2 y2 button]= ginput(1);      % Accept second point in line
    if ~isempty(button) && button >= 1 && button <= 3
        xLimits= xlim(handles.axes_map);
        yLimits= ylim(handles.axes_map);
        if all([x1 x2] >= xLimits(1)) && all([x1 x2] <= xLimits(2)) && ...
                all([y1 y2] >= yLimits(1)) && all([y1 y2] <= yLimits(2))
            mapMat= [mapMat ; ...
                sprintf('line %.3f %.3f %.3f %.3f',x1,y1,x2,y2)];
            plot([x1 x2],[y1 y2],'k--','LineWidth',1)
            x1= x2;     % Save second point for next line starting point
            y1= y2;
        else
            warning('MATLAB:ginput',...
                'Point clicked is out of bounds.  Action has been cancelled.')
        end
    end
end
set(handles.figure_mapmaker,'UserData',mapMat)      % Set data cell array


% --- Executes on button press in push_line_square.
function push_line_square_Callback(hObject, eventdata, handles)
% hObject    handle to push_line_square (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get corner data and add to map matrix
mapMat= get(handles.figure_mapmaker,'UserData');    % Get map data matrix
[x y]= ginput(2);   % Accept two mouse clicks
if ~isempty(x) && length(x) == 2    % Check that two points were clicked
    xLimits= xlim(handles.axes_map);
    yLimits= ylim(handles.axes_map);
    if all(x >= xLimits(1)) && all(x <= xLimits(2)) && ...
            all(y >= yLimits(1)) && all(y <= yLimits(2))
        mapMat= [mapMat ; ...
            sprintf('line %.3f %.3f %.3f %.3f',x(1),y(1),x(1),y(2)) ; ...
            sprintf('line %.3f %.3f %.3f %.3f',x(1),y(2),x(2),y(2)) ; ...
            sprintf('line %.3f %.3f %.3f %.3f',x(2),y(2),x(2),y(1)) ; ...
            sprintf('line %.3f %.3f %.3f %.3f',x(2),y(1),x(1),y(1))];
        axes(handles.axes_map)
        % Split up plot commands so that Undo will only take out one line
        plot([x(1) x(1)],[y(1) y(2)],'k--','LineWidth',1)
        plot([x(1) x(2)],[y(2) y(2)],'k--','LineWidth',1)
        plot([x(2) x(2)],[y(2) y(1)],'k--','LineWidth',1)
        plot([x(2) x(1)],[y(1) y(1)],'k--','LineWidth',1)
        set(handles.figure_mapmaker,'UserData',mapMat)
    else
        warning('MATLAB:ginput',...
            'Point clicked is out of bounds.  Action has been cancelled.')
    end
end


% --- Executes on button press in push_beacon.
function push_beacon_Callback(hObject, eventdata, handles)
% hObject    handle to push_beacon (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get input data and plot beacon
mapMat= get(handles.figure_mapmaker,'UserData');    % Get map data matrix
[x y]= ginput(1);

% Check for valid input
if ~isempty(x)
    xLimits= xlim(handles.axes_map);
    yLimits= ylim(handles.axes_map);
    if x >= xLimits(1) && x <= xLimits(2) && ...
            y >= yLimits(1) && y <= yLimits(2)
        % Figure out which radio button is selected for color
        if get(handles.radio_beacon_black,'Value')
            col= get(handles.radio_beacon_black,'BackgroundColor');
        elseif get(handles.radio_beacon_white,'Value')
            col= get(handles.radio_beacon_white,'BackgroundColor');
        elseif get(handles.radio_beacon_red,'Value')
            col= get(handles.radio_beacon_red,'BackgroundColor');
        elseif get(handles.radio_beacon_blue,'Value')
            col= get(handles.radio_beacon_blue,'BackgroundColor');
        elseif get(handles.radio_beacon_magenta,'Value')
            col= get(handles.radio_beacon_magenta,'BackgroundColor');
        elseif get(handles.radio_beacon_cyan,'Value')
            col= get(handles.radio_beacon_cyan,'BackgroundColor');
        elseif get(handles.radio_beacon_green,'Value')
            col= get(handles.radio_beacon_green,'BackgroundColor');
        else
            col= get(handles.radio_beacon_yellow,'BackgroundColor');
        end
        
        % Update data and plot
        id= get(handles.edit_beacon,'String');
        mapMat= [mapMat ; ...
            sprintf('beacon %.3f %.3f [%.1f %.1f %.1f] %s',...
            x,y,col(1),col(2),col(3),id)];
        axes(handles.axes_map)
        plot(x,y,'Marker','o','Color',col)
        text(x,y,['  ' id])     % Add spaces to stop clutter
        set(handles.figure_mapmaker,'UserData',mapMat)
        
        % Increment beacon ID if unedited
        if ~get(handles.edit_beacon,'UserData')
            idtext= get(handles.edit_beacon,'String');
            idtext(1)= [];  % Delete 'b'
            idnum= str2double(idtext);
            set(handles.edit_beacon,'String',['b' num2str(idnum+1)])
        end
    else
        warning('MATLAB:ginput',...
            'Point clicked is out of bounds.  Action has been cancelled.')
    end
end


% --- Executes during object creation, after setting all properties.
function edit_beacon_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_beacon (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), ...
        get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_beacon_Callback(hObject, eventdata, handles)
% hObject    handle to edit_beacon (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Set UserData to 1 to show that the beacon ID has been edited by the user
set(handles.edit_beacon,'UserData',1)

% Set text color to black
set(handles.edit_beacon,'ForegroundColor',[0 0 0])


% --- Executes on button press in push_vwall.
function push_vwall_Callback(hObject, eventdata, handles)
% hObject    handle to push_vwall (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get virtual wall data and add to map matrix
mapMat= get(handles.figure_mapmaker,'UserData');    % Get data cell array
[x y]= ginput(2);   % Accept two mouse clicks

% Define virtual wall emitter constants
halo_rad= 0.45;     % Radius of the halo around the emitter
range_short= 2.13;  % Range of the wall on the 0-3' setting
ang_short= 0.33;    % Angular range on the 0-3' setting
range_med= 5.56;    % Range of the wall on the 4'-7' setting
ang_med= 0.49;      % Angular range on the 4'-7' setting
range_long= 8.08;   % Range of the wall on the 8'+ setting
ang_long= 0.61;     % Angular range on the 8'+ setting

% Get range data from radio buttons
if get(handles.radio_vwall_low,'Value')     % 0-3' range
    range= 1;
    range_dist= range_short;
    ang= ang_short;
elseif get(handles.radio_vwall_med,'Value') % 4'-7' range
    range= 2;
    range_dist= range_med;
    ang= ang_med;
else                                        % 8'+ range
    range= 3;
    range_dist= range_long;
    ang= ang_long;
end

if ~isempty(x) && length(x) == 2    % Check that two points were clicked
    xLimits= xlim(handles.axes_map);
    yLimits= ylim(handles.axes_map);
    if x(1) >= xLimits(1) && x(1) <= xLimits(2) && ...
            y(1) >= yLimits(1) && y(1) <= yLimits(2)
        % Calculate direction of virtual wall emission
        th= atan2(y(2)-y(1),x(2)-x(1));
        
        % Get endpoints of wall range triangle
        x_1= x(1)+range_dist*cos(th+ang/2);
        y_1= y(1)+range_dist*sin(th+ang/2);
        x_2= x(1)+range_dist*cos(th-ang/2);
        y_2= y(1)+range_dist*sin(th-ang/2);
        
        % Plot halo around emitter and range triangle
        circ_numPts= 21;    % Estimate circle as circ_numPts-1 lines
        circ_ang=linspace(0,2*pi,circ_numPts);
        circ_rad=ones(1,circ_numPts)*halo_rad;
        [circ_x circ_y]= pol2cart(circ_ang,circ_rad);
        circ_x= circ_x+x(1);
        circ_y= circ_y+y(1);
        plot(x(1),y(1),'g*')
        plot(circ_x,circ_y,'g:','LineWidth',1);
        plot([x(1) x_1 x_2 x(1)],[y(1) y_1 y_2 y(1)],'g:','LineWidth',1)
        
        % Save virtual wall data to map
        mapMat= [mapMat ; ...
            sprintf('virtwall %.3f %.3f %.3f %.0f',x(1),y(1),th,range)];
        set(handles.figure_mapmaker,'UserData',mapMat)
    else
        warning('MATLAB:ginput',...
            'Point clicked is out of bounds.  Action has been cancelled.')
    end
end


% --- Executes on key press with focus on figure_mapmaker or any of its controls.
function figure_mapmaker_WindowKeyPressFcn(hObject, eventdata, handles)
% hObject    handle to figure_mapmaker (see GCBO)
% eventdata  structure with the following fields (see FIGURE)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)

% Extract keypress information
keydown= eventdata.Key;
mod= eventdata.Modifier;
% Enable Ctrl+z to undo
if strcmp(keydown,'z') && length(mod) == 1 && strcmp(mod,'control')
    push_undo_Callback(handles.push_undo,[],handles)
end
