function varargout = ConfigMakerGUI(varargin)
% ConfigMakerGUI M-file for ConfigMakerGUI.fig
% User interface to assist with the making of configuration files for input
%   into the simualator
% To run, enter "ConfigMakerGUI" into the command window with no input or
%   output arguments.  This will bring up the GUI window that allows you to
%   input configuration parameters and export them to a text file.

% ConfigMakerGUI.m
% Copyright (C) 2011 Cornell University
% This code is released under the open-source BSD license.  A copy of this
% license should be provided with the software.  If not, email:
% CreateMatlabSim@gmail.com

% Edit the above text to modify the response to help ConfigMakerGUI

% Last Modified by GUIDE v2.5 19-Jul-2010 13:22:16

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ConfigMakerGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @ConfigMakerGUI_OutputFcn, ...
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


% --- Executes just before ConfigMakerGUI is made visible.
function ConfigMakerGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to ConfigMakerGUI (see VARARGIN)
% UIWAIT makes ConfigMakerGUI wait for user response (see UIRESUME)
% uiwait(handles.figure_config);

% Choose default command line output for ConfigMakerGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);


% --- Outputs from this function are returned to the command line.
function varargout = ConfigMakerGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in push_load.
function push_load_Callback(hObject, eventdata, handles)
% hObject    handle to push_load (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Import configuration file
[filename pathname filter]= uigetfile('*.txt','Import Configuration File');
if filename                         % Make sure cancel was not pressed
    fid= fopen([pathname filename]);% Get file handle for parsing
    
    % Initialize storage variable to no noise
    noise= zeros(6,2);
    
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
        sensors= {'wall' 'cliff' 'odometry' 'sonar' 'lidar' 'camera'};
        % Sensors are in order as in the table
        if length(lineWords) == 2 && strcmp(lineWords{1},'com_delay')
            set(handles.edit_comDelay,'String',lineWords{2})
        elseif length(lineWords) == 3 && any(strcmp(lineWords{1},sensors))
            noise(strcmp(lineWords{1},sensors),1:2)= ...
                [str2double(lineWords{2}) str2double(lineWords{3})];
        elseif ~isempty(lineWords)
            warning('MATLAB:invalidInput',...
                'This line in config file %s is unrecognized:\n\t%s',...
                filename,fline)
        end
    end
    fclose(fid);
    
    % Set noise data to the table
    set(handles.table_noise,'Data',noise)
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
        % Get noise data from the table
        noise= get(handles.table_noise,'Data');         % Matrix of doubles
        comDelay= get(handles.edit_comDelay,'String');  % String
        
        % Deal with empty values
        if isempty(comDelay) || isnan(str2double(comDelay))
            comDelay= '0';
        end

        % Write configuration data to file
        fid= fopen(fileName,'wt');
        sensors= {'wall' 'cliff' 'odometry' 'sonar' 'lidar' 'camera'};
        % Sensors are in order as in the table
        fprintf(fid,'%% SensorName NoiseMean NoiseStandardDev\n');
        for i= 1:size(noise,1)
            fprintf(fid,'%s %.5f %.5f\n',sensors{i},noise(i,1),noise(i,2));
        end
        fprintf(fid,'\ncom_delay %s',comDelay);
        fclose(fid);
    end
end


% --- Executes on button press in edit_comDelay.
function edit_comDelay_Callback(hObject, eventdata, handles)
% hObject    handle to edit_comDelay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Check that value is valid
comDelay= str2double(get(handles.edit_comDelay,'String'));
if comDelay < 0 || isnan(comDelay)
    warning('MATLAB:invalidInput',...
        'Communication delay parameter not valid')
    set(handles.edit_comDelay,'String','0')
end


% --- Executes when entered data in editable cell(s) in table_noise.
function table_noise_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to table_noise (see GCBO)
% eventdata  structure with the following fields (see UITABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. 
%     Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate 
%     value for Data
% handles    structure with handles and user data (see GUIDATA)

% Warn and reset if invalid input (non-numeric)
if isnan(eventdata.NewData)
    warning('MATLAB:invalidInput',...
        'Communication delay parameter not valid')
    noise= get(handles.table_noise,'Data');
    noise(eventdata.Indices(1),eventdata.Indices(2))= 0;
    set(handles.table_noise,'Data',noise)
end
    
