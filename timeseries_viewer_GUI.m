function varargout = timeseries_viewer_GUI(varargin)
% TIMESERIES_VIEWER_GUI MATLAB code for timeseries_viewer_GUI.fig
%      TIMESERIES_VIEWER_GUI, by itself, creates a new TIMESERIES_VIEWER_GUI or raises the existing
%      singleton*.
%
%      H = TIMESERIES_VIEWER_GUI returns the handle to a new TIMESERIES_VIEWER_GUI or the handle to
%      the existing singleton*.
%
%      TIMESERIES_VIEWER_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TIMESERIES_VIEWER_GUI.M with the given input arguments.
%
%      TIMESERIES_VIEWER_GUI('Property','Value',...) creates a new TIMESERIES_VIEWER_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before timeseries_viewer_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to timeseries_viewer_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help timeseries_viewer_GUI

% Last Modified by GUIDE v2.5 16-Aug-2016 13:53:59

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @timeseries_viewer_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @timeseries_viewer_GUI_OutputFcn, ...
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


% --- Executes just before timeseries_viewer_GUI is made visible.
function timeseries_viewer_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to timeseries_viewer_GUI (see VARARGIN)

% Choose default command line output for timeseries_viewer_GUI
handles.output = hObject;


% defaultProfile = parallel.defaultClusterProfile;
% myCluster = parcluster(defaultProfile);
% maxWorkers = myCluster.NumWorkers;
% localRun(2) = min(feature('numcores'), maxWorkers);
% disp(' ');
% disp([num2str(localRun(2)) ' CPU cores were detected and will be allocated for parallel processing.']);
% disp(' ');
% % handles.poolObj=parpool('local',localRun(2));
% matlabpool(localRun(2));

handles.currStack=1;
handles.cAxis=[0 3200];
% 
% handles.timeListener=addlistener(handles.slider1,'Action',@slider1_Callback);
% handles.zListener=addlistener(handles.slider2,'Action',@set_z_label);
handles.timeListener=addlistener(handles.slider1,'ContinuousValueChange',@slider1_Callback);
handles.zListener=addlistener(handles.slider2,'ContinuousValueChange',@set_z_label);

handles.flTimeSeries.data = [];
handles.spPos = [];
handles.radii = [];

handles.currentDataFile = [];
handles.currView=[];

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes timeseries_viewer_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = timeseries_viewer_GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles=guidata(hObject);
currPos = round(get(hObject,'Value')+1);
set(handles.text2,'String',sprintf('Stack %03d',currPos));
xlim = get(handles.axes1,'XLim');
ylim = get(handles.axes1,'YLim');
set(handles.axes1,'XLim',xlim,'YLim',ylim,'YDir','normal');
handles.showFrame = imshow(handles.currView(:,:,currPos),'Parent',handles.axes1,'InitialMagnification','fit');
caxis(handles.axes1,handles.cAxis);
set(handles.axes1,'XLim',xlim,'YLim',ylim,'YDir','normal');

currentSpot = str2num(get(handles.spot_selection,'String'));
microns_per_slice = str2num(get(handles.zRes,'String'));
currZ = round(get(handles.slider2,'Value')+1);
if currentSpot > 0
    current_position = handles.spPos(currentSpot,:);
    
    slice_position = round(handles.spPos(currentSpot,3)/microns_per_slice);

    if currZ == slice_position
        try
            delete(handles.cellCircle);
        end
        pixel_center = round(handles.spPos(currentSpot,1:2)/0.41);
        handles.cellCircle = viscircles(handles.axes1,pixel_center,handles.radii(1)/0.41,'EdgeColor','r','linewidth',1);
    end
end

drawnow;
pause(.005);
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% handles=guidata(hObject);
%NOTE for upgrade: CHANGE TO ALLOW HANDLING OF .STACK FILES AND
%BACKTRACKING OF SPOTS
stackDirectory=uigetdir(pwd,'Select processed (.tiff or .klb) directory');
set(handles.status_bar,'String','Loading...');
set(handles.slider2,'enable','off');
set(handles.slider1,'enable','off');
drawnow;
cd(stackDirectory);
handles.currDirectory=stackDirectory;
if get(handles.file_tyep_selector,'Value') == 1
    handles.stackfiles = dir('*.klb');
else
    handles.stackfiles = dir('*.tif');
end
stackdata = readImage(handles.stackfiles(1).name);
handles.zSize = size(stackdata,3);
handles.tSize = numel(handles.stackfiles);

currZ = floor(handles.zSize/2);

set(handles.slider1,'Max',handles.tSize-1,'SliderStep',[1/handles.tSize,5/handles.tSize]);
set(handles.slider2,'Max',handles.zSize-1,'SliderStep',[1/handles.zSize,10/handles.zSize],'Value',currZ);


stackName = sprintf('ts%03d',currZ);

handles.curr_fullStack = readImage(handles.stackfiles(1).name);
handles.curr_fullStack_idx = 1;

fs=filesep;

if ~exist(['tsView',fs,stackName,'.tif'],'file')
    disp('Beginning read...');
    lsStackFiles=handles.stackfiles;
%     parfor t=1:handles.tSize
        fullStack = readImage(lsStackFiles(1).name);
        currView = fullStack(:,:,currZ);   
%         disp(sprintf('Processed frame %03d',t));
%     end    
%     mkdir('tsView');
%     writeImage(currView,['tsView',fs,stackName,'.tif']);
%     handles.fullStack = curr_fullStack;
    handles.currView = currView;
    disp('tsView Not Found. Please Use Gen All Slices!');
else
%     tic
%     handles.currView = readImage(['tsView',fs,stackName,'.tif']);
%     toc
    handles.currView = fast_readTiffSTACK(handles.currView, ['tsView',fs,stackName,'.tif'],[size(handles.curr_fullStack,1),size(handles.curr_fullStack,2),handles.tSize]);
end
    handles.showFrame = imshow(handles.currView(:,:,1),'Parent',handles.axes1);
    set(handles.axes1,'YDir','normal');
    caxis(handles.axes1,handles.cAxis);
    
    set(handles.text2,'String','Stack 001');
    set(handles.text1,'String', sprintf('Z %03d',currZ));
    
set(handles.status_bar,'String','Ready');
set(handles.slider2,'enable','on');
set(handles.slider1,'enable','on');
drawnow;

tsFiles = dir('*ls_fluorescence_time_series.mat');

if numel(tsFiles) == 1
    handles = load_time_series_data(tsFiles.name, handles);
end

guidata(hObject,handles);
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
set(handles.status_bar,'String','Loading...');
set(handles.slider2,'enable','off');
set(handles.slider1,'enable','off');
drawnow;
    
currZ = round(get(hObject,'Value')+1);
set(handles.text1,'String',sprintf('Z %03d',currZ));

stackName = sprintf('ts%03d',currZ);
fs=filesep;

if ~exist(['tsView',fs,stackName,'.tif'],'file')
    disp('Please use GENERATE ALL SLICE MOVIES button!');
%     lsStackFiles=handles.stackfiles;
%     parfor t=1:handles.tSize
%         fullStack = readImage(lsStackFiles(t).name);
%         currView(:,:,t) = fullStack(:,:,currZ);        
%         disp(sprintf('Processed frame %03d',t));
%     end    
%     mkdir('tsView');
%     writeImage(currView,['tsView/',stackName,'.klb']);
%     handles.currView = currView;
%     disp('Done!');
else
%     tic
%     handles.currView = readImage(['tsView/',stackName,'.tif']);
%     toc
   handles.currView = fast_readTiffSTACK(handles.currView,['tsView',fs,stackName,'.tif'],[size(handles.curr_fullStack,1),size(handles.curr_fullStack,2),handles.tSize]);
    set(handles.slider2,'enable','on');
    set(handles.slider1,'enable','on');
    set(handles.status_bar,'String','Ready');
end
%     oldAxes = get(handles.axes1,'
%     xlim = get(handles.axes1,'XLim');
%     ylim = get(handles.axes1,'YLim');
    handles.showFrame = imshow(handles.currView(:,:,round(get(handles.slider1,'Value')+1)),'Parent',handles.axes1);
    set(handles.axes1,'YDir','normal');
%     set(handles.axes1,'XLim',xlim,'YLim',ylim,'YDir','normal');
    caxis(handles.axes1,handles.cAxis);


drawnow;
guidata(hObject,handles)
    

function set_z_label(hObject, eventdata, handles)
handles=guidata(hObject);
currZ = round(get(hObject,'Value')+1);
set(handles.text1,'String',sprintf('Z %03d',currZ));

if handles.curr_fullStack_idx == round(get(handles.slider1,'Value')+1)
    xlim = get(handles.axes1,'XLim');
    ylim = get(handles.axes1,'YLim');
    handles.showFrame = imshow(handles.curr_fullStack(:,:,currZ),'Parent',handles.axes1);
    set(handles.axes1,'XLim',xlim,'YLim',ylim,'YDir','normal');
    caxis(handles.axes1,handles.cAxis);
end
% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on selection change in file_tyep_selector.
function file_tyep_selector_Callback(hObject, eventdata, handles)
% hObject    handle to file_tyep_selector (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns file_tyep_selector contents as cell array
%        contents{get(hObject,'Value')} returns selected item from file_tyep_selector


% --- Executes during object creation, after setting all properties.
function file_tyep_selector_CreateFcn(hObject, eventdata, handles)
% hObject    handle to file_tyep_selector (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double
handles.cAxis = [str2num(get(hObject,'String')) handles.cAxis(2)];
caxis(handles.axes1,handles.cAxis);
guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double
handles.cAxis = [handles.cAxis(1) str2num(get(hObject,'String'))];
caxis(handles.axes1,handles.cAxis);
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in loadZ.
function loadZ_Callback(hObject, eventdata, handles)
% hObject    handle to loadZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles=guidata(hObject);
newIdx =round(get(handles.slider1,'Value')+1);
if handles.curr_fullStack_idx ~= newIdx
    set(handles.slider2,'enable','off');
    set(handles.slider1,'enable','off');
    set(handles.status_bar,'String','Loading...');
    drawnow;
    
    handles.curr_fullStack_idx = newIdx;
    handles.curr_fullStack = readImage(handles.stackfiles(newIdx).name);
    set(handles.status_bar,'String','Ready');
    set(handles.slider2,'enable','on');
    set(handles.slider1,'enable','on');
    drawnow;
end
set(handles.axes1,'XLimMode','manual','YLimMode','manual');
guidata(hObject,handles);


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles=guidata(hObject);
[file,path] = uigetfile('*.mat','Select fluorescence time series data');
handles.currentDataFile = [path,file];
handles=load_time_series_data([path file],handles);
set_flSpot_display(handles);

guidata(hObject,handles);


function spot_selection_Callback(hObject, eventdata, handles)
% hObject    handle to spot_selection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of spot_selection as text
%        str2double(get(hObject,'String')) returns contents of spot_selection as a double
handles=guidata(hObject);

currZ = round(get(handles.slider2,'Value')+1);
microns_per_slice = str2num(get(handles.zRes,'String'));

currentSpot = str2num(get(handles.spot_selection,'String'));

if currentSpot == 0
    try
        delete(handles.cellCircle);
    end
    return
end

set_flSpot_display(handles);

current_position = handles.spPos(currentSpot,:);
    
slice_position = round(handles.spPos(currentSpot,3)/microns_per_slice);

if currZ == slice_position
    try
        delete(handles.cellCircle);
    end
    pixel_center = round(handles.spPos(currentSpot,1:2)/0.41);
    handles.cellCircle = viscircles(handles.axes1,pixel_center,handles.radii(1)/0.41,'EdgeColor','r','linewidth',1);
end

guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function spot_selection_CreateFcn(hObject, eventdata, handles)
% hObject    handle to spot_selection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function zRes_Callback(hObject, eventdata, handles)
% hObject    handle to zRes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of zRes as text
%        str2double(get(hObject,'String')) returns contents of zRes as a double


% --- Executes during object creation, after setting all properties.
function zRes_CreateFcn(hObject, eventdata, handles)
% hObject    handle to zRes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function returnHandles = load_time_series_data(file,handles)

load(file);
returnHandles = handles;
returnHandles.flTimeSeries = fluorescence_time_series;
returnHandles.spPos = spPos;
returnHandles.radii = spRadiiXYZ(1,:);
    
function set_flSpot_display(handles)
%sets the display on fluorescence time series
numspots = size(handles.flTimeSeries,1);
currentSpot = str2num(get(handles.spot_selection,'String'));
microns_per_slice = str2num(get(handles.zRes,'String'));

if currentSpot>0
    slice_position = round(handles.spPos(currentSpot,3)/microns_per_slice);
    current_position = handles.spPos(currentSpot,:);
else
    slice_position = -1;
    current_position = [0,0,0];
end

display_string = sprintf('Current Spot:%3.0f\nlocation: (%4.2f, %4.2f, %4.2f)\nZ Slice: %4.0f\n\nTotal # Spots: %4.0d \n\nSpot Radii (microns): %2.2f XY, %2.2f Z \n',...
                          currentSpot,current_position(1),current_position(2),current_position(3),slice_position,numspots,handles.radii(1),handles.radii(3));
set(handles.time_series_display,'String',display_string);


% --- Executes on button press in GenAllSlices_Button.
function GenAllSlices_Button_Callback(hObject, eventdata, handles)
% hObject    handle to GenAllSlices_Button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%NOTE: change this to create .tif files and then append each one per slice
%for each stack load. this will get rid of the outer for loop....

handles=guidata(hObject);

set(handles.status_bar,'String','Loading...');
set(handles.slider2,'enable','off');
set(handles.slider1,'enable','off');
drawnow;

cd(handles.currDirectory);

mkdir('tsView');
lsStackFiles=handles.stackfiles;
fs=filesep;
% for z=1:handles.zSize
%     stackName = sprintf('ts%03d',z);
%     if ~exist(['tsView',fs,stackName,'.klb'],'file')
%         disp(sprintf('Beginning read of slice %4.0f...',z));
% 
%         parfor t=1:handles.tSize
%             fullStack = readImage(lsStackFiles(t).name);
%             currView(:,:,t) = fullStack(:,:,z);        
%             disp(sprintf('Processed frame %03d',t));
%         end    
%         mkdir('tsView');
%         writeImage(currView,['tsView/',stackName,'.klb']);
% %         handles.currView = currView;
%         disp('Done!');
% %     else    
% %         handles.currView = readImage(['tsView/',stackName,'.klb']);
%     end
% end
cd('tsView');

for z=1:handles.zSize
    stackName = sprintf('ts%03d',z);
    tiffName{z} = [stackName,'.tif'];

    if exist(tiffName{z},'file');
       delete(tiffName{z});
    end
    
%     tiffObj{z} = Tiff(tiffName,'a');

end

for t=1:handles.tSize
    fullStack = readImage(['..\',lsStackFiles(t).name]);
    
    if t==1 %detect whether uint8 or uint16
        if isa(fullStack,'uint8')
            BitsPerSample = 8;
        elseif isa(fullStack,'uint16')
            BitsPerSample = 16; 
        end
    end
    
    for m=1:handles.zSize
%         objt = tiffObj{m};

        tiffObj = Tiff(tiffName{m},'a');
        
        tiffObj.setTag('Photometric',Tiff.Photometric.LinearRaw);
        tiffObj.setTag('BitsPerSample',BitsPerSample);
        tiffObj.setTag('ImageWidth',size(handles.currView,2));
        tiffObj.setTag('ImageLength',size(handles.currView,1));
        tiffObj.setTag('SamplesPerPixel',1);
        tiffObj.setTag('Compression',Tiff.Compression.PackBits);
        tiffObj.setTag('PlanarConfiguration',Tiff.PlanarConfiguration.Chunky);
    
        tiffObj.write( fullStack(:,:,m) );        
    
    end
    
    if t==handles.tSize
        writeDirectory(tiffObj);
        tiffObj.close();
    end
    
    disp(sprintf('Processed frame %03d',t));
end    

currZ = floor(handles.zSize/2);
stackName = sprintf('ts%03d',currZ);

handles.currView = fast_readTiffSTACK(handles.currView, [stackName,'.tif'],[size(handles.curr_fullStack,1),size(handles.curr_fullStack,2),handles.tSize]);

cd('..');
set(handles.status_bar,'String','Ready');
set(handles.slider2,'enable','on');
set(handles.slider1,'enable','on');
drawnow;
guidata(hObject,handles);

%probably not in fact faster
function stackImg = fast_readTiffSTACK(stackImg,tiffName,stackDimsXYT)
%fast read of tsView files
if ~isequal(size(stackImg),stackDimsXYT)
    stackImg = zeros(stackDimsXYT(1),stackDimsXYT(2),stackDimsXYT(3));
end
tic
parfor k=1:stackDimsXYT(3)
%     tic
    stackImg(:,:,k) = imread(tiffName,'Index',k,'PixelRegion',{[1 stackDimsXYT(1)],[1,stackDimsXYT(2)]});
%     toc
end
toc