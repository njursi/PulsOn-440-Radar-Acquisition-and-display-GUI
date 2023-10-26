function varargout = Radar_demo(varargin)
% RADAR_DEMO MATLAB code for Radar_demo.fig
%      RADAR_DEMO, by itself, creates a new RADAR_DEMO or raises the existing
%      singleton*.
%
%      H = RADAR_DEMO returns the handle to a new RADAR_DEMO or the handle to
%      the existing singleton*.
%
%      RADAR_DEMO('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in RADAR_DEMO.M with the given input arguments.
%
%      RADAR_DEMO('Property','Value',...) creates a new RADAR_DEMO or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Radar_demo_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Radar_demo_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Radar_demo

% Last Modified by GUIDE v2.5 26-Oct-2023 20:59:50

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Radar_demo_OpeningFcn, ...
                   'gui_OutputFcn',  @Radar_demo_OutputFcn, ...
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


% --- Executes just before Radar_demo is made visible.
function Radar_demo_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Radar_demo (see VARARGIN)

% Choose default command line output for Radar_demo
handles.output = hObject;
set(handles.edit6,'String',cd);
axes(handles.axes1);
imshow(imread('Logo.jpeg'));
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Radar_demo wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Radar_demo_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


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



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double


% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
a=uigetdir;
set(handles.edit6,'String',a);
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
global scanRaw 
savepath=[get(handles.edit6,'String'),'\',get(handles.edit7,'String')];
  % dlmwrite(savepath, scanRaw, '-append');
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
global savepath scanRaw
savepath=[get(handles.edit6,'String'),'\',get(handles.edit7,'String')];
mrmIpAddr = '192.168.1.100';
%mrmIpAddr = '192.168.1.100';
scanStartPs = str2num(get(handles.edit1,'String')); % Adjust this to match antenna delay
C_mps = 299792458;
maxDistance_m = str2num(get(handles.edit2,'String'));  % MRM will quantize to closest above this number
scanStopPs = scanStartPs + (2*maxDistance_m/C_mps)*1e12; % 1e12 ps in one sec
pulseIntegrationIndex = str2num(get(handles.edit3,'String')); % The number of pulses per scan point
transmitGain = str2num(get(handles.edit4,'String')); % Tx power (0 for FCC legal)
scanIntervalTime_ms = str2num(get(handles.edit5,'String')); % Time between start of each scan in millisecs
scanRes_ps = 61;
;


restart = 0; stop = 0; done = 0;


%% Open a socket for communicating with the MRM
sckt = sckt_mgr('get');
if isempty(sckt)
    sckt_mgr('open');
end


%% Get the configuration of the MRM
get_cfg_rqst(mrmIpAddr,1)
[msg,msgType,msgID,mrmIpAddr] = read_pckt;
if isempty(msgID)
    error('Unable to communicate with the MRM')
end
[CFG,msgType,msgID] = parse_msg(msg);


%% Update the config structure & send to the MRM
% The MRM API specifies the variable types
CFG.scanStartPs = uint32(scanStartPs);
CFG.scanStopPs = uint32(scanStopPs);
CFG.pulseIntegrationIndex = uint16(pulseIntegrationIndex); % The higher this is the higher the snr but longer it takes to scan
CFG.transmitGain = uint8(transmitGain);
set_cfg_rqst(mrmIpAddr,2,CFG);

%% Read the confirm from the MRM
[msg,msgType,msgID,mrmIpAddr] = read_pckt;
if ~strcmp(msgType,'1101') % MRM_SET_CONFIG_CONFIRM
    error(fprintf('Invalid message type: %s, should have been 1101',msgType));
end


%% Command radar to scan designated number of scans (-1 continuous)
scanCount = 1;
CTL.scanCount = uint16(scanCount); % 2^16-1 for continuous
CTL.reserved = uint16(0); % Aligns to word
CTL.scanIntervalTime = uint32(scanIntervalTime_ms*1000); % Microsecs between scan starts
axes(handles.axes2);
loopI = 0;  % This is the loop and scan index
while (~done)
    tStart = tic;
    loopI = loopI + 1;

    % Implement restart button behavior
    if restart == 1
        loopI = 1;
        restart = 0;
    end

    % Allows you to setup before collecting the first reference scan
    % Point the radar into open space.
    if loopI == 1
        pause(1)
    end

    %% Request a scan and read it back
    % Request a scan
    ctl_rqst(mrmIpAddr,msgID,CTL)
    % Read the confirm
    [msg,msgType,msgID,mrmIpAddr] = read_pckt;
    % Read the first scan msg.  Analyze the hdr for how many follow
    [msg,msgType,msgID,mrmIpAddr] = read_pckt;
    [scanInfo,msgType,msgID] = parse_msg(msg);
    scanRaw = double(scanInfo.scanData);  % Save the good stuff. Append to this later
    % Loop, reading the entire waveform scan into scanDataSaved
    for j = 1:scanInfo.numberMessages-1
        [msg,msgType,msgID,mrmIpAddr] = read_pckt;
        [scanInfo,msgType,msgID] = parse_msg(msg);
        scanRaw = [scanRaw, double(scanInfo.scanData)];
    end
    dlmwrite(savepath, scanRaw, '-append');
    %% Process the scan depending on the loop iteration
    %
    % In iteration 1 save the scan as a reference template
    % Note: the first scan is lousy for some reason, so changed to scan2
    if loopI == 2;
        %maxMag = max(abs(scanRaw));
        scanScaleFactor = 100.0/max(abs(scanRaw));
        scanTemplate = scanScaleFactor*scanRaw;
        distanceAxis_m = ([0:length(scanRaw)-1]*scanRes_ps/1e12)*C_mps/2;  % scanIndex*(61ps/step)/(ps/sec)*(meters/sec)/2 (round trip)
        hold off;
        plot(distanceAxis_m,scanTemplate,'Color',[0.5 0.5 0.5]);
        axis tight;
        xlabel('Distance (m)');
        ylabel('Signal Amplitude');
    end

    % On iteration 2 compute and plot the delta between raw and template
    % Low-pass filter the absolute value to estimate an envelope
    if loopI == 3
        scanLen = length(scanRaw);
        scanDelta = abs(scanScaleFactor*scanRaw - scanTemplate);
        scanEnvelope = movingAvg(scanDelta);
        minThreshold = 3*[scanEnvelope + std(scanDelta)];
        scanThreshold = (100./[1:scanLen].^0.6) + minThreshold; % add 1/r^alpha. adjust alpha until it looks right.
        hold on
        plot(distanceAxis_m,scanThreshold,'r--');
        hEnv = plot(distanceAxis_m,scanEnvelope,'b');
        hDetList = plot(distanceAxis_m(1:scanLen),zeros(1,scanLen),'r.');  % Do this to set up detection list update
        legend('Raw Scan','Threshold','Enveloped Scan','Detection List');
    end

    % From now on compute and plot delta scan and detection list
    if loopI >= 4
        scanDelta = abs(scanScaleFactor*scanRaw - scanTemplate);
        scanEnvelope = fir_lpf_ord5(scanDelta);
        detectionI = find(scanEnvelope > scanThreshold);
        detectionV = scanEnvelope(detectionI);

        set(hEnv,'YData',scanEnvelope);
        set(hDetList,'XData',distanceAxis_m(detectionI),'YData',detectionV);

        tElapsed = toc(tStart)*1000;
        
        % set(number,'string',sprintf('%3.1fms',tElapsed));

        if length(detectionI) >= 3
            distance1 = distanceAxis_m(detectionI(1));
            sigStr1 = sum(scanEnvelope(detectionI(1:3)));
            set(handles.edit8,'String',[num2str(distance1),'    ',num2str(sigStr1)]);
            fprintf('Distance: %3.2f, Magnitude: %3.2f\n',distance1, sigStr1);
        end

    end

    drawnow;

end


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
close(Radar_demo);



function edit8_Callback(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit8 as text
%        str2double(get(hObject,'String')) returns contents of edit8 as a double


% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
