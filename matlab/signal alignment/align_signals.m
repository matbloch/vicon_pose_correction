close all;
clc;

%% =============================================
%%      select paths

% load the rosbag
[filename,filepath] = uigetfile('*.bag','Select the ROS .bag file with the Vicon pose measurements');
viconFile = [filepath,filename];

% load pose logs
[filename,filepath] = uigetfile('*.csv','Select the pose estimations');
poseEstimationFile = [filepath,filename];
%% =============================================
%%      load vicon data

% load rosbag
bag = rosbag(viconFile);
% select topic
topics = bag.AvailableTopics.Properties.RowNames;

if size(bag.AvailableTopics, 1) ~= 1
    
    [topic_nr,ok] = listdlg('PromptString','Select the Vicon topic:',...
                    'SelectionMode','single',...
                    'ListString',topics);
    if ok ~= 1
        disp('Please select the Vicon topic');
        return
    end
else
    topic_nr = 1;
end

vicon_select = select(bag,'Topic',topics(topic_nr)); 
%% =============================================
%%      extract timeseries and plot

disp('Building Vicon timeseries...');
ts = timeseries(vicon_select, 'Transform.Translation.X', 'Transform.Translation.Y', 'Transform.Translation.Z');

selection_ok = 0;
while selection_ok ~= 1
    
    % display original
    plot(ts, 'LineWidth', 3)
    
    disp('Please select the region of interest...');
    
    % extract region
    disp('Extracting ROI...');
    [x, y] = ginput(2);
    ts_extracted = getsampleusingtime(ts,x(1),x(2));
    
    plot(ts_extracted, 'LineWidth', 3)
    title('Extracted Vicon timeseries')

    choice = questdlg('Would you like to reselect the region of interest?', ...
        'ROI selection', ...
        'Continue','Reselect ROI', 'cancel');
    % Handle response
    switch choice
        case 'Continue'
            selection_ok = 1;
        case 'cancel'
            disp('Reselection ROI...');
        case 'Reselect ROI'
            disp('Reselection ROI...');
    end

end

ts = ts_extracted;
vicon_x_coords = ts.Data(:,1);
vicon_y_coords = ts.Data(:,2);
vicon_z_coords = ts.Data(:,3);

% figure
figure % new figure window
disp('Displaying extracted region of interest...');
subplot(4,1,1)
plot(ts, 'LineWidth', 3)
title('Vicon ROI')
subplot(4,1,2)
plot(vicon_x_coords)
title('Pose estimation: X-Axis')
subplot(4,1,3)
plot(vicon_y_coords)
title('Pose estimation: Y-Axis')
subplot(4,1,4)
plot(vicon_z_coords)
title('Pose estimation: Z-Axis')

%% =============================================
%%      get sampling frequencies

% get sampling frequency
fs_vicon = input('Please enter the sampling frequency of Vicon ([]/0 = calculate sampling frequency): ');

% calculate sampling frequency
if (isempty(fs_vicon) == 1) || fs_vicon == 0
    fs_vicon = round((vicon_select.NumMessages-1)/(vicon_select.EndTime - vicon_select.StartTime))
end

% get sampling frequency
fs_tracker = input('Please enter the sampling frequency of the pose estimation: ');


%% =============================================
%%      get sampling frequencies

filepath = fullfile(fileparts(which('align_signals')), 'data', 'VICON0805_20160805_173725_factor100_new.csv');
tracker_data = csvread(filepath);

% use same scale
tracker_x_coords = tracker_data(:,1);
tracker_y_coords = tracker_data(:,2);
tracker_z_coords = tracker_data(:,3);

disp('Resampling pose estimation...');
% resample to same frequency
tracker_z_coords = resample(tracker_z_coords,fs_vicon,fs_tracker);

% moving average filter
% tracker_z_coords = filter(ones(1, 10)/10, 1, tracker_z_coords);



%% =============================================
%%      start alignment

% select output channel
vicon_x_coords = vicon_z_coords;
tracker_x_coords = tracker_z_coords;


%% start alignement
mx = max(numel(vicon_x_coords),numel(tracker_x_coords));

ax(1) = subplot(2,1,1);
stem(vicon_x_coords)
xlim([0 mx+1])

ax(2) = subplot(2,1,2);
stem(tracker_x_coords)
xlim([0 mx+1])

if numel(vicon_x_coords) > numel(tracker_x_coords)
    slong = vicon_x_coords;
    sshort = tracker_x_coords;
else
    slong = tracker_x_coords;
    sshort = vicon_x_coords;
end


[acor,lag] = xcorr(slong,sshort);
[acormax,I] = max(abs(acor));
lagDiff = lag(I)

%{

% plot cross correlation
figure
stem(lag,acor)
hold on
plot(lagDiff,acormax)
hold off
%}

%{
Align the signals. Think of the lagging signal as being "longer" than the other, in the sense that you have to "wait longer" to detect it.
    If lagDiff is positive, "shorten" the long signal by considering its elements from lagDiff+1 to the end.
    If lagDiff is negative, "lengthen" the short signal by considering its elements from -lagDiff+1 to the end.
You must add 1 to the lag difference because MATLAB® uses one-based indexing.

%}

if lagDiff > 0
    sorig = sshort;
    salign = slong(lagDiff+1:end);
else
    sorig = slong;
    salign = sshort(-lagDiff+1:end);
end

figure
subplot(2,1,1)
stem(sorig)
xlim([0 mx+1])

subplot(2,1,2)
stem(salign)
xlim([0 mx+1])

%% ------------
% 
% 
% [x1,x2] = alignsignals(vicon_x_coords,tracker_x_coords);
% 
% subplot(2,1,1)
% stem(x1)
% xlim([0 mx+1])
% 
% subplot(2,1,2)
% stem(x2,'*')
% xlim([0 mx+1])



