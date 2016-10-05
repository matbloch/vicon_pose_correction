%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% @title: Signal Alignment Tool for Vicon Measurements with ROS
% @author: Matthias Bloch
% @version: 1.0
% 
% README:
% .csv file containing pose estimation:
% column format: [X,Y,Z,RotX,RotY,RotZ,W]
% X,Y,Z: Position
% RotX,RotY,RotZ,W: Angle Axis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;
clear;
clc;

%% Example presets

% vicon_file = 'G:\projects_dev\vicon_pose_correction\vicon_pose_measurements.bag';
% poseEstimationFile = 'G:\projects_dev\vicon_pose_correction\pose_estimations.csv';
% vicon_scale = 1000;
% estimation_scale = 1;
% fs_vicon = 100;
% fs_tracker = 30;
% alignment_channel_nr = 3;
% start_time = 18;
% end_time = 293;
% start_time_vicon = 291;
% end_time_vicon = 583;

%% =============================================
%%      select files

% load the rosbag
if ~exist('vicon_file', 'var')
    [filename,filepath] = uigetfile('*.bag','Select the ROS .bag file with the Vicon pose measurements');
    vicon_file = [filepath,filename];
end

% load pose logs
if ~exist('poseEstimationFile', 'var')
    [filename,filepath] = uigetfile('*.csv','Select the pose estimations');
    poseEstimationFile = [filepath,filename];
end

%% =============================================
%%      select signal scaling

if ~exist('vicon_scale', 'var')
    vicon_scale = input('Please enter the position scaling of the Vicon measurements (signal *= s, []: s=1): ');
    if (isempty(vicon_scale) == 1) || vicon_scale == 0
        vicon_scale = 1;
    end
end

if ~exist('estimation_scale', 'var')
    estimation_scale = input('Please enter the position scaling of the Pose estimation (signal *= s, []: s=1): ');
    if (isempty(estimation_scale) == 1) || estimation_scale == 0
        estimation_scale = 1;
    end
end

%% =============================================
%%      select sampling frequency

if ~exist('fs_vicon', 'var')
    % get sampling frequency
    fs_vicon = input('Please enter the sampling frequency of Vicon ([]/0 = calculate sampling frequency): ');

    % calculate sampling frequency
    if (isempty(fs_vicon) == 1) || fs_vicon == 0
        fs_vicon = round((vicon_select.NumMessages-1)/(vicon_select.EndTime - vicon_select.StartTime))
    end
end

% get estimation sampling frequency
if ~exist('fs_tracker', 'var')
    fs_tracker = input('Please enter the sampling frequency of the pose estimation: ');
end

%% =============================================
%%      select rosbag topic

% load rosbag
bag = rosbag(vicon_file);
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
%%      select alignment channel

channels = {'X', 'Y', 'Z'};

if ~exist('alignment_channel_nr', 'var')
    ok = 0;
    while ok ~= 1
    [alignment_channel_nr,ok] = listdlg('PromptString','Select the axis on which you want to align the signals:',...
                    'SelectionMode','single',...
                    'ListString',channels);
        if ok ~= 1
            disp('Please select the Vicon topic');
        end
    end
end
fprintf('Perform alignment along the %s-axis \n', channels{1,alignment_channel_nr});

%% =============================================
%%      build data timeseries
%%	vicon_data: vicon data
%%	tracker_data: pose estimation data
%% =============================================

disp('Building Vicon timeseries...');

% load total transformation
ts = timeseries(vicon_select, ...
'Transform.Translation.X', ...
'Transform.Translation.Y', ...
'Transform.Translation.Z', ...
'Transform.Rotation.X', ...
'Transform.Rotation.Y', ...
'Transform.Rotation.Z', ...
'Transform.Rotation.W' ...
);

% scale position
ts.Data(:,1:3) = ts.Data(:,1:3) * vicon_scale;

vicon_data = ts.Data(:,:);

% load tracker data
tracker_data = csvread(poseEstimationFile);

% scale position
tracker_data(:,1:3) = tracker_data(:,1:3) * estimation_scale;

% resample to same frequency (lower of both)
if fs_vicon > fs_tracker
    % disp('Resampling pose estimation...');
	% [p,q] = rat(fs_vicon / fs_tracker)
    % tracker_sync_dim = resample(tracker_sync_dim,p,q);
    disp('Resampling vicon measurement...');
	[p,q] = rat(fs_tracker / fs_vicon);
	vicon_data = resample(vicon_data,p,q);
else
    % disp('Resampling vicon measurement...');
	% [p,q] = rat(fs_tracker / fs_vicon)
    % vicon_sync_dim = resample(vicon_sync_dim,p,q);
    disp('Resampling pose estimation...');
	[p,q] = rat(fs_vicon / fs_tracker);
    tracker_data = resample(tracker_data,p,q);
end



%% =============================================
%%      load csv file with pose estimations
%%
%%	tracker_data_alignment_roi
%% =============================================


if ~exist('start_time', 'var') || ~exist('end_time', 'var')
    selection_ok = 0;

    f = figure; % new figure window
    set(f,'name','Select pose estimation data range','numbertitle','off')
    while selection_ok ~= 1

        % display original
        plot(tracker_data(:,alignment_channel_nr), 'LineWidth', 3)

        disp('Please select the region of interest of the pose estimation data...');

        % extract region
        disp('Extracting ROI...');
        [x, y] = ginput(2);

        % reselect
        while x(1) == x(2)
            [x, y] = ginput(2);
        end

        start_time = x(1);
        end_time = x(2);

        if x(2) < x(1)
            start_time = x(2);
            end_time = x(1);
        end

        start_time = floor(start_time);
        end_time = ceil(end_time);

        % crop signal
        tracker_data_alignment_roi = tracker_data(start_time:end_time,:);
        % display
        plot(tracker_data_alignment_roi(:,alignment_channel_nr), 'LineWidth', 3)
        title('Extracted pose estimation timeseries')

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
else
    tracker_data_alignment_roi = tracker_data(start_time:end_time,:);
end

%% =============================================
%%      extract Vicon alignment ROI and plot
%%
%%	vicon_ts_alignment_roi
%% =============================================

if ~exist('start_time_vicon', 'var') || ~exist('end_time_vicon', 'var')
    selection_ok = 0;

    % extract the alignment roi
    while selection_ok ~= 1

        % display original
        plot(vicon_data(:,alignment_channel_nr))

        disp('Please select the alignment region of the Vicon data...');

        % select region to extract
        disp('Extracting ROI...');
        [x, y] = ginput(2);

        % reselect
        while x(1) == x(2)
            [x, y] = ginput(2);
        end

        start_time_vicon = x(1);
        end_time_vicon = x(2);

        if x(2) < x(1)
            start_time_vicon = x(2);
            end_time_vicon = x(1);
        end

        start_time_vicon = floor(start_time_vicon);
        end_time_vicon = ceil(end_time_vicon);

        % crop signal
        vicon_ts_alignment_roi = vicon_data(start_time_vicon:end_time_vicon,:);
        % display
        plot(vicon_ts_alignment_roi(:,alignment_channel_nr), 'LineWidth', 3)
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
    
else
    vicon_ts_alignment_roi = vicon_data(start_time_vicon:end_time_vicon,:);
end

%% =============================================
%%      extract synch channel data

% select fitting channel
vicon_sync_dim = vicon_ts_alignment_roi(:,alignment_channel_nr);
tracker_sync_dim = tracker_data_alignment_roi(:,alignment_channel_nr);


% mean shift
tracker_sync_dim = tracker_sync_dim - mean(tracker_sync_dim);
vicon_sync_dim = vicon_sync_dim - mean(vicon_sync_dim);



%% =============================================
%%      start alignment

tracker_data_shifted = 0;

%% start alignement
mx = max(numel(vicon_sync_dim),numel(tracker_sync_dim));

if numel(vicon_sync_dim) > numel(tracker_sync_dim)
    slong = vicon_sync_dim;
    sshort = tracker_sync_dim;
    disp('vicon longer');
else
    slong = tracker_sync_dim;
    sshort = vicon_sync_dim;
    disp('tracker longer');
end


[acor,lag] = xcorr(slong,sshort);
[acormax,I] = max(abs(acor));
lagDiff = lag(I)

f = figure; % new figure window
set(f,'name','Signal Cross-correlation','numbertitle','off')
plot(lag, acor);
hold on;
% draw maximum
plot(lag(I), acormax, 'r*');
line([lag(I) lag(I)], get(gca,'ylim'), 'Color', 'r');
legend('Cross-covariance', 'Maximum Cross-covariance', sprintf('Timeshift: %s',num2str(lag(I))));
xlabel('Lag [samples]');
ylabel('Normalized Cross-correlation [-]');
title('Signal Cross-correlation');
hold off;

%{
Align the signals. Think of the lagging signal as being "longer" than the other, in the sense that you have to "wait longer" to detect it.
    If lagDiff is positive, "shorten" the long signal by considering its elements from lagDiff+1 to the end.
    If lagDiff is negative, "lengthen" the short signal by considering its elements from -lagDiff+1 to the end.
You must add 1 to the lag difference because MATLAB® uses one-based indexing.
%}

if lagDiff > 0
    sorig = sshort; % == tracker
    salign = slong(lagDiff+1:end);  % == vicon
    if numel(vicon_sync_dim) > numel(tracker_sync_dim)
        % slong = vicon_sync_dim
        orig = 'Pose estimation';
        aligned = 'Vicon measurement';
        first_clr = 'r';
        second_clr = 'b';

    else
        % slong = tracker_sync_dim
        orig = 'Vicon measurement';
        aligned = 'Pose estimation';
        first_clr = 'b';
        second_clr = 'r';
        tracker_data_shifted = 1;
    end
else
    sorig = slong;
    salign = sshort(-lagDiff+1:end);
    if numel(vicon_sync_dim) > numel(tracker_sync_dim)
        % sshort = tracker_sync_dim
        orig = 'Vicon measurement';
        aligned = 'Pose estimation';
        first_clr = 'b';
        second_clr = 'r';
        tracker_data_shifted = 1;
    else
        % sshort = vicon_sync_dim
        orig = 'Pose estimation';
        aligned = 'Vicon measurement';
        first_clr = 'r';
        second_clr = 'b';
    end
end


%% =============================================
%%      display sub results (resample results)

f = figure; % new figure window
set(f,'name','Aligned zero mean signals','numbertitle','off')
hold on
plot(sorig, first_clr)
% findpeaks(sorig)
plot(salign, second_clr)
% findpeaks(salign)
xlim([0 mx+1])
title('Aligned signals (alignment ROI)')
ylabel(sprintf('%c-Axis', channels{alignment_channel_nr}));
xlabel('Time')
legend(orig,aligned,'Location','northwest')

%% =============================================
%%      calculate all shifted signals

vicon_data_new = vicon_data;
tracker_data_new = tracker_data;

f = figure; % new figure window

% if alignment range vicon < 
if lagDiff > 0
    if tracker_data_shifted == 1
        % zero padding at front of pose estimate
        tracker_data_new = [zeros(start_time_vicon-lagDiff-start_time,size(tracker_data,2));tracker_data_new];
    else
        % shift vicon to the left = shift tracker to the right
        tracker_data_new = [zeros(start_time_vicon+lagDiff-start_time,size(tracker_data,2));tracker_data_new];
    end
else
    if tracker_data_shifted == 1
        tracker_data_new = [zeros(start_time_vicon+lagDiff-start_time,size(tracker_data,2));tracker_data_new];
    else
        tracker_data_new = [zeros(start_time_vicon-lagDiff-start_time,size(tracker_data,2));tracker_data_new];
    end
end

% display shifted/aligned signals

set(f,'name','Aligned signals','numbertitle','off')
hold on
plot(tracker_data_new(:,alignment_channel_nr)) %% todo: scaling factor
plot(vicon_data_new(:,alignment_channel_nr))

title('Aligned signals')
ylabel('Z-Axis');
xlabel('Time')
legend('Pose estimation','Vicon measurement','Location','northwest')

%% =============================================
%%      CROP OUTPUT
%% =============================================

selection_ok = 0;
%% =============================================
%%      limit the signal extend
f = figure; % new figure window
set(f,'name','Select region to save','numbertitle','off');
while selection_ok ~= 1

    % display original
    plot(tracker_data_new(:,alignment_channel_nr)) %% todo: scaling factor
    hold on
    plot(vicon_data_new(:,alignment_channel_nr))
    title('Aligned signals')
    ylabel(sprintf('%c-Axis', channels{alignment_channel_nr}));
    xlabel('Time')
    legend('Pose estimation','Vicon measurement','Location','northwest')
    hold off

    % extract region
    disp('Selecting the signal range to save...');
    [x, y] = ginput(2);

    start_time = x(1);
    end_time = x(2);
    if x(2) < x(1)
        start_time = x(2);
        end_time = x(1);
    end

    start_time = floor(start_time);
    end_time = ceil(end_time);

    tracker_data_extracted = tracker_data_new(start_time:end_time,:);
    vicon_data_extracted = vicon_data_new(start_time:end_time,:);

    % plot extracted region
    plot(tracker_data_extracted(:,alignment_channel_nr)) %% todo: scaling factor
    hold on
    % findpeaks(sorig)
    plot(vicon_data_extracted(:,alignment_channel_nr))
    % findpeaks(salign)

    title('Aligned signals')
    ylabel('Position');
    xlabel('Time')
    legend('tracker','vicon','Location','northwest')
    hold off

    choice = questdlg('Would you like to reselect the region to save?', ...
        'Save region', ...
        'Save','Reselect region to save', 'cancel');
    % Handle response
    switch choice
        case 'Save'
            selection_ok = 1;
        case 'Reselect region to save'
            disp('Reselection region to save...');
        case 'cancel'
            disp('Reselection region to save...');
    end

end

% crop
tracker_data_new = tracker_data_extracted;
vicon_data_new = vicon_data_extracted;

filename = ['aligned_signals_',datestr(now, 'yyddmm-HHSS'),'.csv'];
min_index = min([size(tracker_data_new,1),size(vicon_data_new,1)]);

%     write titles
%     fid=fopen(filename,'wt');
%     fprintf(fid,'%s,','Vicon');
%     fprintf(fid,'%s,','Pose estimation');
%     fclose(fid);

dlmwrite (filename, [vicon_data_new(1:min_index,:),tracker_data_new(1:min_index,:)], '-append');

fprintf('saving aligned results to: "%s"\n', filename);
