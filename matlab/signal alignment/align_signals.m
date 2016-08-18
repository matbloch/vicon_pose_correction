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
%% =============================================
%%      select paths

% load the rosbag
[filename,filepath] = uigetfile('*.bag','Select the ROS .bag file with the Vicon pose measurements');
viconFile = [filepath,filename];

% load pose logs
[filename,filepath] = uigetfile('*.csv','Select the pose estimations');
poseEstimationFile = [filepath,filename];

%% =============================================
%%      select signal scaling

vicon_scale = input('Please enter the position scaling of the Vicon measurements (signal *= s, []: s=1): ');
if (isempty(vicon_scale) == 1) || vicon_scale == 0
    vicon_scale = 1;
end

estimation_scale = input('Please enter the position scaling of the Pose estimation (signal *= s, []: s=1): ');
if (isempty(estimation_scale) == 1) || estimation_scale == 0
    estimation_scale = 1;
end

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
%%      select alignment channel

channels = {'X', 'Y', 'Z'};
ok = 0;
while ok ~= 1
[alignment_channel_nr,ok] = listdlg('PromptString','Select the axis on which you want to align the signals:',...
                'SelectionMode','single',...
                'ListString',channels);
    if ok ~= 1
        disp('Please select the Vicon topic');
    end
end

fprintf('Perform alignment along the %s-axis \n', channels{1,alignment_channel_nr});

%% =============================================
%%      extract timeseries and plot (alignment axis only)

disp('Building Vicon timeseries...');

% geometry_msgs/TransformStamped topics
% TransformStamped_topics = { ...
%     'Transform.Translation.X', ...
%     'Transform.Translation.Y', ...
%     'Transform.Translation.Z', ...
%     'Transform.Rotation.X', ...
%     'Transform.Rotation.Y', ...
%     'Transform.Rotation.Z', ...
%     'Transform.Rotation.W' ...
% };
% 
% ts = timeseries(vicon_select, TransformStamped_topics{1,alignment_channel_nr});

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

% ts.TimeInfo.Units = 'seconds';

selection_ok = 0;
while selection_ok ~= 1
    
    % display original

%     plot(get(ts.Data(), 'Transform.Translation.X'), 'LineWidth', 3)
    plot(ts.Time, ts.Data(:,alignment_channel_nr))
    
    disp('Please select the region of interest of the Vicon data...');
    
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
    
    % crop signal
    ts_extracted = getsampleusingtime(ts,start_time_vicon,end_time_vicon);
    % display
    plot(ts_extracted.Time, ts_extracted.Data(:,alignment_channel_nr), 'LineWidth', 3)
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

% % ts = ts_extracted;
% vicon_x_coords_extracted = ts_extracted.Data(:,1);
% vicon_y_coords_extracted = ts_extracted.Data(:,2);
% vicon_z_coords_extracted = ts_extracted.Data(:,3);
% 
% % figure
% figure % new figure window
% disp('Displaying extracted region of interest...');
% subplot(4,1,1)
% plot(ts, 'LineWidth', 3)
% title('Vicon ROI')
% subplot(4,1,2)
% plot(vicon_x_coords_extracted)
% title('X-Axis')
% subplot(4,1,3)
% plot(vicon_y_coords_extracted)
% title('Y-Axis')
% subplot(4,1,4)
% plot(vicon_z_coords_extracted)
% title('Z-Axis')

%% =============================================
%%      load csv file

tracker_data = csvread(poseEstimationFile);

% scale position
tracker_data(:,1:3) = estimation_scale * tracker_data(:,1:3);

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
    
    start_time = x(1);
    end_time = x(2);
    
    % reselect
    while x(1) == x(2)
        [x, y] = ginput(2);
    end
    
    if x(2) < x(1)
        start_time = x(2);
        end_time = x(1);
    end
    
    start_time = floor(start_time);
    end_time = ceil(end_time);
    
    tracker_data_extracted = tracker_data(start_time:end_time,:);
    
    plot(tracker_data_extracted(:,alignment_channel_nr), 'LineWidth', 3)
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



%% =============================================
%%      select fitting channel

% select fitting channel
vicon_dim = ts_extracted.Data(:,alignment_channel_nr);
tracker_dim = tracker_data_extracted(:,alignment_channel_nr);

%% =============================================
%%      filtering/smoothing
%{
 De-noise with edge-preserving non-linear Medial Filters:

This is an important step. 
The objective here is to smooth out your energy envelopes, but without destruction or smoothing out your edges and fast rise times. 
There is actually an entire field devoted to this, but for our purposes here, we can simply use an easy to implement non-linear Medial
 filter. (Median Filtering). This is a powerful technique because unlike mean filtering, medial filtering will not null out your edges,
 but at the same time 'smooth' out your signal without significant degradation of the important edges, 
since at no time is any arithmetic being performed on your signal (provided the window length is odd).
%}

% tracker_data(:,3) = medfilt1(tracker_data(:,3),20);
% tracker_data(:,3) = medfilt1(tracker_data(:,3),20);
% tracker_data(:,3) = medfilt1(tracker_data(:,3),20);


% moving average filter
% tracker_z_coords = filter(ones(1, 20)/20, 1, tracker_z_coords);


%% =============================================
%%      mean shift signals

% % pose estimation
% tracker_mean = trimmean(tracker_data_extracted,10);
% tracker_data_extracted = bsxfun(@minus, tracker_data_extracted,tracker_mean);
% % vicon
% vicon_x_coords_extracted = vicon_x_coords - mean(vicon_x_coords);
% vicon_y_coords_extracted = vicon_y_coords - mean(vicon_y_coords);
% vicon_z_coords_extracted = vicon_z_coords - mean(vicon_z_coords);

tracker_dim = tracker_dim - mean(tracker_dim);
vicon_dim = vicon_dim - mean(vicon_dim);

%% =============================================
%%      resampling and filtering

% get sampling frequency
fs_vicon = input('Please enter the sampling frequency of Vicon ([]/0 = calculate sampling frequency): ');

% calculate sampling frequency
if (isempty(fs_vicon) == 1) || fs_vicon == 0
    fs_vicon = round((vicon_select.NumMessages-1)/(vicon_select.EndTime - vicon_select.StartTime))
end

% get estimation sampling frequency
fs_tracker = input('Please enter the sampling frequency of the pose estimation: ');

% resample to same frequency (lower of both)
if fs_vicon > fs_tracker
    % disp('Resampling pose estimation...');
	% [p,q] = rat(fs_vicon / fs_tracker)
    % tracker_dim = resample(tracker_dim,p,q);
    disp('Resampling vicon measurement...');
	[p,q] = rat(fs_tracker / fs_vicon);
    vicon_dim = resample(vicon_dim,p,q);
else
    % disp('Resampling vicon measurement...');
	% [p,q] = rat(fs_tracker / fs_vicon)
    % vicon_dim = resample(vicon_dim,p,q);
    disp('Resampling pose estimation...');
	[p,q] = rat(fs_vicon / fs_tracker);
    tracker_dim = resample(tracker_dim,p,q);
end

%% =============================================
%%      start alignment

tracker_data_shifted = 0;

%% start alignement
mx = max(numel(vicon_dim),numel(tracker_dim));

if numel(vicon_dim) > numel(tracker_dim)
    slong = vicon_dim;
    sshort = tracker_dim;
    disp('vicon longer');
else
    slong = tracker_dim;
    sshort = vicon_dim;
end

%{
xcorr normalizes the maximal value of the correlation (the value at zero) to 1
xcorr(x,y) returns the cross-correlation of two discrete-time sequences, x and y. 
Cross-correlation measures the similarity between x and shifted (lagged) copies of y as a function of the lag. 
If x and y have different lengths, the function appends zeros at the end of the shorter 
vector so it has the same length, N, as the other.
The cross-correlation of the two measurements is maximum at a lag equal to the delay.
corrcoef: same as xcorr, subtracts mean
%}

% subtract mean from each signal to get normlaized cross-correlation
% coefficient, which is invariant to change of amplitude and bias of
% signals
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
    sorig = sshort; % == tracker
    salign = slong(lagDiff+1:end);  % == vicon
    if numel(vicon_dim) > numel(tracker_dim)
        % slong = vicon_dim
        orig = 'Pose estimation';
        aligned = 'Vicon measurement';
        first_clr = 'r';
        second_clr = 'b';

    else
        % slong = tracker_dim
        orig = 'Vicon measurement';
        aligned = 'Pose estimation';
        first_clr = 'b';
        second_clr = 'r';
        tracker_data_shifted = 1;
    end
else
    sorig = slong;
    salign = sshort(-lagDiff+1:end);
    if numel(vicon_dim) > numel(tracker_dim)
        % sshort = tracker_dim
        orig = 'Vicon measurement';
        aligned = 'Pose estimation';
        first_clr = 'b';
        second_clr = 'r';
        tracker_data_shifted = 1;
    else
        % sshort = vicon_dim
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
title('Prealigned signals')
ylabel('Z-Axis [m]');
xlabel('Time')
legend(orig,aligned,'Location','northwest')

%% =============================================
%%      calculate all shifted signals

% reload vicon data - now load all topics

% ts = timeseries(vicon_select, ...
% 'Transform.Translation.X', ...
% 'Transform.Translation.Y', ...
% 'Transform.Translation.Z', ...
% 'Transform.Rotation.X', ...
% 'Transform.Rotation.Y', ...
% 'Transform.Rotation.Z', ...
% 'Transform.Rotation.W' ...
% );
% ts_extracted = getsampleusingtime(ts,start_time_vicon,end_time_vicon);
    
	
% resample tracker signal
% vicon_data_new = ts_extracted.Data(:,:);
% tracker_data_new = resample(tracker_data_extracted,fs_vicon,fs_tracker);


vicon_data_new = ts_extracted.Data(:,:);


if fs_vicon > fs_tracker
	% [p,q] = rat(fs_vicon / fs_tracker)
    % tracker_dim = resample(tracker_dim,p,q);
	[p,q] = rat(fs_tracker / fs_vicon);
	% resample vicon to tracker frequency
	vicon_data_new = resample(ts_extracted.Data,p,q);
	tracker_data_new = tracker_data_extracted;
else
	% [p,q] = rat(fs_tracker / fs_vicon)
    % vicon_dim = resample(vicon_dim,p,q);
	[p,q] = rat(fs_vicon / fs_tracker);
	tracker_data_new = resample(tracker_data_extracted,p,q);
	vicon_data_new = ts_extracted.Data(:,:);
end


if lagDiff > 0
    if tracker_data_shifted == 1
        tracker_data_new = tracker_data_new(lagDiff+1:end,:);
    else
        vicon_data_new = vicon_data_new(lagDiff+1:end,:);
    end
else
    if tracker_data_shifted == 1
        tracker_data_new = tracker_data_new(-lagDiff+1:end,:);
    else
        vicon_data_new = vicon_data_new(-lagDiff+1:end,:);
    end
end

% display shifted/aligned signals
f = figure; % new figure window
set(f,'name','Aligned signals','numbertitle','off')
hold on
plot(tracker_data_new(:,3)) %% todo: scaling factor
% findpeaks(sorig)
plot(vicon_data_new(:,3))
% findpeaks(salign)
xlim([0 mx+1])
title('Aligned signals')
ylabel('Z-Axis [m]');
xlabel('Time')
legend('Pose estimation','Vicon measurement','Location','northwest')

%% =============================================
%%      save results

choice = questdlg('Would to exit now and save the results?', ...
    'Exit and save aligned signals', ...
    'Save and Exit','Continue', 'cancel');
save_and_exit = 0;
switch choice
    case 'Save and Exit'
        save_and_exit = 1;
    case 'cancel'
%         disp('Reselection ROI...');
    case 'Continue'
%         disp('Reselection ROI...');
end


%% =============================================
%%      save data

if save_and_exit == 1
   
    selection_ok = 0;
    %% =============================================
    %%      limit the signal extend
    f = figure; % new figure window
    set(f,'name','Select region to save','numbertitle','off');
    while selection_ok ~= 1
    
        % display original
        plot(tracker_data_new(:,1:3)) %% todo: scaling factor
        hold on
        % findpeaks(sorig)
        plot(vicon_data_new(:,1:3))
        % findpeaks(salign)
        xlim([0 mx+1])
        title('Aligned signals')
        ylabel('Z-Axis [m]');
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
        plot(tracker_data_extracted(:,1:3)) %% todo: scaling factor
        hold on
        % findpeaks(sorig)
        plot(vicon_data_extracted(:,1:3))
        % findpeaks(salign)
        xlim([0 mx+1])
        title('Aligned signals')
        ylabel('Position [m]');
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
end

%% =============================================
%%      maximum alignment


% selection_ok = 0;
% while selection_ok ~= 1
%     
%     % display after prealignment
%     hold on
%     plot(sorig, first_clr)
%     findpeaks(sorig)
%     plot(salign, second_clr)
%     findpeaks(salign)
%     hold off
%     xlim([0 mx+1])
%     title('Prealigned signals')
%     ylabel('Z-Axis [m]');
%     xlabel('Time')
%     legend(first,second,'Location','northwest')
%     
%     disp('Please select a region where maximum alignment is performed...');
%     
%     % extract region
%     disp('Extracting region...');
%     [x, y] = ginput(2);
%     
%     start_time = x(1)
%     end_time = x(2)
%     if x(2) < x(1)
%         start_time = x(2);
%         end_time = x(1);
%     end
%     
%     start_time = floor(start_time);
%     end_time = ceil(end_time);
%     
% 
%     % calculate maximas
%     
%     % choose largest
%     
%     % align on maximas
%     
%     % display result
%     
%     
%     choice = questdlg('Would you like to reselect the region of interest?', ...
%         'ROI selection', ...
%         'Continue','Reselect ROI', 'cancel');
%     % Handle response
%     switch choice
%         case 'Continue'
%             selection_ok = 1;
%         case 'cancel'
%             disp('Reselection ROI...');
%         case 'Reselect ROI'
%             disp('Reselection ROI...');
%     end
% 
% end



%% ------------
% 
% 
% [x1,x2] = alignsignals(vicon_dim,tracker_dim);
% 
% subplot(2,1,1)
% stem(x1)
% xlim([0 mx+1])
% 
% subplot(2,1,2)
% stem(x2,'*')
% xlim([0 mx+1])

%% =============================================
%%      save data
% 
% filename = ['aligned_signals_',datestr(now, 'yyddmm-HHMMSS'),'.csv'];
% 
% min_index = min([numel(sorig),numel(salign)]);
% fid=fopen(filename,'wt');
% fprintf(fid,'%s,','Vicon');
% fprintf(fid,'%s,','Pose estimation');
% fclose(fid);
% 
% % vicon first, then pose estimation
% if strcmp(first,'Vicon') == 1
%     dlmwrite (filename, [sorig(1:min_index,:),salign(1:min_index,:)], '-append');
% else
%     dlmwrite (filename, [salign(1:min_index,:), sorig(1:min_index,:)], '-append');
% end
% 
% fprintf('saving aligned results to: "%s"\n', filename);
% 
% 


