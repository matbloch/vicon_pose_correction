close all;
clear;
clc;

%% =============================================
%%      select paths

% load pose logs
[filename,filepath] = uigetfile('*.csv','Select the pose estimations');
poseEstimationFile = [filepath,filename];

%% =============================================
%%      load csv file

% filepath = fullfile(fileparts(which('align_signals')), 'data', 'VICON0805_20160805_173725_factor100_new.csv');
tracker_data = csvread(poseEstimationFile);

selection_ok = 0;

figure % new figure window
while selection_ok ~= 1
    
    % display original
    plot(tracker_data(:,1:3), 'LineWidth', 3)
    
    disp('Please select the region of interest of the pose estimation data...');
    
    % extract region
    disp('Extracting ROI...');
    [x, y] = ginput(2);
    
    start_time = x(1)
    end_time = x(2)
    if x(2) < x(1)
        start_time = x(2);
        end_time = x(1);
    end
    
    start_time = floor(start_time);
    end_time = ceil(end_time);
    
    tracker_data_extracted = tracker_data(start_time:end_time,1:3);
    
    plot(tracker_data_extracted, 'LineWidth', 3)
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


tracker_data = tracker_data_extracted;

%% =============================================
%%      mean shift signals

mymean = trimmean(tracker_data,10)

% pose estimation
tracker_data = bsxfun(@minus, tracker_data,mymean);


figure
plot(tracker_data(:,3))
return

%% =============================================
%%      filtering/smooting

% use same scale?
tracker_x_coords = tracker_data(:,1)/10;
tracker_y_coords = tracker_data(:,2)/10;
tracker_z_coords = tracker_data(:,3)/10+0.01;

tracker_z_coords = medfilt1(tracker_z_coords,20);
tracker_z_coords = medfilt1(tracker_z_coords,20);
tracker_z_coords = medfilt1(tracker_z_coords,20);



