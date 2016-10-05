%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% @title: Plotting tool for marker tracker measurements
% @author: Matthias Bloch
% @version: 1.0
% 
% README:
% Pose logs:
% .csv file containing pose estimation from vicon and from tracker
% column format: 
% [X,Y,Z,RotX,RotY,RotZ,W]_vicon, [-],
% [X,Y,Z,RotX,RotY,RotZ,W]_tracker
% X,Y,Z: Position
% RotX,RotY,RotZ,W: Angle Axis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;
clear;
clc;

% load map log
[filename,filepath] = uigetfile('*.csv','Select the extracted marker map');
poseEstimationFile = [filepath,filename];

dim_scale = 10; % original: mm
vector_size = 10;

%% =============================================
%%      Draw marker map

marker_map = csvread(poseEstimationFile);
sz = size(marker_map);
nr_markers = sz(1);

%% iterate
for i = 1:nr_markers
    marker = marker_map(i,:);
    marker_id = marker(1);
    disp( sprintf( 'Drawing marker %d...', marker_id ) );
    
    % extract position and rotation matrix
    pos = marker(2:4)/dim_scale;   % cm
    axang = marker(5:8);
    rotm = axang2rotm(axang);
    
    % render marker
    hold on
    renderPose(pos, rotm, vector_size);
    text(pos(1),pos(2),pos(3),['   ' ...
    num2str(marker_id)],'HorizontalAlignment','left','FontSize',12);
end

% set aspect ration 1:1
daspect([1 1 1])

%% =============================================
%%      Draw reference marker map

% Construct a questdlg with three options
choice = questdlg('Would you like to load the reference map?', ...
	'Load marker dataset', ...
	'Yes','No','No');
% Handle response
load_another_ds = 0;
switch choice
    case 'Yes'
        load_another_ds = 1;
    case 'No'
        load_another_ds = 0;
end

% load map log
if(load_another_ds)
    [filename,filepath] = uigetfile('*.csv','Select the extracted marker map');
    poseEstimationFile = [filepath,filename];

    marker_map = csvread(poseEstimationFile);
    sz = size(marker_map);
    nr_markers = sz(1);

    %% iterate
    for i = 1:nr_markers
        marker = marker_map(i,:);
        marker_id = marker(1);
        disp( sprintf( 'Drawing marker %d...', marker_id ) );

        % extract position and rotation matrix
        pos = marker(2:4)/dim_scale;   % cm
        axang = marker(5:8);
        rotm = axang2rotm(axang);

        % render marker
        hold on
        renderPose(pos, rotm, vector_size);
    end

end

%% =============================================
%%      Load tracker pose logs

% Construct a questdlg with three options
choice = questdlg('Would you like to load the synchronized pose logs?', ...
	'Load pose logs', ...
	'Yes','No','No');
% Handle response
load_camera_logs = 0;
switch choice
    case 'Yes'
        load_camera_logs = 1;
    case 'No'
        load_camera_logs = 0;
end

C={'a', 'b', 'c'};

if(load_camera_logs)
    [filename,filepath] = uigetfile('*.csv','Select pose logs');
    poseEstimationFile = [filepath,filename];
    tracker_data = csvread(poseEstimationFile);
    
    % draw vicon tracks
    s1 = scatter3(tracker_data(:,1)/dim_scale, tracker_data(:,2)/dim_scale, tracker_data(:,3)/dim_scale,'filled');
    % draw tracker position estimation
    s2 = scatter3(tracker_data(:,9)/dim_scale, tracker_data(:,10)/dim_scale, tracker_data(:,11)/dim_scale,'filled');
    legend([s1, s2], {'Vicon', 'Marker Tracker'});
    hold on;
end