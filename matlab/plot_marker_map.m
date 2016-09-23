close all;
clear;
clc;

% load map log
[filename,filepath] = uigetfile('*.csv','Select the extracted marker map');
poseEstimationFile = [filepath,filename];


dim_scale = 10; % original: mm
vector_size = 10;

%% =============================================
%%      load csv file

marker_map = csvread(poseEstimationFile);

sz = size(marker_map);
nr_markers = sz(1);




%% iterate
for i = 1:nr_markers

    %
    marker = marker_map(i,:);
    marker_id = marker(1);
    
    disp( sprintf( 'Drawing marker %d...', marker_id ) )
    
    pos = marker(2:4)/dim_scale;   % cm
    axang = marker(5:8);
    rotm = axang2rotm(axang)


    x_axis = [rotm(1,1), rotm(2,1), rotm(3,1)]*vector_size;
    y_axis = [rotm(1,2), rotm(2,2), rotm(3,2)]*vector_size;
    z_axis = [rotm(1,3), rotm(2,3), rotm(3,3)]*vector_size;

    hold on
    q = quiver3(pos(1),pos(2),pos(3),x_axis(1),x_axis(2),x_axis(3))
    q.Color = 'red';
    q = quiver3(pos(1),pos(2),pos(3),y_axis(1),y_axis(2),y_axis(3))
    q.Color = 'green';
    q = quiver3(pos(1),pos(2),pos(3),z_axis(1),z_axis(2),z_axis(3))
    q.Color = 'blue';
    text(pos(1),pos(2),pos(3),['   ' ...
    num2str(marker_id)],'HorizontalAlignment','left','FontSize',12);

end

% load pose logs
[filename,filepath] = uigetfile('*.csv','Select pose logs');
poseEstimationFile = [filepath,filename];

%% =============================================
%%      load csv file

tracker_data = csvread(poseEstimationFile);

plot3(tracker_data(:,1)/dim_scale, tracker_data(:,2)/dim_scale, tracker_data(:,3)/dim_scale);
hold on;

% axis([-40 20 20 -40 0 40])

zlim([-0.3 40])