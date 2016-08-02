close all;
clc;

%% load the rosbag

filepath = fullfile(fileparts(which('align_signals')), 'data', 'map_0.bag');
bag = rosbag(filepath);
bag.AvailableTopics;
% bag.MessageList(500:505,:)

% select message
topic_name = '/vicon/SMOUNT_MB/SMOUNT_MB';
vicon_select = select(bag,'Topic',topic_name);  % select based on topic

% read the data
% msgs = readMessages(vicon_select);
% size(msgs)


%% extract timeseries and plot
ts = timeseries(vicon_select, 'Transform.Translation.X', 'Transform.Translation.Y');

vicon_x_coords = ts.Data(:,1);
vicon_x_coords = ts.Data(:,2);
% figure
% plot(ts, 'LineWidth', 3)


%% load tracker data

filepath = fullfile(fileparts(which('align_signals')), 'data', 'map_0.csv');
tracker_data = csvread(filename);
    
tracker_x_coords = tracker_data(:,1);
tracker_y_coords = tracker_data(:,2);

%% start alignement

mx = max(numel(vicon_x_coords),numel(tracker_x_coords));

ax(1) = subplot(2,1,1);
stem(vicon_x_coords)
xlim([0 mx+1])

ax(2) = subplot(2,1,2);
stem(tracker_x_coords,'*')
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

figure
stem(lag,acor)
hold on
plot(lagDiff,acormax,'*')
hold off

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

subplot(2,1,1)
stem(sorig)
xlim([0 mx+1])

subplot(2,1,2)
stem(salign,'*')
xlim([0 mx+1])

