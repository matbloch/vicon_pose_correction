close all;
clc;

filepath = fullfile(fileparts(which('align_signals')), 'data', 'map_0.bag');


% load the rosbag
bag = rosbag(filepath);
bag.AvailableTopics;
% bag.MessageList(500:505,:)

% select message
topic_name = '/vicon/SMOUNT_MB/SMOUNT_MB';
vicon_select = select(bag,'Topic',topic_name);  % select based on topic


% read the data
% msgs = readMessages(vicon_select);
% size(msgs)

% msgs{2}
% msgs{2}.Transform.Translation.Z

% 
ts = timeseries(vicon_select, 'Transform.Translation.X', 'Transform.Translation.Y', 'Transform.Translation.Z');

figure
plot(ts, 'LineWidth', 3)


% ts.Data

% .Translation, .Rotation (Quaternion)

% plot
% ts = timeseries(bagMsgs, 'Pose.Pose.Position.X');
% ts = timeseries(vicon_select)

% vicon_poses = readMessages(vicon_select)



% odom = timeseries(bag,'Pose.Pose.Position.X','Pose.Pose.Position.Y');


% tform.Transform.Translation



% msgs = bag.readAll({topic1});
% 
% fprintf('Read %i messages\n', length(msgs));
% 
% bagselect2 = select(bag, 'Time', ...
% [bag.StartTime bag.StartTime + 1], 'Topic', topic_name)




break

sz = 30;

% generate random signal: length:
sg = randn(1,randi(8)+10);   % min length 3
s1 = [zeros(1,randi(sz)-1) sg zeros(1,randi(sz)-1)];

length = size(sg,2);
sg = sg + randn(1,length)*0.4;

s2 = [zeros(1,randi(sz)-1) sg zeros(1,randi(sz)-1)];

mx = max(numel(s1),numel(s2));

ax(1) = subplot(2,1,1);
stem(s1)
xlim([0 mx+1])

ax(2) = subplot(2,1,2);
stem(s2,'*')
xlim([0 mx+1])

if numel(s1) > numel(s2)
    slong = s1;
    sshort = s2;
else
    slong = s2;
    sshort = s1;
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

