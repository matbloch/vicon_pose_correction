close all;
clear;
clc;

% load pose logs
[filename,filepath] = uigetfile('*.csv','Select the pose estimations');
poseEstimationFile = [filepath,filename];


%% =============================================
%%      load csv file

tracker_data = csvread(poseEstimationFile);


plot3(tracker_data(:,1), tracker_data(:,2), tracker_data(:,3));
hold on;

plot3(-tracker_data(:,8), -tracker_data(:,9), tracker_data(:,10));