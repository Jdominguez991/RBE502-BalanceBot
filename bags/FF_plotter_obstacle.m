% Load rosbag file. Make sure to select the appropriate file
clear;clc;
bagselect = rosbag('bag_files/FF_controller_obstacle.bag'); % load bag file

% plot full timeseries
data = bagselect.timeseries; % output data as time series
rpy = timeseries2timetable(data); % convert time series to time table so an index is available
figure()%rpy.Time(57:602),
plot(rpy.("/teeterbot/rpy Properties")(57:602,2))   % plot pitch
xlabel('Time [sec]')
%xticks([0 10 20 30 40 50 60])
xticklabels({'0','10','20','30','40','50','60'})
ylabel('Pitch [radians]')
title('Pitch Angle vs. Time with Keyboard Command')