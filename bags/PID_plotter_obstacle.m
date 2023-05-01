% Load rosbag file. Make sure to select the appropriate file
clear;clc;
bagselect = rosbag('bag_files/PID_controller_obstacle_rev3.bag'); % load bag file

% plot full timeseries
data = bagselect.timeseries; % output data as time series
rpy = timeseries2timetable(data); % convert time series to time table so an index is available
figure()
plot(rpy.("/teeterbot/rpy Properties")(44:573,2))   % plot pitch
xlabel('Time [sec]')
ylabel('Pitch [radians]')
xticklabels({'0','10','20','30','40','50','60'})
title('Pitch Angle vs. Time with Keyboard Command')

