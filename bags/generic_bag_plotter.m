% Load rosbag file. Make sure to select the appropriate file
clear;clc;
bagselect = rosbag('bag_files/_2023-04-27-02-53-45.bag'); % load bag file

% plot full timeseries
data = bagselect.timeseries; % output data as time series
rpy = timeseries2timetable(data); % convert time series to time table so an index is available
figure()
plot(rpy.Time,rpy.("/teeterbot/rpy Properties")(:,2))   % plot pitch
xlabel('Time [sec]')
ylabel('Pitch [radians]')



