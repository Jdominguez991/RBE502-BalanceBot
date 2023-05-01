% Load rosbag file. Make sure to select the appropriate file
clear;clc;
pd_100 = rosbag('bag_files/SS_FF_100N.bag'); % load bag file
pd_500 = rosbag('bag_files/SS_FF_500N.bag'); % load bag file
pd_1000 = rosbag('bag_files/SS_FF_1000N.bag'); % load bag file
pd_2000 = rosbag('bag_files/SS_FF_2000N.bag'); % load bag file

% plot full timeseries
data_100 = pd_100.timeseries; % output data as time series
rpy_100 = timeseries2timetable(data_100); % convert time series to time table so an index is available
figure("Name",'100')
plot(rpy_100.Time,rpy_100.("/teeterbot/rpy Properties")(:,2))   % plot pitch
xlabel('Time [sec]')
ylabel('Pitch [radians]')

data_500 = pd_500.timeseries; % output data as time series
rpy_500 = timeseries2timetable(data_500); % convert time series to time table so an index is available
figure("Name",'500')
plot(rpy_500.Time,rpy_500.("/teeterbot/rpy Properties")(:,2))   % plot pitch
xlabel('Time [sec]')
ylabel('Pitch [radians]')


data_1000 = pd_1000.timeseries; % output data as time series
rpy_1000 = timeseries2timetable(data_1000); % convert time series to time table so an index is available
figure("Name",'1000')
plot(rpy_1000.Time,rpy_1000.("/teeterbot/rpy Properties")(:,2))   % plot pitch
xlabel('Time [sec]')
ylabel('Pitch [radians]')


data_2000 = pd_2000.timeseries; % output data as time series
rpy_2000 = timeseries2timetable(data_2000); % convert time series to time table so an index is available
figure("Name",'2000')
plot(rpy_2000.Time,rpy_2000.("/teeterbot/rpy Properties")(:,2))   % plot pitch
xlabel('Time [sec]')
ylabel('Pitch [radians]')

% all on one plot
figure('Name','Overlay_rev2')
plot(rpy_100.("/teeterbot/rpy Properties")(625:729,2))
hold on
plot(rpy_500.("/teeterbot/rpy Properties")(286:372,2))
hold on
plot(rpy_1000.("/teeterbot/rpy Properties")(282:364,2))
hold on
plot(rpy_2000.("/teeterbot/rpy Properties")(261:338,2))
xlabel('Time [sec]')
ylabel('Pitch [radians]')
xlim([0 70])
%xlim([seconds(rpy_100.Time(625)) seconds(rpy_100.Time(729))])
xticklabels({'0.0','0.4','0.8','1.2','1.6','2.0','2.4','2.8'})
legend('100 N','500 N','1000 N','2000 N','Location','northwest')
title('Pitch Angle vs Time using a Gazebo Force')

