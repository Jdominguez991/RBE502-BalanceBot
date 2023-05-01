% Load rosbag file. Make sure to select the appropriate file
clear;clc;
bagselect = rosbag('bag_files/PID_controller_rev2.bag'); % load bag file

% plot full timeseries
data = bagselect.timeseries; % output data as time series
rpy = timeseries2timetable(data); % convert time series to time table so an index is available
figure()
plot(rpy.Time,rpy.("/teeterbot/rpy Properties")(:,2))   % plot pitch
xlabel('Time [sec]')
ylabel('Pitch [radians]')

% 
% all on one plot
plot(rpy.("/teeterbot/rpy Properties")(126:156,2))
hold on
plot(rpy.("/teeterbot/rpy Properties")(269:299,2))
hold on
plot(rpy.("/teeterbot/rpy Properties")(443:473,2))
hold on
plot(rpy.("/teeterbot/rpy Properties")(605:635,2))
xlabel('Time [sec]')
ylabel('Pitch [radians]')
xlim([0 30])
legend('100 N','500 N','1000 N','2000 N')
xticklabels({'0.0','0.5','1.0','1.5','2.0','2.5','3.0'})
title('Pitch Angle vs Time using a Gazebo Force')

% 100 N
t01 = seconds(rpy.Time(135));
trise1 = seconds(rpy.Time(138));
tsettle1 = seconds(rpy.Time(161));
risetime1 = trise1-t01
settlingtime1 = tsettle1 - trise1
overshoot1 = 0;

% 500 N
t02 = seconds(rpy.Time(278));
trise2 = seconds(rpy.Time(281));
tsettle2 = seconds(rpy.Time(305));
risetime2 = trise2 - t02
settlingtime2 = abs(tsettle2 - trise2)

% 1000 N
t03 = seconds(rpy.Time(452));
trise3 = seconds(rpy.Time(455));
tsettle3 = seconds(rpy.Time(478));
risetime3 = trise3 - t03
settlingtime3 = abs(tsettle3 - trise3)

% 2500 N
t04 = seconds(rpy.Time(614));
trise4 = seconds(rpy.Time(617));
tsettle4 = seconds(rpy.Time(639));
risetime4 = trise4 - t04
settlingtime4 = abs(tsettle4 - trise4)

forces = [100, 500, 1000, 2000];
risetimes = [risetime1,risetime2,risetime3,risetime4]
settlingtimes = [settlingtime1, settlingtime2,settlingtime3,settlingtime4]
