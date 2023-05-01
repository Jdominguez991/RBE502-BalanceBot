% Load rosbag file. Make sure to select the appropriate file
clear;clc;
bagselect = rosbag('bag_files/FF_controller_rev2.bag'); % load bag file

% plot full timeseries
data = bagselect.timeseries; % output data as time series
rpy = timeseries2timetable(data); % convert time series to time table so an index is available
figure()
plot(rpy.Time,rpy.("/teeterbot/rpy Properties")(:,2))   % plot pitch
xlabel('Time [sec]')
ylabel('Pitch [radians]')

% all on one plot
figure()
plot(rpy.("/teeterbot/rpy Properties")(138:168,2))
hold on
plot(rpy.("/teeterbot/rpy Properties")(391:421,2))
hold on
plot(rpy.("/teeterbot/rpy Properties")(541:571,2))
hold on
plot(rpy.("/teeterbot/rpy Properties")(693:723,2))
xlabel('Time [sec]')
ylabel('Pitch [radians]')
xlim([0 30])
xticklabels({'0.0','0.5','1.0','1.5','2.0','2.5','3.0'})
legend('100 N','500 N','1000 N','2000 N')
title('Pitch Angle vs Time using a Gazebo Force')


% 100 N
t01 = seconds(rpy.Time(147));
trise1 = seconds(rpy.Time(149));
tsettle1 = seconds(rpy.Time(167));
risetime1 = trise1-t01
settlingtime1 = tsettle1 - trise1
overshoot1 = 0;

% 500 N
t02 = seconds(rpy.Time(400));
trise2 = seconds(rpy.Time(403));
tsettle2 = seconds(rpy.Time(451));
risetime2 = trise2 - t02
settlingtime2 = abs(tsettle2 - trise2)
%max overshoot occurs at idx 427 (0-.000424)

% 1000 N
t03 = seconds(rpy.Time(550));
trise3 = seconds(rpy.Time(553));
tsettle3 = seconds(rpy.Time(609));
risetime3 = trise3 - t03
settlingtime3 = abs(tsettle3 - trise3)
% max overshoot occurs at idx 576 (-0.000845)

% 2000 N
t04 = seconds(rpy.Time(702));
trise4 = seconds(rpy.Time(705));
tsettle4 = seconds(rpy.Time(774));
risetime4 = trise4 - t04
settlingtime4 = abs(tsettle4 - trise4)
% max overshoot occurs at idx 727 (-0.0021)


forces = [100, 500, 1000, 2000];
risetimes = [risetime1,risetime2,risetime3,risetime4]
settlingtimes = [settlingtime1, settlingtime2,settlingtime3,settlingtime4]
overshoot = [0 -0.000424 -0.000845 -0.0021]
