% Load rosbag file. Make sure to select the appropriate file
clear;clc;
PID_controller_obstacle = rosbag('bag_files/PID_controller_obstacle_rev3.bag'); % load bag file
FF_controller_obstacle = rosbag('bag_files/FF_controller_obstacle.bag'); % load bag file


% plot full timeseries
PIDrpy = timeseries2timetable(PID_controller_obstacle.timeseries); % convert time series to time table so an index is available
FFrpy = timeseries2timetable(FF_controller_obstacle.timeseries); % convert time series to time table so an index is available
figure()
 plot(PIDrpy.("/teeterbot/rpy Properties")(44:563,2))   % plot pitch
 hold on
 plot(FFrpy.("/teeterbot/rpy Properties")(57:602,2)) 
xlabel('Time [sec]')
ylabel('Pitch [radians]')
legend('PID','Feedforward')

% initial movement
figure()
plot(PIDrpy.("/teeterbot/rpy Properties")(44:96,2))
hold on
plot(FFrpy.("/teeterbot/rpy Properties")(57:109,2))
legend('PID','Feedforward')
xlabel('Time [sec]')
ylabel('Pitch [radians]')

% stopping
figure()
plot(PIDrpy.("/teeterbot/rpy Properties")(129:169,2))
hold on
plot(FFrpy.("/teeterbot/rpy Properties")(142:182,2))
legend('PID','Feedforward')
xlabel('Time [sec]')
ylabel('Pitch [radians]')


% combo
figure()
plot(PIDrpy.("/teeterbot/rpy Properties")(44:169,2))
hold on
plot(FFrpy.("/teeterbot/rpy Properties")(57:182,2))
legend('PID','PD with Feedforward','Location','northwest')
xlabel('Time [sec]')
ylabel('Pitch [radians]')
xticklabels({'0','2','4','6','8','10','12'})
title('Pitch Angle vs. Time with Keyboard Command')

% PID - moving
t01 = seconds(PIDrpy.Time(53));
trise1 = seconds(PIDrpy.Time(57));
tsettle1 = seconds(PIDrpy.Time(96));
risetime1 = trise1-t01
settlingtime1 = tsettle1 - trise1

% FF - moving
t02 = seconds(FFrpy.Time(66));
trise2 = seconds(FFrpy.Time(70));
tsettle2 = seconds(FFrpy.Time(112));
risetime2 = trise2-t02
settlingtime2 = tsettle2 - trise2

