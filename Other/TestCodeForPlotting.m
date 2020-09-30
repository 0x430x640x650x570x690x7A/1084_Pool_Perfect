clc;
clear;
close all;
%Mission 3 sensitivity study
%M3 = 2 + [N_(#laps X sensor length X sensor weight) / Max_(#laps X sensor length X sensor weight)] 
% where Max_(#laps X sensor length) is the highest #laps X sensor length score of all teams

maxLaps = 5; % in laps
maxSensorL = 10; % in cm
maxSensorW = .250; % in kg

myPercentOfMax = .7;
myLaps = floor(maxLaps * myPercentOfMax);
mySensorL = maxSensorL * myPercentOfMax;
mySensorW = maxSensorW * myPercentOfMax;

dt = .01; % increment change
change = .50:dt:1.50; % change in percent from -50% to 50%

M3_laps = 2 + ((floor(myLaps*change) * mySensorL * mySensorW) / (maxLaps * maxSensorL * maxSensorW));
M3_length = 2 + ((myLaps * (mySensorL*change) * mySensorW) / (maxLaps * maxSensorL * maxSensorW));
M3_weight = 2 + ((myLaps * mySensorL * (mySensorW*change)) / (maxLaps * maxSensorL * maxSensorW));

figure(1);
plot(change, M3_laps);
xlabel('% Change');
ylabel('Score');
title('Score vs Laps');
grid on;
hold on;

figure(2);
plot(change, M3_length);
xlabel('% Change');
ylabel('Score');
title('Score vs Length (cm)');
grid on;
hold on;

figure(3);
plot(change, M3_weight);
xlabel('% Change');
ylabel('Score');
title('Score vs Weight (kg)');
grid on;
hold on;

figure(4);
plot3(floor(myLaps*change), (mySensorL*change), (mySensorW*change));
xlabel('Laps');
ylabel('Length');
zlabel('Weight');
title('Laps vs Length vs Weight');
grid on;
hold on;

%maxLaps = 5; % in laps
%maxSensorL = 10; % in cm
%maxSensorW = .250; % in kg

laps = floor(maxLaps.*change); % in laps
sensorL = maxSensorL.*change; % in cm
sensorW = maxSensorW.*change; % in kg
M3 = 2 + ((floor(myLaps.*change) .* (mySensorL.*change) .* (mySensorW.*change)) ./ (maxLaps .* maxSensorL .* maxSensorW));


figure(5);
scatter3(laps,sensorL,sensorW,5,M3,'filled')    % draw the scatter plot
ax = gca;
ax.XDir = 'reverse';
view(-31,14)
xlabel('Laps')
ylabel('Sensor Length (cm)')
zlabel('Sensor Weight (kg)')

cb = colorbar;                                     % create and label the colorbar
cb.Label.String = 'M3 Score';



