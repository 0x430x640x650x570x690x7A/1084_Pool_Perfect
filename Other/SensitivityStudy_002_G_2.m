clc;
clear;
close all;
%Mission 3 sensitivity study
%M3 = 2 + [N_(#laps X sensor length X sensor weight) / Max_(#laps X sensor length X sensor weight)] 
% where Max_(#laps X sensor length) is the highest #laps X sensor length score of all teams
% ONLY values that need to be changed are MAX, MYVALUES and CHANGE incr

%set max values
maxLaps = 10; % in laps
maxSensorL = 10; % in cm
maxSensorW = .25; % in kg

%set expected values
%myPercentOfMax = .7;
myLaps = 3; %floor(maxLaps * myPercentOfMax);
mySensorL =  7;%maxSensorL * myPercentOfMax;
mySensorW =  .23;%maxSensorW * myPercentOfMax;

dt = 1; % increment change by this %
change = -50:dt:50; % change in percent from -50% to 50%

% SCORE VS LAPS GRAPH
figure(1);
%find min and max expected values
minExpectedLaps = floor(myLaps * .5);
maxExpectedLaps = floor(myLaps * 1.5);
if(maxExpectedLaps >= maxLaps)
    maxExpectedLaps = myLaps;
    ylim([-inf 3])
end
for i = minExpectedLaps:maxExpectedLaps
    M3_laps = 2 + ((floor(i*(1+change*.01)) * mySensorL * mySensorW) / (maxLaps * maxSensorL * maxSensorW));
    label = sprintf('# Laps = %g', i);
    plot(change, M3_laps, 'DisplayName', label);
    hold on;
end

legend('show', 'Location', 'northwest')
xlabel('% Change');
ylabel('Score');
title('Score vs Laps');
grid on;

% SCORE VS SENSOR LENGTH GRAPH
figure(2);
minExpectedLength = mySensorL * .5;
maxExpectedLength = mySensorL * 1.5;
if(maxExpectedLength >= maxSensorL)
    maxExpectedLength = mySensorL;
    ylim([-inf 3])
end
stepSizeL = (maxExpectedLength-minExpectedLength) / 5;
for i = minExpectedLength:stepSizeL:maxExpectedLength
    M3_length = 2 + ((myLaps * (i*(1+change*.01)) * mySensorW) / (maxLaps * maxSensorL * maxSensorW));
    label = sprintf('Length = %g', i);
    plot(change, M3_length, 'DisplayName', label);
    hold on;
end
legend('show', 'Location', 'northwest')
xlabel('% Change');
ylabel('Score');
title('Score vs Sensor Length (cm)');
grid on;

% SCORE VS SENSOR WEIGHT GRAPH
figure(3);
minExpectedWeight = mySensorW * .5;
maxExpectedWeight = mySensorW * 1.5;
if(maxExpectedWeight >= maxSensorW)
    maxExpectedWeight = mySensorW;
    ylim([-inf 3])
end
stepSizeW = (maxExpectedWeight-minExpectedWeight) / 5;
for i = minExpectedWeight:stepSizeW:maxExpectedWeight
    M3_weight = 2 + ((myLaps * mySensorL * (i*(1+change*.01))) / (maxLaps * maxSensorL * maxSensorW));
    label = sprintf('Weight = %g', i);
    plot(change, M3_weight, 'DisplayName', label);
    hold on;
end
legend('show', 'Location', 'northwest')
xlabel('% Change');
ylabel('Score');
title('Score vs Sensor Weight (kg)');
grid on;
