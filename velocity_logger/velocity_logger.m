clear;
clc;
close all;

%% ============================================================
% Paste Arduino Serial output directly below
%
% Columns:
%  1  time
%  2  dt
%  3  refPosL
%  4  actualPosL
%  5  refPosR
%  6  actualPosR
%  7  pwmL
%  8  pwmR
%  9  maxPwmL
% 10  maxPwmR
% 11  actualVelL
% 12  actualVelR
% 13  refVelL
% 14  refVelR
% 15  entityVel
% 16  pathRefVel
%% ============================================================

data = [
];

%% Check data format
if size(data,2) ~= 16
    error('Data must contain exactly 16 columns. Current columns = %d', size(data,2));
end

%% ============================================================
% Extract data
%% ============================================================

time = data(:,1);
dt = data(:,2);

refPosL = data(:,3);
actualPosL = data(:,4);

refPosR = data(:,5);
actualPosR = data(:,6);

pwmL = data(:,7);
pwmR = data(:,8);

maxPwmL = data(:,9);
maxPwmR = data(:,10);

actualVelL = data(:,11);
actualVelR = data(:,12);

refVelL = data(:,13);
refVelR = data(:,14);

entityVel = data(:,15);
pathRefVel = data(:,16);

time = time - time(1);


%% ============================================================
% Position tracking
%% ============================================================

figure;

plot(time, refPosL, '--', 'LineWidth', 1.5);
hold on;
plot(time, actualPosL, 'LineWidth', 1.5);
plot(time, refPosR, '--', 'LineWidth', 1.5);
plot(time, actualPosR, 'LineWidth', 1.5);

grid on;
xlabel('Time (s)');
ylabel('Position (counts)');
title('Position Tracking');

legend( ...
    'Reference L', ...
    'Actual L', ...
    'Reference R', ...
    'Actual R', ...
    'Location', 'best');


%% ============================================================
% Position error
%% ============================================================

errorL = refPosL - actualPosL;
errorR = refPosR - actualPosR;

figure;

plot(time, errorL, 'LineWidth', 1.5);
hold on;
plot(time, errorR, 'LineWidth', 1.5);
yline(0, '--');

grid on;
xlabel('Time (s)');
ylabel('Position Error (counts)');
title('Position Tracking Error');

legend('L Error', 'R Error', 'Location', 'best');


%% ============================================================
% PID PWM + saturation
%% ============================================================

figure;

plot(time, pwmL, 'LineWidth', 1.5);
hold on;
plot(time, pwmR, 'LineWidth', 1.5);

plot(time, maxPwmL, '--', 'LineWidth', 1.2);
plot(time, -maxPwmL, '--', 'LineWidth', 1.2);

plot(time, maxPwmR, ':', 'LineWidth', 1.2);
plot(time, -maxPwmR, ':', 'LineWidth', 1.2);

grid on;
xlabel('Time (s)');
ylabel('PWM');
title('PID Output and PWM Limits');

legend( ...
    'PWM L', ...
    'PWM R', ...
    '+Max L', ...
    '-Max L', ...
    '+Max R', ...
    '-Max R', ...
    'Location', 'best');


%% ============================================================
% Motor velocity tracking
%% ============================================================

figure;

plot(time, actualVelL, 'LineWidth', 1.5);
hold on;
plot(time, refVelL, '--', 'LineWidth', 1.5);

plot(time, actualVelR, 'LineWidth', 1.5);
plot(time, refVelR, '--', 'LineWidth', 1.5);

grid on;
xlabel('Time (s)');
ylabel('Velocity (mm/s)');
title('Motor Velocity Tracking');

legend( ...
    'Actual L', ...
    'Reference L', ...
    'Actual R', ...
    'Reference R', ...
    'Location', 'best');


%% ============================================================
% Entity velocity
%% ============================================================

figure;

plot(time, entityVel, 'LineWidth', 1.5);
hold on;
plot(time, pathRefVel, '--', 'LineWidth', 1.5);

grid on;
xlabel('Time (s)');
ylabel('Velocity (mm/s)');
title('Entity Velocity');

legend( ...
    'Measured Entity Velocity', ...
    'Reference Path Velocity', ...
    'Location', 'best');


%% ============================================================
% Control timing
%% ============================================================

figure;

plot(time, dt * 1000, 'LineWidth', 1.2);

grid on;
xlabel('Time (s)');
ylabel('dt (ms)');
title('Control Loop Timing');


%% ============================================================
% Diagnostic summary
%% ============================================================

fprintf('\n===== G1 Diagnostic Summary =====\n');

fprintf('Mean dt: %.3f ms\n', mean(dt) * 1000);
fprintf('Max dt : %.3f ms\n', max(dt) * 1000);
fprintf('Min dt : %.3f ms\n', min(dt) * 1000);

fprintf('\n');

fprintf('Mean |L error|: %.2f counts\n', mean(abs(errorL)));
fprintf('Mean |R error|: %.2f counts\n', mean(abs(errorR)));

fprintf('Max |L error| : %.1f counts\n', max(abs(errorL)));
fprintf('Max |R error| : %.1f counts\n', max(abs(errorR)));

fprintf('\n');

fprintf('Max |PWM L|: %.1f / limit %.1f\n', ...
    max(abs(pwmL)), max(maxPwmL));

fprintf('Max |PWM R|: %.1f / limit %.1f\n', ...
    max(abs(pwmR)), max(maxPwmR));

satL = mean(abs(pwmL) >= (maxPwmL - 1)) * 100;
satR = mean(abs(pwmR) >= (maxPwmR - 1)) * 100;

fprintf('L saturation samples: %.1f %%\n', satL);
fprintf('R saturation samples: %.1f %%\n', satR);

fprintf('=================================\n');
