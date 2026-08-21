clear;
clc;
close all;

portName = "COM5";
baudRate = 115200;

arduino = serialport(portName, baudRate);
configureTerminator(arduino, "LF");
flush(arduino);

logData = [];

figure;

motor1Line = animatedline("LineWidth", 1.5);
motor2Line = animatedline("LineWidth", 1.5);
entityLine = animatedline("LineWidth", 1.5);

grid on;
xlabel("Time (s)");
ylabel("Velocity (mm/s)");
title("CoreXY velocity profile");

legend( ...
    "Motor 1", ...
    "Motor 2", ...
    "Entity speed", ...
    "Location", "best" ...
);

while ishandle(gcf)
    serialLine = strtrim(readline(arduino));

    if serialLine == "END"
        break;
    end

    values = str2double(split(serialLine, ","));

    if numel(values) ~= 6 || any(isnan(values))
        continue;
    end

    time = values(1);
    motor1 = values(2);
    motor2 = values(3);
    velocityX = values(4);
    velocityY = values(5);
    entity = values(6);

    addpoints(motor1Line, time, motor1);
    addpoints(motor2Line, time, motor2);
    addpoints(entityLine, time, entity);

    drawnow limitrate;

    logData(end + 1, :) = [
        time,
        motor1,
        motor2,
        velocityX,
        velocityY,
        entity
    ];
end

variableNames = { ...
    'time_s', ...
    'motor1_mm_s', ...
    'motor2_mm_s', ...
    'x_mm_s', ...
    'y_mm_s', ...
    'entity_mm_s' ...
};

velocityTable = array2table( ...
    logData, ...
    'VariableNames', variableNames ...
);

writetable(velocityTable, 'velocity_log.csv');

disp('Saved velocity_log.csv');