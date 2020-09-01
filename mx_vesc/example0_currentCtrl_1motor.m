clear myMotor

%% initialization
myMotor = mx_vesc('/dev/VESC_002');  % define USB port here
myMotor.open;                       % open vesc
curr_cmd = 0.4;

% %% reading angle and sending current
% tic;
% while (toc < 10)                    
%     myMotor.send_current(curr_cmd); % sends current command
%     curr_cmd = curr_cmd * -1;
%     
%     myMotor.get_sensors;            % get new sensor data
%     fprintf("Motor current: %.2f A\n", myMotor.sensors.current_motor); 
%     % read sensor data and print
%     
%     pause(0.5);
% end
% 
% myMotor.delete;                     


%% reading angle and sending current
tic;                    
myMotor.send_current(curr_cmd); % sends current command
toc
curr_cmd = curr_cmd * -1;

myMotor.get_sensors;            % get new sensor data
fprintf("Motor current: %.2f A\n", myMotor.sensors.current_motor); 
% read sensor data and print

pause(0.5);
                     
