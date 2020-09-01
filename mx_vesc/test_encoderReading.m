clear all
clear myMotor task_filter task_controller

%% global variables are needed for inter-tasks read/write

global myMotor q dq task_filter task_printer task_plotter f q_list plotWindow gearRatio

myMotor = mx_vesc('/dev/VESC_001');  % define USB port here
%myMotor{2} = mx_vesc('/dev/ttyACM1');  % if there's other VESCs
q = 0;                              % motor position in rad
dq = 0;                             % motor rate in rad/s

% mx_task class definition
% second argument is the desired task period
task_filter = mx_task(@()filter, 1/1000); % filter task
task_printer = mx_task(@()printer, 1/10); % printer task
task_plotter = mx_task(@()plotter, 1/10); % printer task

%% Filter and printing
myMotor.open;                       % open vesc

timestart = mx_sleep(0);            % get number of seconds since epoch
timenow = timestart;                % needed to enter the while loop below


plotWindow = 100;
gearRatio = 5;
duration = 10;
q_list = []
figure(1)

while (timenow <= timestart + duration) % runs tasks for 10 seconds                    
    timenow = mx_sleep(0.00001); % sleeps thread for 10us
    % place task runs here according to task priorities
    task_filter.run(timenow);
    task_printer.run(timenow);
    task_plotter.run(timenow);
end

myMotor.delete;      

%% tasks definition
function filter()
global task_printer myMotor q dq  gearRatio% define passed global variables here

oq = q;
myMotor.get_sensors(); % get sensor
q = (myMotor.sensors.pid_pos - 180) * pi / 180 ; % read pos and converts it to rad
qt = q - oq;                                    % change in q
qt = qt - 2*pi*(qt>pi) + 2*pi*(qt<=-pi);        % take short side
dq = 0.01 * qt / task_printer.lastPeriod +...
     0.99 * dq;               % euler approximation on speed + low pass
end

function printer()
global q dq

fprintf("pos: %.4f rad \t spd: %.4f rad/s\n", q, dq);   
% PRINTING FUNCTIONS ARE VERY SLOW
% AVOID PRINTING WHEN DOING A HIGH FREQUENCY PID LOOP
% TRY TO PRINT WITH AN ACTUAL PARALLEL PROCESS
end

function plotter()
global q_list q plotWindow 
q_list =  [q_list, q];
if size(q_list,2) > plotWindow
    q_list = q_list(end-plotWindow:end);
end
figure(1)
plot(q_list)
drawnow
end

