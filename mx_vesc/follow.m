clear lMotor rMotor
global lMotor rMotor
global q dq qd dqd q1 q2
global task_filter task_controller task_sense task_match

lMotor = mx_vesc('/dev/VESC_001');  % define USB port here
lMotor.open;                       % open vesc
rMotor = mx_vesc('/dev/VESC_002');  
rMotor.open;    
              
q = 0;                              % motor position in rad
dq = 0;                             % motor rate in rad/s
qd = 0;                             % desired pos
dqd = 0;                            % desired rate

task_sense = mx_task(@()sense, 1/3500); % filter task
task_filter = mx_task(@()filter, 1/3500); % filter task
task_controller = mx_task(@()controller, 1/3500); % controller task
task_match=mx_task(@()match,1/3500);

timestart = mx_sleep(0);            % get number of seconds since epoch
timenow = timestart;                % needed to enter the while loop below

while (timenow <= timestart + 100) % runs tasks for 10 seconds                    
    timenow = mx_sleep(1/5000);
    
    % place task runs here according to task priorities
    task_sense.run(timenow);
    task_filter.run(timenow);
    task_match.run(timenow);
    task_controller.run(timenow);
    
end

lMotor.delete;
rMotor.delete;



function filter()
global task_filter lMotor q dq % define passed global variables here

oq = q;
lMotor.get_sensors(); % get sensor
q = (lMotor.sensors.pid_pos - 180) * pi / 180; % read pos and converts it to rad
qt = q - oq;                                    % change in q
qt = qt - 2*pi*(qt>pi) + 2*pi*(qt<=-pi);        % take short side
dq = qt / task_filter.lastPeriod;               % euler approximation on speed
end

function controller()
global lMotor q dq qd dqd

qt = q - qd;                                    % tracking error
qt = qt - 2*pi*(qt>pi) + 2*pi*(qt<=-pi);        % short side
dqt = dq - dqd;                                 % rate error
s = dqt + sign(qt) * min(50*pi, abs(200 * qt));      % sliding surface with saturated rate
cmd_current = -0.01 * s;                           % smooth sliding controller
cmd_current = sign(cmd_current) * min(abs(cmd_current), 10); % current limit
lMotor.send_current(cmd_current);

end

function sense()
global lMotor rMotor qd 
    rMotor.get_sensors;            % get new sensor data
    lMotor.get_sensors;            % get new sensor data
    fprintf("Motor positions are: %.2f and %.2f \n", lMotor.sensors.pid_pos,rMotor.sensors.pid_pos);
    
    fprintf("this is the qd %.2f \n", qd)
end

function match()
global qd rMotor posid dqd
rMotor.get_sensors;
posid=rMotor.sensors.pid_pos;
qd=deg2rad(posid);
dqd=0;
end

