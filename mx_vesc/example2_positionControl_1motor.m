clear myMotor task_filter task_controller
try
    close(fig(1));
catch
end

%% global variables are needed for inter-tasks read/write
global myMotor dt_rec dt_idx
global q dq qd dqd
global task_filter task_controller task_cmd

myMotor = mx_vesc('/dev/VESC_001');  % define USB port here
%myMotor{2} = mx_vesc('/dev/ttyACM1');  % if there's other VESCs
dt_rec = zeros(100000, 1);          % for task period histogram
dt_idx = 0;                             
q = 0;                              % motor position in rad
dq = 0;                             % motor rate in rad/s
qd = 0;                             % desired pos
dqd = 0;                            % desired rate
%myMotor.curr_dir = -1;              % flips current direction if necessary

% mx_task class definition
% second argument is the desired task period
task_filter = mx_task(@()filter, 1/2000); % filter task
task_controller = mx_task(@()controller, 1/2000); % controller task
task_cmd = mx_task(@()cmd_gen, 1/2000); % command generator task

%% Main tasks
myMotor.open;                       % open vesc

timestart = mx_sleep(0);            % get number of seconds since epoch
timenow = timestart;                % needed to enter the while loop below

while (timenow <= timestart + 10) % runs tasks for 10 seconds                    
    timenow = mx_sleep(0.00001); % sleeps thread for 10us
    
    % place task runs here according to task priorities
    task_filter.run(timenow);
    task_cmd.run(timenow);
    task_controller.run(timenow);
end

myMotor.delete;      

fig(1) = figure(1);
fig(1) = histogram(dt_rec(1: dt_idx));
fig(1).BinWidth = task_controller.period/50;
xlim([0 task_controller.period*5])
hold on
plot(max(dt_rec), 0, 'rx');
hold off

%% Task definitions
function filter()
global task_filter myMotor q dq % define passed global variables here

oq = q;
myMotor.get_sensors(); % get sensor
q = (myMotor.sensors.pid_pos - 180) * pi / 180; % read pos and converts it to rad
qt = q - oq;                                    % change in q
qt = qt - 2*pi*(qt>pi) + 2*pi*(qt<=-pi);        % take short side
dq = qt / task_filter.lastPeriod;               % euler approximation on speed
end

function controller()
global task_controller myMotor q dq qd dqd dt_rec dt_idx

qt = q - qd;                                    % tracking error
qt = qt - 2*pi*(qt>pi) + 2*pi*(qt<=-pi);        % short side
dqt = dq - dqd;                                 % rate error
s = dqt + sign(qt) * min(50*pi, abs(200 * qt));      % sliding surface with saturated rate
cmd_current = -0.01 * s;                           % smooth sliding controller
cmd_current = sign(cmd_current) * min(abs(cmd_current), 15); % current limit
myMotor.send_current(cmd_current);

% recording periods
dt_idx = dt_idx + 1;
dt_rec(dt_idx) = task_controller.lastPeriod;
end

function cmd_gen()
global task_cmd qd dqd

qd = pi/4 * (sin(task_cmd.timeInitiated * 2*pi*1)); % step generator
dqd = 0;
end

