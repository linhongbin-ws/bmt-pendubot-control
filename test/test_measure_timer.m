clear all; clc;
% addpath('..')


controller = pendubot_controller();
controller = controller.setTaskPlotter(false);
controller = controller.setTaskPrinter(true);
controller = controller.start;

controller_timer = timer('ExecutionMode','fixedRate', 'Period', 0.001, 'TimerFcn', @myfun, 'UserData', controller)
controller.isStopTimer = false;
start(controller_timer);

function myfun(obj, evt)
    controller = get(obj, 'UserData');
    controller.run();
    
    if controller.isStopTimer
        stop(obj);
    end
end


% controller_timer
% clearvars controller_timer
