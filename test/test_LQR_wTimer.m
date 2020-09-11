clear all; clc;


controller = pendubot_controller();
controller.setTaskPlotter(false);
controller.setTaskPrinter(false);
controller.set_enable_lqr(true);
duration = 20;
controller.maxTor1 = 1.19;


controller.set_zeroTor();
controller.start_timer();
% controller = controller.start();
% controller_timer = timer('ExecutionMode','fixedRate', 'Period', 0.001, 'TimerFcn', @myfun, 'UserData', controller);
% controller.isStopTimer = false;
% start(controller_timer);
% 
% function myfun(obj, evt)
%     controller = get(obj, 'UserData');
%     controller.run();
%     
%     if controller.isStopTimer
%         stop(obj);
%         controller.set_zeroTor();
%         controller.stop();
%         controller.delete_controller();
%         clearvars obj controller;
%     end
% end
% 
