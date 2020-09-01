 %%
addpath('./mx_vesc')
clear all; clc;

% test how stable is the controller
for i = 1:1
    controller = pendubot_controller();
    controller = controller.setTaskPlotter(false);
    controller = controller.setTaskPrinter(true);
    %controller = controller.setTaskPlotter(true);
    duration = 5;
    controller = controller.start;
    tic
    while (controller.timeNow-controller.timeStart<=duration)
        controller = controller.run();
    end
    controller.stop;
    toc
    clear controller duration
end
% controller.delete_controller;
global RecordCell
size(RecordCell)
