clear all; clc;
% addpath('..')

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
