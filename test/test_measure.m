clear all; clc;
% addpath('..')

for i = 1:1
    controller = pendubot_controller();
    controller = controller.setTaskPlotter(false);
    controller = controller.setTaskPrinter(true);
    duration = 2;
    controller = controller.start;
    tic
    while (controller.timeNow-controller.timeStart<=duration)
        controller = controller.run(); 
    end
    controller.stop;
    toc
    %clear controller duration
end
save_file_str = fullfile('data', 'test.png');
controller.plot_sampling_data(save_file_str)