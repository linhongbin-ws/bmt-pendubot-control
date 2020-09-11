clear all; clc;


controller = pendubot_controller();
controller = controller.setTaskPlotter(false);
controller = controller.setTaskPrinter(false);
duration = 3;
controller.maxTor1 = 2;


controller.set_zeroTor();
controller = controller.start();
tic

i = 0

while (controller.sampling_counter < duration / controller.dT_Sampling)
    controller = controller.run();
    if i ~= size(controller.record_buffer{1},2)
        i = size(controller.record_buffer{1},2);
        controller.desTor1 =  - controller.maxTor1;
    end
    %fprintf('desTor1: %.2f\n', desTor1)
end
controller.set_zeroTor();
controller.stop();
controller.delete_controller();

toc
% clearvars controller
controller.plot_sampling_data('test_constant.png')

