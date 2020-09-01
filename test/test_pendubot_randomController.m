clear all; clc;


controller = pendubot_controller();
controller = controller.setTaskPlotter(false);
controller = controller.setTaskPrinter(true);
duration = 10;
controller.maxTor1 = 1.8;
controller.maxTor2 = 1;


controller.set_zeroTor();
controller = controller.start();
tic

i = 0

while (size(controller.record_buffer{1},2) < duration / controller.dT_Record)
    controller = controller.run();
    if i ~= size(controller.record_buffer{1},2)
        i = size(controller.record_buffer{1},2);
        controller.desTor1 =  (rand-0.5) * 2 * controller.maxTor1;
    end
    %fprintf('desTor1: %.2f\n', desTor1)
end
controller.set_zeroTor();
controller.stop();
controller.delete_controller();

toc
clearvars controller

