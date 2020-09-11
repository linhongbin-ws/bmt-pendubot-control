clear all; clc;


controller = pendubot_controller();
controller.setTaskPlotter(false);
controller.setTaskPrinter(false);
controller.set_enable_lqr(true);
duration = 20;
controller.maxTor1 = 1.19;


controller.set_zeroTor();
controller = controller.start();
tic

i = 0;

while (controller.sampling_counter < duration / controller.dT_Sampling)
    controller = controller.run();
%     if i ~= size(controller.record_buffer{1},2)
%         i = size(controller.record_buffer{1},2);
%         q1 = controller.q1;
%         q2 = controller.q2;
%         dq1 = controller.dq1_fil;
%         dq2 = controller.dq2_fil;
%         u = LQR_pendubot([q1, q2-q1, dq1,  dq2-dq1].');
%         current = u(1)/(29.2e-3);
% 
%     end
    %fprintf('desTor1: %.2f\n', desTor1)
end
controller.set_zeroTor();
controller.stop();
controller.delete_controller();

toc
clearvars controller

