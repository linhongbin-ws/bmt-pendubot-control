clear all; clc;
% addpath('..')

for i = 1:1
    controller = pendubot_controller();
    controller = controller.setTaskPlotter(false);
    controller = controller.setTaskPrinter(false);
    %controller = controller.setTaskPlotter(true);
    duration = 30;
    controller = controller.start;
    tic
    cnt = 0;
    settings_pendubot
%     angi = [3 4];   
%     plant.angi = angi;
%     cost.fcn = @loss_pendubot;                  % cost function
%     cost.gamma = 1;                             % discount factor
%     cost.p = [0.14, 0.14];                         % lengths of pendulums
%     cost.width = 0.14;                          % cost function width
%     cost.expl = 0;                              % exploration parameter (UCB)
%     cost.angle = plant.angi;                    % index of angle (for cost function)
%     cost.target = [0 0 0 0]';                   % target state
    while (controller.timeNow-controller.timeStart<=duration)
        controller = controller.run();
        if cnt~= controller.sampling_counter
            cnt = controller.sampling_counter;
            q1 = controller.record_buffer{1}(end);
            q2 = controller.record_buffer{2}(end);
            dq1_fil = controller.record_buffer{3}(end);
            dq2_fil = controller.record_buffer{4}(end);
            m = [dq1_fil, dq2_fil, q1, q2];
            s = zeros(4,4);
            [L, ~, ~, ~] = loss_pendubot(cost, m, s);
            fprintf("q1: %.2f, q2: %.2f, dq1: %.2f, dq2: %.2f, cost: %.2f\n", q1, q2, dq1_fil, dq2_fil, L);
        end
    end
    controller.stop;
    toc
    clear controller duration
end
