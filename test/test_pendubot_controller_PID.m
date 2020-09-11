%%
clear all; clc;
controller = pendubot_controller();
controller = controller.setTaskPlotter(false);
controller = controller.setTaskPrinter(false);
controller = controller.setTaskPID(true);
duration = 10;
controller.maxTor1 = 2;
controller.maxTor2 = 2;

p2d = 20;
controller.PID_p1 = 10;
controller.PID_d1 = controller.PID_p1 / 20;
controller.PID_p2 = 5;
controller.PID_d2 = controller.PID_p2 / 40;

controller.set_zeroTor();
controller = controller.start();
controller.isEnableSafeTrip = true;
controller.move_joint(pi, pi);
tic
while (controller.timeNow-controller.timeStart<=duration)
    controller = controller.run(); 
end
controller.set_zeroTor();
controller.stop();
controller.delete_controller();
controller.plot_sampling_data()

toc


