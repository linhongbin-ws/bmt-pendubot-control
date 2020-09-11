%%
% controller.isStopTimer = true;


%%
ratio = 0.15/0.15;
controller.LQR_a = 0.15;
controller.LQR_b = 0.15;
controller.LQR_p = controller.LQR_a/ ratio;
controller.LQR_d = controller.LQR_b/ ratio;
controller.dq1_filRatio = 0.6;
controller.dq2_filRatio = 0.6;
controller.LQR_wdw = 15;







%tune 1
%%
% controller.LQR_a = 0.15;
% controller.LQR_b = 0.15;
% controller.LQR_p = 0.08;
% controller.LQR_d = 0.08;
