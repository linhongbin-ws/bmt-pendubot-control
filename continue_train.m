clear all; clc;
addpath(genpath('./pilco-matlab'))
load(['./data/pendubot_16_H60.mat'])
dynmodel.train = @train
% applyController; % apply controller to system
% disp(['controlled trial # ' num2str(j)]);
% if plotting.verbosity > 0;      % visualization of trajectory
%     if ~ishandle(1); figure(1); else set(0,'CurrentFigure',1); end; clf(1);
%         draw_rollout_pendubot;
% end
trail = j;
fprintf("current trail is %d\n", trail)
for j = trail+1:100
  tic
  trainDynModel;   % train (GP) dynamics model
  learnPolicy;     % learn policy
  toc
  applyController; % apply controller to system
  disp(['controlled trial # ' num2str(j)]);
  if plotting.verbosity > 0;      % visualization of trajectory
    if ~ishandle(1); figure(1); else set(0,'CurrentFigure',1); end; clf(1);
    draw_rollout_pendubot;
  end
end