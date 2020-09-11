% include some paths
try
  rd = './pilco-matlab/';
  addpath([rd 'base'],[rd 'util'],[rd 'gp'],[rd 'control'],[rd 'loss']);
catch
end
% load('./archive/data-2020-08-31/pendubot_60_H60.mat')
load('./data/pendubot_19_H150.mat')
% setup for vesc controller
controller = pendubot_controller();
controller = controller.setTaskPlotter(false);
controller = controller.setTaskPrinter(false);
controller.maxTor1 = policy.maxU;
controller.maxTor2 = 1;
controller.dT_Sampling = dt;

start=gaussian(mu0, S0);
H =500;
%
if isfield(plant,'augment'), augi = plant.augi;             % sort out indices!
else plant.augment = inline('[]'); augi = []; end
if isfield(plant,'subplant'), subi = plant.subi;
else plant.subplant = inline('[]',1); subi = []; end
odei = plant.odei; poli = plant.poli; dyno = plant.dyno; angi = plant.angi;
simi = sort([odei subi]);
nX = length(simi)+length(augi); nU = length(policy.maxU); nA = length(angi);

state(simi) = start; state(augi) = plant.augment(state);      % initializations
x = zeros(H+1, nX+2*nA);
x(1,simi) = start' + randn(size(simi))*chol(plant.noise);
x(1,augi) = plant.augment(x(1,:));
u = zeros(H, nU); latent = zeros(H+1, size(state,2)+nU);
y = zeros(H, nX); L = zeros(1, H); next = zeros(1,length(simi));


% start controller
controller.set_zeroTor();
controller.start;

state = [0,0,0,0];

i = 0;
while(controller.sampling_counter<= H) %-------------------------------------------- generate trajectory
  controller = controller.run();
  if i ~= controller.sampling_counter
      i = size(controller.sampling_counter,2); % udpate i 
      
      % get the update signals
      q1 = controller.record_buffer{1}(end);
      q2 = controller.record_buffer{2}(end);
      dq1_fil = controller.record_buffer{3}(end);
      dq2_fil = controller.record_buffer{4}(end);

      next = [dq1_fil, dq2_fil, q1, q2];

      % 3. Stop rollout if constraints violated ------------------------------------
      if isfield(plant,'constraint') && plant.constraint(next(odei))
        H = i-1;
        fprintf('state constraints violated...\n');
        break;
      end

      % 4. Augment state and randomize ---------------------------------------------
      state(simi) = next(simi); state(augi) = plant.augment(state);
    %       x(i+1,simi) = state(simi) + randn(size(simi))*chol(plant.noise);
      x(i,simi) = state(simi);
      x(i,augi) = plant.augment(x(i,:));

      % 5. Compute Cost ------------------------------------------------------------


      s = x(i,dyno)'; sa = gTrig(s, zeros(length(s)), angi); s = [s; sa];
      x(i,end-2*nA+1:end) = s(end-2*nA+1:end);

      % 1. Apply policy ... or random actions --------------------------------------
      if isfield(policy, 'fcn')
        u(i,:) = policy.fcn(policy,s(poli),zeros(length(poli)));
      else
        u(i,:) = policy.maxU.*(2*rand(1,nU)-1);
      end
      latent(i,:) = [state u(i,:)];                                  % latent state

    %       % 2. Simulate dynamics -------------------------------------------------------
    %       next(odei) = simulate(state(odei), u(i,:), plant);
    %       next(subi) = plant.subplant(state, u(i,:));
      controller.desTor1 = u(i,1);
  end
end
controller.desTor1 = 0;
controller.set_zeroTor();
controller.stop();
controller.delete_controller();

