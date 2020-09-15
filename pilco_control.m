function u = pilco_control(q1, q2, dq1, dq2, policy)
  next = [dq1, dq2, q1, q2];

  % 4. Augment state and randomize ---------------------------------------------
%       x(i+1,simi) = state(simi) + randn(size(simi))*chol(plant.noise);
  x = zeros(1, 8);
  x(1:4) = next;
  poli = [1,2,5,6,7,8];

  s = x(1:4)'; sa = gTrig(s, zeros(length(s)), 3:4); s = [s; sa];


  % 1. Apply policy ... or random actions --------------------------------------
  if isfield(policy, 'fcn')
    u = policy.fcn(policy,s(poli),zeros(length(poli)));
  else
    u = policy.maxU.*(2*rand-1);
  end

end