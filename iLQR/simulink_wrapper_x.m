function x = simulink_wrapper_x(X, U)
% Same as the other wrapper but this one returns the state
mdl = 'pole_cart_model';

% Need to run the model for one time step to calculate our x_dot
simIn = Simulink.SimulationInput(mdl);
% U inputs
simIn = simIn.setVariable('u_feedforward', U);
simIn = simIn.setVariable('x0', X);

results = sim(simIn);

x = results.simout_x;
% Only need to grab from one timestamp
x = x.Data(end, :)';