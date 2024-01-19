function x_dot = simulink_wrapper(X, U)

mdl = 'pole_cart_model';

% Need to run the model for one time step to calculate our x_dot
simIn = Simulink.SimulationInput(mdl);

% U inputs
simIn = simIn.setVariable('u_feedforward', U);
simIn = simIn.setVariable('x0', X);

results = sim(simIn);

x_dot = results.simout_xdot;
% Only need to grab from one timestamp
x_dot = x_dot.Data(end, :)';