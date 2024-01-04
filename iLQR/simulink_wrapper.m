function x_dot = simulink_wrapper(X, U)

mdl = 'pole_cart_model';

% Need to run the model for one time step to calculate our x_dot
simIn = Simulink.SimulationInput(mdl);
t_end = double(get_param(mdl, 'StopTime'));
t = 0:0.01:t_end;
% U inputs
simIn = simIn.setVariable('u_feedforward', timeseries(U*ones(length(t), 1), t));
simIn = simIn.setVariable('x0', X);

results = sim(simIn);

x_dot = results.simout_xdot;
% Only need to grab from one timestamp
x_dot = x_dot.Data(1, :)';