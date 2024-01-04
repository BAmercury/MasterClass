% Run model

% Gains
K = [ -70.7107  -37.8345  105.5298   20.9238];

x0 = [0;0;0;0];

t = 0:0.01:5;
u_feedforward = timeseries(zeros(length(t), 1), t);
u0 = 0;


% Test Finite Diff

[A, B] = finite_diff(@simulink_wrapper, x0, u0, 1e-4, 1e-4)


