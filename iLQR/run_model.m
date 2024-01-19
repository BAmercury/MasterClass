% Run model

% Gains
K = [ -70.7107  -37.8345  105.5298   20.9238];

x0 = [0;0;0;0];

t = 0:0.01:5;
u_feedforward = 1;
u0 = 0;

h = 0.05;  % simulation time step 

% Test Finite Diff

[A, B] = finite_diff(@simulink_wrapper, x0, u0, 1e-4, 1e-4);


% Setup Cost Weights

% Q-matrix (n, n)
Q = eye(4);
% R-matrix (m, m)
R = 0.1;
% Terminal Cost
Qn = 100*eye(4);
