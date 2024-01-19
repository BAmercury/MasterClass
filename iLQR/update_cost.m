function J = update_cost(x_traj, x_goal, u_traj, Q, R, Qn, Nt)

% Q and R are the state/input weighting matrices
% Qn is the terminal cost weighting matrix 

J = 0.0;

for i = 1:(Nt-1)
    J = J + cost_to_go(x_traj(:, i), x_goal, u_traj(i), Q, R);
end
J = J + terminal_cost(x_traj(:, end), x_goal, Qn);