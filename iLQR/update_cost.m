function J = update_cost(x_traj, x_goal, u_traj, Q, R, Qn)

J = 0.0;

for i = 1:length(x_traj)
    J = J + cost_to_go(x_traj(i), x_goal, u_traj(i), Q, R);
end
J = J + terminal_cost(x_traj(end), x_goal, Qn);