function cost = terminal_cost(x, x_goal, Qn)

cost= 0.5*(x-x_goal)'*Qn*(x-x_goal);