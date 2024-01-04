function cost = cost_to_go(x, x_goal, u, Q, R)

cost = 0.5*((x-x_goal)'*Q*(x-x_goal)) + 0.5*R*u*u;