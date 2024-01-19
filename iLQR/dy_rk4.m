function yout = dy_rk4(fcn_handle, x, u, h)

f1 = fcn_handle(x, u);
f2 = fcn_handle(x + 0.5*h*f1, u);
f3 = fcn_handle(x + 0.5*h*f2, u);
f4 = fcn_handle(x + h*f3, u);

yout = x + (h/6.0)*(f1 + 2*f2 + 2*f3 + f4);