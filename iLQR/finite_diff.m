function [A, B] = finite_diff(fun_handle, X0, U0, DX, DU)
% Numerically linearizes plant function using symmetric difference
% quotients

% Get number of states and controls
n = length(X0);
m = length(U0);

% A Matrix
A = zeros(n, n);
for i = 1:n

        x_plus = X0;
        x_minus = X0;

        x_plus(i) = x_plus(i) + DX;
        x_minus(i) = x_minus(i) - DX;
        
        x_dot_plus = feval(fun_handle, x_plus, U0);
        x_dot_minus = feval(fun_handle, x_minus, U0);
        
        A(:, i) = (x_dot_plus - x_dot_minus) / (2*DX);
end

% B Matrix
B = zeros(n, m);
for i = 1:m

        u_plus = U0;
        u_minus = U0;

        u_plus(i) = u_plus(i) + DU;
        u_minus(i) = u_minus(i) - DU;
        
        x_dot_plus = feval(fun_handle, X0, u_plus);
        x_dot_minus = feval(fun_handle, X0, u_minus);
        
        B(:, i) = (x_dot_plus - x_dot_minus) / (2*DU);
end