function x_dot = pole_cart_dynamics(x, u)

% State variables:
% x = [cart position, cart velocity, pendulum angle, pendulum angular
% velocity]'

% paramters
m_c = 0.5; % (kg) cart mass
m_p = 0.2; % (kg) pendulum mass
b_c = 0.1; % (N/m/sec) friction parameter for cart
l_p = 0.3; % (m) pendulum arm length
I_p = 0.006; % (kg-m^2) moment of inertia of pendulum
g = 9.81; % (m/s^2) gravity 


x_dot = zeros(4, 1); % 4 state variables
p = I_p*(m_c+m_p)+m_c*m_p*l_p^2; %denominator for the A and B matrices

x_dot(1) = x(2);
x_dot(2) = (-(I_p+m_p*l_p^2)*b_c/p)*x(2) + ((m_p^2*g*l_p^2) / p)*x(3) + ( (I_p+m_p*l_p^2) / p)*u;
x_dot(3) = x(4);
x_dot(4) = (-(m_p*l_p*b_c)/p)*x(2) +  (m_p*g*l_p*(m_c+m_p)/p)*x(3) +  (m_p*l_p/p)*u;
