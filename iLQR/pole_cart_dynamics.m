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

s = sin(x(3));
c = cos(x(3)); 

x_dot = zeros(4, 1); % 4 state variables

denom = 1 / ((m_c + m_p) - (m_p^2*l_p^2*c^2 / (I_p + m_p*l_p^2) ) );


x_dot(1) = x(2); % cart velocity 
x_dot(2) = ( u - b_c*x(2) + (m_p^2*l_p^2*g*s*c)/(I_p + m_p*l_p^2) + m_p*l_p*x(4)^2*s) * denom;
x_dot(3) = x(4); % pen ang vel
x_dot(4) = (-m_p*l_p*x_dot(2)*c - m_p*g*l_p*s) / (I_p + m_p*l_p^2); % pen ang accel