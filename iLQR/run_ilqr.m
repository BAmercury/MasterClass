% Run ilQR

% Variables
Nx = 4; % number of states
Nu = 1; % number of controls
Tfinal = 5.0; % terminal time
h = 0.05; % time step
Nt = int16(Tfinal/h)+1; % number of time steps
t = 0:h:Tfinal; 

% Initial guess
x0 = [0; 0; 0; 0];
x_goal = [0, 0, pi, 0]'; 
x_traj = kron(ones(1, Nt), x0);
%u_traj = 0.0001*ones(Nt-1, 1);
u_traj = zeros(Nt-1, 1);
%u_traj(1) = 0.0001;

% Setup Cost Weights

% Q-matrix (n, n)
Q = eye(4);
% R-matrix (m, m)
R = 0.1;
% Terminal Cost
Qn = 100*eye(Nx);

% Initial rollout
for k = 1:(Nt-1)
    %x_traj(:, k+1) = feval(@simulink_wrapper_x, x_traj(:, k), u_traj(k));
    x_traj(:, k+1) = dy_rk4(@pole_cart_dynamics,  x_traj(:, k), u_traj(k), h);
end

J = update_cost(x_traj, x_goal, u_traj, Q, R, Qn, Nt); 

disp("Initial Rollout cost: " + J)
%% Full Algorithm

p = ones(Nx, Nt); 
P = zeros(Nx, Nx, Nt); 
d = ones(Nt-1, 1);
K = zeros(Nu, Nx, Nt-1);
dJ = 1.0; 

xn = zeros(Nx,Nt);
un = zeros(Nt-1, 1);

gx = zeros(Nx);
gu = 0.0;
Gxx = zeros(Nx,Nx);
Guu = 0.0;
Gxu = zeros(Nx);
Gux = zeros(1, Nx);

iter = 0;

figure;
%hold on;

while max(abs(d(:))) > 1e-3
    iter = iter + 1;
    disp("Iteration: " + iter)
    
    [dJ, p, P, d, K] = backward_pass(p, P, d, K, x_traj, x_goal, u_traj, Q, R, Nt, Qn, @pole_cart_dynamics); 
    %disp("dJ: " + dJ)
    %disp("Feedback Gains: " + K(:,:,end))
    % Forward rollout with line search
    xn(:, 1) = x_traj(:, 1); 
    alpha = 1.0;
    
    for k = 1:(Nt-1)
        un(k) = u_traj(k) - alpha*d(k) - dot( K(:,:,k), xn(:,k)-x_traj(:,k) );
        %xn(:, k+1) =  feval(@simulink_wrapper_x, xn(:, k), un(k) );  %#ok<*FVAL>
        xn(:, k+1) = dy_rk4(@pole_cart_dynamics, xn(:, k), un(k), h);
    end
    Jn = update_cost(xn, x_goal, un, Q, R, Qn, Nt); 
    
    %disp("Forward rollout cost Jn: " + Jn)
    
    while Jn > (J - 1e-2*alpha*dJ)
        alpha = 0.5*alpha;
        for k = 1:(Nt-1)
            un(k) = u_traj(k) - alpha*d(k) - dot( K(:,:,k), xn(:,k)-x_traj(:,k) ); 
            %xn(:, k+1) = feval(@simulink_wrapper_x, xn(:, k), un(k) );
            xn(:, k+1) = dy_rk4(@pole_cart_dynamics, xn(:, k), un(k), h );
        end
        Jn = update_cost(xn, x_goal, un, Q, R, Qn, Nt);
        %disp("Line Search Jn: " + Jn)
        %disp("alpha: " + alpha)
        %disp("un(k): " + un(k))
    end
    %disp("feedforward max" + max(abs(d(:))))
    J = Jn;
    disp("Updated cost J: " + J)
    x_traj = xn;
    u_traj = un;
    plot(t(1:Nt-1), u_traj)
    pause(0.05)
end
    