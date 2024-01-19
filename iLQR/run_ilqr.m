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
x_goal = [0, pi, 0, 0]'; 
x_traj = kron(ones(1, Nt), x0);
u_traj = 0.001*ones(Nt-1, 1);

% Setup Cost Weights

% Q-matrix (n, n)
Q = eye(4);
% R-matrix (m, m)
R = 0.1;
% Terminal Cost
Qn = 100*eye(Nx);

% Initial rollout
for k = 1:(Nt-1)
    x_traj(:, k+1) = feval(@simulink_wrapper_x, x_traj(:, k), u_traj(k)); 
end

J = update_cost(x_traj, x_goal, u_traj, Q, R, Qn); 

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

while max(abs(d(:))) > 1e-3
    iter = iter + 1;
    disp("Iteration: " + iter)
    
    [dJ, p, P, d, K] = backward_pass(p, P, d, K, x_traj, x_goal, u_traj, Q, R, Nt, Qn, @simulink_wrapper); 
    disp("dJ: " + dJ)
    disp("Feedback Gains: " + K(:,:,end))
    % Forward rollout with line search
    xn(:, 1) = x_traj(:, 1); 
    alpha = 1.0;
    
    for k = 1:(Nt-1)
        un(k) = u_traj(k) - alpha*d(k) - dot( K(:,:,k), (xn(:,k)-x_traj(:,k)) );
        xn(:, k+1) =  feval(@simulink_wrapper_x, xn(:, k), un(k) );  %#ok<*FVAL>
    end
    Jn = update_cost(xn, x_goal, un, Q, R, Qn); 
    
    disp("Forward rollout cost Jn: " + Jn)
    
    while isnan(Jn) || round(Jn, 2) > round(J - 1e-2*alpha*dJ, 2)
        alpha = 0.5*alpha;
        for k = 1:(Nt-1)
            un(k) = u_traj(k) - alpha*d(k) - dot( K(:,:,k), (xn(:,k)-x_traj(:,k)) ); 
            xn(:, k+1) = feval(@simulink_wrapper_x, xn(:, k), un(k) );
        end
        Jn = update_cost(xn, x_goal, un, Q, R, Qn);
        disp("Line Search Jn: " + Jn)
        disp("alpha: " + alpha)
        disp("un(k): " + un(k))
    end
    disp("feedforward max" + max(abs(d(:))))
    J = Jn;
    x_traj = xn;
    u_traj = un;
end
    