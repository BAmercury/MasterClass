function [dJ, p, P, d, K] = backward_pass(p, P, d, K, x_traj, x_goal, u_traj, Q, R, Nt, Qn, h, fcn_handle)

% Take some nominal or guess trajectory, linearize around it
% Backwards we go to compute our approximate value/cost-to-go function 

% Nt = Number of time steps
% p, P: Gradient and Hessian matrices of cost-to-go function
% d: feedforward 
% K: feedback gains

% function returns an array of the 
% p: Gradient of the cost-to-go over the trajectory
% P: Hessian of cost-to-go cost over the trajectory 

% Initialize some variables
dJ = 0; % Cost delta
% Terminal costs 
p(:, Nt) = Qn * (x_traj(:, Nt) - x_goal); 
P(:, :, Nt) = Qn; 
% backwards pass
for i = (Nt-1):-1:1
    
    
    % Calculate derivatives
    q = Q*(x_traj(:, i) - x_goal);
    r = R*u_traj(i);
    
    % Linearize dynamics at that trajectory with the control
    [A, B] = finite_diff(fcn_handle, x_traj(:, i), u_traj(i), 1e-4, 1e-4);
    C = [1 0 0 0;
     0 0 1 0];
    D = [0;
     0];
    % discretize
    sys_d = c2d(ss(A, B, C, D), h, 'zoh');
    A = sys_d.A;
    %Ad = eye(4,4) + A*h;
    B = sys_d.B;
    %Bd = B*h;
    gx = q + A' * p(:, i+1);
    gu = r + B' * p(:, i+1); 
    
    %A = Ad;
    %B = Bd;
    
    % iLQR (Gauss-Newton)
    Gxx = Q + A'*(P(:, :, i+1)*A); % Nx x Nx
    Guu = R + B'*(P(:, :, i+1)*B); % scalar
    Gxu = A' * (P(:, :, i+1)*B); % Nx x 1
    Gux = B' * (P(:, :, i+1)*A); % 1 x Nx
    
    % Regularization
    beta = 0.1;
    
    ifPosDef = false;
    
    while ~ifPosDef
        try chol([Gxx Gxu; Gux Guu]);
            % Matrix is symmetric positive definite
            ifPosDef = true; % exit loop
        catch ME
            % Matrix is not symmetric positive definite
            ifPosDef = false;
            Gxx = Gxx + A'*beta*eye*A;
            Guu = Guu + B'*beta*eye*B;
            Gxu = Gxu + A'*beta*eye*B;
            Gux = Gux + B'*beta*eye*A;
            beta = 2*beta;
            disp("Regularizing G")
        end
    end
        
     
    
    d(i) = inv(Guu)*gu;
    K(:,:,i) = inv(Guu)*Gux;
    
    p(:,i) = gx - K(:,:,i)'*gu + K(:,:,i)' * Guu * d(i) - Gxu*d(i);
    P(:,:,i) = Gxx + K(:,:,i)'*Guu*K(:,:,i) - Gxu*K(:,:,i) - K(:,:,i)'*Gux;
               
    dJ = dJ + gu'*d(i);
end
    

    


