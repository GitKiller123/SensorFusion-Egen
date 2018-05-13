%% Test the PF filter update function 
N = 20000;

for i=1:10
    % Define p(x)
    sigma = 1+rand(1);
    p_mean = [2+rand(1) 1]';
    P = [sigma^2 0      ; ...
         0       (sigma)^2];

    % Sample X_kmin1 with W_kmin1 from p(x) (2D) via proposal q(x) (uniform)

    X_kmin1 = (2*rand(2,N)-1) * sigma * 4 + p_mean; % Take samples from q(x)

    W = exp(sum(-0.5 * ((X_kmin1-p_mean)'/P)'.*(X_kmin1-p_mean),1)); % Unnormalized weights
    W_kmin1 = W/sum(W,2); % Normalize

    meanAppr_kmin1 = X_kmin1 * W_kmin1';
    covAppr_kmin1 = (X_kmin1-meanAppr_kmin1) * ((X_kmin1-meanAppr_kmin1)' .* W_kmin1'); % mxN * (Nxm .* N*1)

    % Run pfFilterStep on the particles and weights, using linear gaussian models
    H = [0 2];
    meas_h = @(X_k) (h(X_k,H));
    meas_R = 10;
    A = [2 0 ; 0 0.5];
    proc_f = @(X_kmin1) (f(X_kmin1, A));
    proc_Q = [5 1; 1 5];

    % measurement
    yk = H*A*p_mean + 2*rand(1); % Add two to offset measurement from deterministic value.

    % Filter update step
    [X_k, W_k] = pfFilterStep(X_kmin1, W_kmin1, yk, proc_f, proc_Q, meas_h, meas_R);
    
    assert((size(X_k,1) == size(X_kmin1,1)) && (size(X_k,2) == size(X_kmin1,2)), ...
            'Output X_k dimensions are incorrect');
    assert((size(W_k,1) == size(W_kmin1,1)) && (size(W_k,2) == size(W_kmin1,2)), ...
            'Output W_k dimensions are incorrect');
    
    % Compute mean and covariance and compare with analytical value (Use Kalman
    % update)
    meanAppr = X_k * W_k';
    covAppr = (X_k-meanAppr) * ((X_k-meanAppr)' .* W_k'); % mxN * (Nxm .* N*1)

    % Kalman Predict
    Pk_pred = A*P*A'+proc_Q;
    xk_pred = A*p_mean;

    % Kalman Update
    S_k = H*Pk_pred*H'+meas_R;
    vk = yk - H*xk_pred;
    Kk = Pk_pred*H'/S_k;

    meanAnal = xk_pred + Kk*vk;
    covAnal = Pk_pred - Kk*S_k*Kk';

    assert(norm(meanAnal-meanAppr) < 0.2 ,'Filter does not produce particles that correctly reflect the posterior mean')
    assert(norm(covAnal-covAppr) < 1 ,'Filter does not produce particles that correctly reflect the posterior mean')

    disp(['Test ' num2str(i) ' succeeded!'])
end

function X_k = f(X_kmin1, A)
%
% X_kmin1:  [n x N] N states vectors at time k-1
% A:        [n x n] Matrix such that x_k = A*x_k-1 + q_k-1
    X_k = A*X_kmin1;
end

function H_k = h(X_k, H)
%
% X_k:  [n x N] N states
% H:    [m x n] Matrix such that y = H*x + r_k
    H_k = H*X_k;
end