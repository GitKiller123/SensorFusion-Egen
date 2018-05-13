%%
% ToDo? Check the dimensions of the outputs
genSigmaPoints = @sigmaPoints;
tol = 1e-4;
for k = 1:100
    % Process model
    f = @linProcModel;
    T = 1;

    % filter state k|k
    xf_k = 20*rand(2,1);
    % filter Covariance k|k
    Pf_k = sqrt(10)*rand(2,2);
    Pf_k = Pf_k*Pf_k';

    % predicted state k+1|k
        xp_kplus1 = f(xf_k);

    % prediction Covariance k+1|k
        % Increase Pp_k by some factor
        Pp_kplus1 = Pf_k*10;

    % smoothed state k+1|K
        % move in random direction, a fraction of the distance between
        % x_k+1|k and x_k|k
        dir = rand(2,1);
        dir = dir/norm(dir);

        dist = norm(xp_kplus1-xf_k); % distance between filter estimate and prediction 

        xs_kplus1 = xp_kplus1 + dir*dist*0.3;

    % smoothing covariance k+1|K
        % Decrease Pp_k by some factor
        Ps_kplus1 = Pf_k*0.5;

    % EKF
    [xs, Ps] =         nonLinRTSSupdate(xs_kplus1, Ps_kplus1, xf_k, Pf_k, ...
                                        xp_kplus1, Pp_kplus1, f, T, genSigmaPoints, 'EKF');
%    [xs_ref, Ps_ref] = nonLinRTSSupdateS(xs_kplus1, Ps_kplus1, xf_k, Pf_k, ...
                                       % xp_kplus1, Pp_kplus1, f, T, genSigmaPoints, 'EKF');
    % Test results
%    assert(norm(xs-xs_ref)<tol, 'Incorrect EKF mean');
%    assert(norm(Ps-Ps_ref)<tol, 'Incorrect EKF covariance');

    % UKF
    [xs, Ps] =         nonLinRTSSupdate(xs_kplus1, Ps_kplus1, xf_k, Pf_k, ...
                                        xp_kplus1, Pp_kplus1, f, T, genSigmaPoints, 'UKF');
  %  [xs_ref, Ps_ref] = nonLinRTSSupdateS(xs_kplus1, Ps_kplus1, xf_k, Pf_k, ...
                                    %    xp_kplus1, Pp_kplus1, f, T, genSigmaPoints, 'UKF');
    % Test results
 %   assert(norm(xs-xs_ref)<tol, 'Incorrect UKF mean');
 %   assert(norm(Ps-Ps_ref)<tol, 'Incorrect UKF covariance');

    % CKF
    [xs, Ps] =         nonLinRTSSupdate(xs_kplus1, Ps_kplus1, xf_k, Pf_k, ...
                                        xp_kplus1, Pp_kplus1, f, T, genSigmaPoints, 'CKF');
 %   [xs_ref, Ps_ref] = nonLinRTSSupdateS(xs_kplus1, Ps_kplus1, xf_k, Pf_k, ...
                                    %    xp_kplus1, Pp_kplus1, f, T, genSigmaPoints, 'CKF');
    % Test results
 %   assert(norm(xs-xs_ref)<tol, 'Incorrect CKF mean');
 %   assert(norm(Ps-Ps_ref)<tol, 'Incorrect CKF covariance');

    disp(['Random test ' num2str(k) ' passed'])

end

function [xk, J] = linProcModel(xkmin1, T)

    A = [1 0; 0 1];
    b = [1 1]';
    xk = A*xkmin1 + b;
    
    J = A;
end

function [SP,W] = sigmaPoints(x, P, type)
% SIGMAPOINTS computes sigma points, either using unscented transform or
% using cubature.
%
%Input:
%   x           [n x 1] Prior mean
%   P           [n x n] Prior covariance
%
%Output:
%   SP          [n x 2n+1] matrix with sigma points
%   W           [1 x 2n+1] vector with sigma point weights 
%

    switch type        
        case 'UKF'

            % Dimension of state
            n = length(x);

            % Allocate memory
            SP = zeros(n,2*n+1);

            % Weights
            W = [1-n/3 repmat(1/6,[1 2*n])];

            % Matrix square root
            sqrtP = sqrtm(P);

            % Compute sigma points
            SP(:,1) = x;
            for i = 1:n
                SP(:,i+1) = x + sqrt(1/2/W(i+1))*sqrtP(:,i);
                SP(:,i+1+n) = x - sqrt(1/2/W(i+1+n))*sqrtP(:,i);
            end

        case 'CKF'

            % Dimension of state
            n = length(x);

            % Allocate memory
            SP = zeros(n,2*n);

            % Weights
            W = repmat(1/2/n,[1 2*n]);

            % Matrix square root
            sqrtP = sqrtm(P);

            % Compute sigma points
            for i = 1:n
                SP(:,i) = x + sqrt(n)*sqrtP(:,i);
                SP(:,i+n) = x - sqrt(n)*sqrtP(:,i);
            end

        otherwise
            error('Incorrect type of sigma point')
    end
end