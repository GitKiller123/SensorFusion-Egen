tol = 0.5;

% Number of time steps;
N = 100;

% Define prior
x_0     = [0 0 10 0 0]';
n       = length(x_0);
P_0     = diag([1 1 1 1*pi/180 1*pi/180].^2);

% Covariance 
sigV = 1;
sigOmega = 1*pi/180;
G = [zeros(2,2); 1 0; 0 0; 0 1];
Q = G*diag([sigV^2 sigOmega^2])*G';

% Motion model
motionModel = @coordinatedTurnMotion;

% Random sensor position sequence
S = zeros(2,N);

% Measurement noise covariance
R = diag([10 5*pi/180].^2);

% Measurement model
measModel = @rangeBearingMeasurements;

% function handle for generating sigma points
genSigmaPoints = @sigmaPoints;

for k=1:10
    
    %disp(['k=' num2str(k)]);
    
    % Sample time
    T = rand;
    
    % generate state sequence
    X = genNonLinearStateSequence(x_0, P_0, motionModel, T, Q, N);
    
    % generate measurements
    Y = genNonLinearMeasurementSequence(X, S, measModel, R);
    
    % Kalman filter
    [xEs, PEs, xEf, PEf, xEp, PEp] = ...
        nonLinRTSsmoother(Y, x_0, P_0, motionModel, T, Q, S, measModel, R, genSigmaPoints, 'EKF');
    [xUs, PUs, xUf, PUf, xUp, PUp] = ...
        nonLinRTSsmoother(Y, x_0, P_0, motionModel, T, Q, S, measModel, R, genSigmaPoints, 'UKF');
    [xCs, PCs, xCf, PCf, xCp, PCp] = ...
        nonLinRTSsmoother(Y, x_0, P_0, motionModel, T, Q, S, measModel, R, genSigmaPoints, 'CKF');
    
    [xEs_ref, PEs_ref, xEf_ref, PEf_ref, xEp_ref, PEp_ref] = ...
        nonLinRTSsmoother_ref(Y, x_0, P_0, motionModel, T, Q, S, measModel, R, genSigmaPoints, 'EKF');
    [xUs_ref, PUs_ref, xUf_ref, PUf_ref, xUp_ref, PUp_ref] = ...
        nonLinRTSsmoother_ref(Y, x_0, P_0, motionModel, T, Q, S, measModel, R, genSigmaPoints, 'UKF');
    [xCs_ref, PCs_ref, xCf_ref, PCf_ref, xCp_ref, PCp_ref] = ...
        nonLinRTSsmoother_ref(Y, x_0, P_0, motionModel, T, Q, S, measModel, R, genSigmaPoints, 'CKF');
    
    % Check dimensions
    assert(((size(xEs,1) + size(xEf,1) + size(xEp,1)) == 3*n), ...
           'Incorrect ERTSS state estimate output dimensions n, (Expected [n x N]')
    assert(((size(xEs,2) + size(xEf,2) + size(xEp,2)) == 3*N), ...
           'Incorrect ERTSS state estimate output dimensions N, (Expected [n x N]')
    assert(((size(PEs,1) + size(PEf,1) + size(PEp,1)) == 3*n), ...
           'Incorrect ERTSS covariance output dimensions n, (Expected [n x n x N]')
    assert(((size(PEs,2) + size(PEf,2) + size(PEp,2)) == 3*n), ...
           'Incorrect ERTSS covariance output dimensions n, (Expected [n x n x N]')   
    assert(((size(PEs,3) + size(PEf,3) + size(PEp,3)) == 3*N), ...
           'Incorrect ERTSS covariance output dimensions N, (Expected [n x n x N]')
       
    assert(((size(xUs,1) + size(xUf,1) + size(xUp,1)) == 3*n), ...
           'Incorrect URTSS state estimate output dimensions n, (Expected [n x N]')
    assert(((size(xUs,2) + size(xUf,2) + size(xUp,2)) == 3*N), ...
           'Incorrect URTSS state estimate output dimensions N, (Expected [n x N]')
    assert(((size(PUs,1) + size(PUf,1) + size(PUp,1)) == 3*n), ...
           'Incorrect URTSS covariance output dimensions n, (Expected [n x n x N]')
    assert(((size(PUs,2) + size(PUf,2) + size(PUp,2)) == 3*n), ...
           'Incorrect URTSS covariance output dimensions n, (Expected [n x n x N]')   
    assert(((size(PUs,3) + size(PUf,3) + size(PUp,3)) == 3*N), ...
           'Incorrect URTSS covariance output dimensions N, (Expected [n x n x N]')   
    
           
    assert(((size(xCs,1) + size(xCf,1) + size(xCp,1)) == 3*n), ...
           'Incorrect CRTSS state estimate output dimensions n, (Expected [n x N]')
    assert(((size(xCs,2) + size(xCf,2) + size(xCp,2)) == 3*N), ...
           'Incorrect CRTSS state estimate output dimensions N, (Expected [n x N]')
    assert(((size(PCs,1) + size(PCf,1) + size(PCp,1)) == 3*n), ...
           'Incorrect CRTSS covariance output dimensions n, (Expected [n x n x N]')
    assert(((size(PCs,2) + size(PCf,2) + size(PCp,2)) == 3*n), ...
           'Incorrect CRTSS covariance output dimensions n, (Expected [n x n x N]')   
    assert(((size(PCs,3) + size(PCf,3) + size(PCp,3)) == 3*N), ...
           'Incorrect CRTSS covariance output dimensions N, (Expected [n x n x N]')   
    
    assert(norm(xEs-xEs_ref)<tol,'Incorrect ERTSS means')
    %assert(norm(reshape(PEs-PEs_ref,[n n*N]))<tol,'Incorrect ERTSS covariances')
    assert(norm(xUs-xUs_ref)<tol,'Incorrect URTSS means')
    %assert(norm(reshape(PUs-PUs_ref,[n n*N]))<tol,'Incorrect URTSS covariances')
    assert(norm(xCs-xCs_ref)<tol,'Incorrect CRTSS means')
    %assert(norm(reshape(PCs-PCs_ref,[n n*N]))<tol,'Incorrect CRTSS covariances')
    
    assert(norm(xEf-xEf_ref)<tol,'Incorrect EKF means')
    %assert(norm(reshape(PEf-PEf_ref,[n n*N]))<tol,'Incorrect EKF covariances')
    assert(norm(xUf-xUf_ref)<tol,'Incorrect UKF means')
    %assert(norm(reshape(PUf-PUf_ref,[n n*N]))<tol,'Incorrect UKF covariances')
    assert(norm(xCf-xCf_ref)<tol,'Incorrect CKF means')
    %assert(norm(reshape(PCf-PCf_ref,[n n*N]))<tol,'Incorrect CKF covariances')
     
    assert(norm(xEp-xEp_ref)<tol,'Incorrect predicted EKF means')
    %assert(norm(reshape(PEp-PEp_ref,[n n*N]))<tol,'Incorrect predicted EKF covariances')
    assert(norm(xUp-xUp_ref)<tol,'Incorrect predicted UKF means')
    %assert(norm(reshape(PUp-PUp_ref,[n n*N]))<tol,'Incorrect predicted UKF covariances')
    assert(norm(xCp-xCp_ref)<tol,'Incorrect predicted CKF means')
    %assert(norm(reshape(PCp-PCp_ref,[n n*N]))<tol,'Incorrect predicted CKF covariances')
    
    disp(['Random test ' num2str(k) ' passed'])
    
end

% Plot results
figure(1);clf;hold on;
for ix = 1:n
    subplot(n,3,[3*(ix-1)+1]);
    plot(X(ix,:),'-','linewidth',3);
    hold on
    plot(xEs(ix,:),'-','linewidth',2);
    plot(xEf(ix,:),'--','linewidth',2);
    
    subplot(n,3,[3*(ix-1)+2]);
    plot(X(ix,:),'-','linewidth',3);
    hold on
    plot(xUs(ix,:),'-','linewidth',2);
    plot(xUf(ix,:),'--','linewidth',2);
    
    subplot(n,3,[3*(ix-1)+3]);
    plot(X(ix,:),'-','linewidth',3);
    hold on
    plot(xCs(ix,:),'-','linewidth',2);
    plot(xCf(ix,:),'--','linewidth',2);
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

function [h, H] = rangeBearingMeasurements(x, s)
%RANGEBEARINGMEASUREMENTS calculates the range and the bearing to the
%position given by the state vector x, from a sensor locateed in s
%
%Input:
%   x           [n x 1] State vector
%   s           [2 x 1] Sensor position
%
%Output:
%   h           [2 x 1] measurement vector
%   H           [2 x n] measurement model Jacobian
%
% NOTE: the measurement model assumes that in the state vector x, the first
% two states are X-position and Y-position.

    % Range
    rng = norm(x(1:2)-s);
    % Bearing
    ber = atan2(x(2)-s(2),x(1)-s(1));
    % Measurement vector
    h = [rng;ber];

    % Measurement model Jacobian
    H = [
        (x(1)-s(1))/rng      (x(2)-s(2))/rng     0 0 0;
        -(x(2)-s(2))/(rng^2) (x(1)-s(1))/(rng^2) 0 0 0
        ];

end

function [f, F] = coordinatedTurnMotion(x, T)
%COORDINATEDTURNMOTION calculates the predicted state using a coordinated
%turn motion model, and also calculated the motion model Jacobian
%
%Input:
%   x           [5 x 1] state vector
%   T           [1 x 1] Sampling time
%
%Output:
%   f           [5 x 1] predicted state
%   F           [5 x 5] motion model Jacobian
%
% NOTE: the motion model assumes that the state vector x consist of the
% following states:
%   px          X-position
%   py          Y-position
%   v           velocity
%   phi         heading
%   omega       turn-rate

    % Velocity
    v = x(3);
    % Heading
    phi = x(4);
    % Turn-rate
    omega = x(5);

    % Predicted state
    f = x + [
        T*v*cos(phi);
        T*v*sin(phi);
        0;
        T*omega;
        0];

    % Motion model Jacobian
    F = [
        1 0 T*cos(phi) -T*v*sin(phi) 0;
        0 1 T*sin(phi) T*v*cos(phi)  0;
        0 0 1          0             0;
        0 0 0          1             T;
        0 0 0          0             1
        ];
end

function X = genNonLinearStateSequence(x_0, P_0, f, T, Q, N)
%GENLINEARSTATESEQUENCE generates an N-long sequence of states using a 
%    Gaussian prior and a linear Gaussian process model
%
%Input:
%   x_0         [n x 1] Prior mean
%   P_0         [n x n] Prior covariance
%   f           Motion model function handle
%   T           Sampling time
%   Q           [n x n] Process noise covariance
%   N           [1 x 1] Number of states to generate
%
%Output:
%   X           [n x N] State vector sequence
%

    % Dimension of state vector
    n = length(x_0);

    % allocate memory
    X = zeros(n, N);

    % Generete start state
    X(:,1) = mvnrnd(x_0', P_0)';

    % Generate sequence
    for k = 2:N+1

        % generate noise vector
        q = mvnrnd(zeros(1,n), Q)';

        % Propagate through process model
        [fX, ~] = f(X(:,k-1),T);
        X(:,k) = fX + q;

    end

end

function Y = genNonLinearMeasurementSequence(X, S, h, R)
%GENNONLINEARMEASUREMENTSEQUENCE generates ovservations of the states 
% sequence X using a non-linear measurement model.
%
%Input:
%   X           [n x N+1] State vector sequence
%   S           [n x N] Sensor position vector sequence
%   h           Measurement model function handle
%   R           [m x m] Measurement noise covariance
%
%Output:
%   Y           [m x N] Measurement sequence
%

    % Parameters
    N = size(X,2);
    m = size(R,1);

    % Allocate memory
    Y = zeros(m,N-1);

    for k = 1:N-1
        % Measurement
        [hX,~] = h(X(:,k+1),S(:,k));
        % Add noise
        Y(:,k) = hX + mvnrnd(zeros(1,m), R)';

    end

end