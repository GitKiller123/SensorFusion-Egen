tol = 1e-1;
for k=1:100
    % Define prior
    x_0     = [0 0 10 0 0]';
    n       = length(x_0);
    P_0     = diag([1 1 1 1*pi/180 1*pi/180].^2);
    % Sample time
    T = rand;
    % Covariance
    sigV = 1;
    sigOmega = 1*pi/180;
    G = [zeros(2,2); 1 0; 0 0; 0 1];
    Q = G*diag([sigV^2 sigOmega^2])*G';
    
    motionModel = @(x) coordinatedTurnMotion(x, T);
    hSP = @sigmaPoints;
    %hSP = @reference.sigmaPoints;
    
    % EKF
    [xp, Pp] = nonLinKFprediction(x_0, P_0, motionModel, Q, 'EKF');
    %[xp_ref, Pp_ref] = nonLinKFprediction(x_0, P_0, motionModel, Q, 'EKF');

    
    % UKF
    [xp, Pp] = nonLinKFprediction(x_0, P_0, motionModel, Q, 'UKF');
    %[xp_ref, Pp_ref] = nonLinKFprediction(x_0, P_0, motionModel, Q, 'UKF');

    
    % CKF
    [xp, Pp] = nonLinKFprediction(x_0, P_0, motionModel, Q, 'CKF');
    %[xp_ref, Pp_ref] = nonLinKFprediction(x_0, P_0, motionModel, Q, 'CKF');

    
    disp(['Random test ' num2str(k) ' passed'])
    
end