% Number of time steps;
N = 500;
type = 'EKF'

% Define prior
x_0     = rand(5,1); 
n       = length(x_0); 
P_0     = rand(5,5);
P_0     = P_0*P_0';

% Sample time
T = 1;

% Covariance
sigV = 1;
sigOmega = 1*pi/180;
G = [zeros(2,2); 1 0; 0 0; 0 1];
Q = G*diag([sigV^2 sigOmega^2])*G';

% Motion model function handle. Note how sample time T is inserted into the function.
f = @(x) coordinatedTurnMotion(x, T);

% generate state sequence
X = genNonLinearStateSequence(x_0, P_0, f, Q, N);
% Random sensor position sequence
s1 = [0, 100]';
s2 = [0, -100]';
% Measurement noise covariance
R = diag([2*pi/180 2*pi/180].^2);
% Measurement model
h = @(X) dualBearingMeasurement(X, s1, s2);
% Generate measurements
Y = genNonLinearMeasurementSequence(X, h, R);

[xf, Pf, xp, Pp] = nonLinearKalmanFilter(Y, x_0, P_0, f, Q, h, R, type);