clear all; close all;

%% True track
% Sampling period
T = 0.1;
% Length of time sequence
K = 600;
% Allocate memory
omega = zeros(1,K+1);
% Turn rate
omega(200:400) = -pi/201/T;
% Initial state
x0 = [0 0 20 0 omega(1)]';
% Allocate memory
X = zeros(length(x0),K+1);
X(:,1) = x0;
% Create true track
for i=2:K+1
% Simulate
X(:,i) = coordinatedTurnMotion(X(:,i-1), T);
% Set turn?rate
X(5,i) = omega(i);
end

%% Task 3

task3.x_0 = [0 0 0 0 0]';
task3.P_0 = diag([10^2 10^2 10^2 (5*pi/180)^2 (pi/180)^2]);
task3.s1 = [280 -80]';
task3.s2 = [280 -200]';
sigma_phi1 = 4*pi/180;
sigma_phi2 = sigma_phi1;
R = diag([sigma_phi1^2 sigma_phi2^2]);

T = 0.1;
f = @(x)coordinatedTurnMotion(x, T);
h = @(x)dualBearingMeasurement(x, task3.s1, task3.s2);

type = 'CKF';

sigma_v = 1;
sigma_w = pi/180;
Q = 1*diag([0 0 sigma_v 0 sigma_w]);

task3.Y = genNonLinearMeasurementSequence(X, h, R);
[task3.xf, task3.Pf, task3.xp, task3.Pp] = nonLinearKalmanFilter(task3.Y, task3.x_0, task3.P_0, f, Q, h, R, type);
task3.Ypos = ang2pos(task3.Y, task3.s1, task3.s2);

plot(X(1,:),X(2,:))
hold all
% plot(task3.Ypos(1,:),task3.Ypos(2,:),'--r')
plot(task3.xf(1,:),task3.xf(2,:),'k')
% legend('True state', 'Measurements', 'CKF filtered')
