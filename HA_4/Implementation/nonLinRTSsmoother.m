function [xs, Ps, xf, Pf, xp, Pp] = ...
    nonLinRTSsmoother(Y, x_0, P_0, f, T, Q, S, h, R, SP_func, type)
%NONLINRTSSMOOTHER Filters measurement sequence Y using a 
% non-linear Kalman filter. 
%
%Input:
%   Y           [m x N] Measurement sequence for times 1,...,N
%   x_0         [n x 1] Prior mean for time 0
%   P_0         [n x n] Prior covariance
%   f                   Motion model function handle
%   T                   Sampling time
%   Q           [n x n] Process noise covariance
%   S           [n x N] Sensor position vector sequence
%   h                   Measurement model function handle
%   R           [n x n] Measurement noise covariance
%   sigmaPoints Handle to function that generates sigma points.
%   type        String that specifies type of non-linear filter/smoother
%
%Output:
%   xf          [n x N]     Filtered estimates for times 1,...,N
%   Pf          [n x n x N] Filter error convariance
%   xp          [n x N]     Predicted estimates for times 1,...,N
%   Pp          [n x n x N] Filter error convariance
%   xs          [n x N]     Smoothed estimates for times 1,...,N
%   Ps          [n x n x N] Smoothing error convariance

% your code here!
% We have offered you functions that do the non-linear Kalman prediction and update steps.
% Call the functions using
% [xPred, PPred] = nonLinKFprediction(x_0, P_0, f, T, Q, sigmaPoints, type);
% [xf, Pf] = nonLinKFupdate(xPred, PPred, Y, S, h, R, sigmaPoints, type);

%% Parameters
N = size(Y,2);

n = length(x_0);
m = size(Y,1);

x_temp = x_0;
P_temp = P_0;

%% Data allocation
xf = zeros(n,N);
Pf = zeros(n,n,N);
xp = zeros(n,N);
Pp = zeros(n,n,N);
for k = 1:N
    [x_temp, P_temp] = nonLinKFprediction_HA_4(x_temp, P_temp, f, T, Q, SP_func, type);
    xp(:,k) = x_temp;
    Pp(:,:,k) = P_temp;
    [x_temp, P_temp] = nonLinKFupdate_HA_4(x_temp, P_temp, Y(:,k), S(:,k), h, R, SP_func, type);
    xf(:,k) = x_temp;
    Pf(:,:,k) = P_temp;
end

xs = zeros(n,N);
Ps = zeros(n,n,N);
xs(:,end) = xf(:,end);
Ps(:,:,end) = Pf(:,:,end);

for k = N-1:-1:1
    [xs(:,k), Ps(:,:,k)] = nonLinRTSSupdate(xs(:,k+1), Ps(:,:,k+1),xf(:,k),Pf(:,:,k),xp(:,k+1), ...
                                     Pp(:,:,k+1),f,T, SP_func,type);
end
end