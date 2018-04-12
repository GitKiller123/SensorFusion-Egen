function [x, P] = kalmanFilter(Y, x_0, P_0, A, Q, H, R)
%KALMANFILTER Filters measurements sequence Y using a Kalman filter. 
%
%Input:
%   Y           [m x N] Measurement sequence
%   x_0         [n x 1] Prior mean
%   P_0         [n x n] Prior covariance
%   A           [n x n] State transition matrix
%   Q           [n x n] Process noise covariance
%   H           [m x n] Measurement model matrix
%   R           [m x m] Measurement noise covariance
%
%Output:
%   x           [n x N] Estimated state vector sequence
%   P           [n x n x N] Filter error convariance
%

%% Parameters
N = size(Y,2);

n = length(x_0);
m = size(Y,1);

x_temp = x_0;
P_temp = P_0;

%% Data allocation
x = zeros(n,N);
P = zeros(n,n,N);
for k = 1:N
    [x_temp, P_temp] = linearPrediction(x_temp, P_temp, A, Q);
    [x_temp, P_temp] = linearUpdate(x_temp, P_temp, Y(:,k), H, R);
    x(:,k) = x_temp;
    P(:,:,k) = P_temp;    
end
   
end