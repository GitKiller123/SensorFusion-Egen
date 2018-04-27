clear all; close all;

x_01 = [120 120]';
x_02 = [120 -20]';

P_01 = [5^2 0;
        0 10^2];
P_02 = P_01;

s1 = [0 100]';  %Sensor 1 position
s2 = [100 0]';  %Sensor 2 position
Sigma_phi = 0.1*pi/180; %Measurement noise

N = 1000;
T = 1;
type = {'EKF' 'UKF' 'CKF'};

f = @(y)dualBearingMeasurement(y, s1, s2);
h = @(y)dualBearingMeasurement(y, s1, s2);
Q = 0;

X1 = [];
X2 = [];
for i = 1:N
    X1 = [X1 mvnrnd(x_01, P_01)'];
    X2 = [X2 mvnrnd(x_02, P_02)'];
end
R = [Sigma_phi^2 0;
        0 Sigma_phi^2];
Y1 = genNonLinearMeasurementSequence(X1, h, R);
Y2 = genNonLinearMeasurementSequence(X2, h, R);

Y1_corr = ang2pos(Y1, s1, s2);
Y2_corr = ang2pos(Y2, s1, s2);

%Task 1a
E_Y1 = mean(Y1_corr');
E_Y2 = mean(Y2_corr');
Y1_COV = cov(Y1_corr');
Y2_COV = cov(Y2_corr');

Run_plot = true;

%Task 1b
for i = 1:3
    [task1.(type{i}).x1, task1.(type{i}).P1] = nonLinKFprediction(x_01, P_01, f, Q, type{i});
    [task1.(type{i}).x2, task1.(type{i}).P2] = nonLinKFprediction(x_02, P_02,f, Q, type{i});
    task1.(type{i}).x1corr = ang2pos(task1.(type{i}).x1, s1, s2);
    task1.(type{i}).x2corr = ang2pos(task1.(type{i}).x2, s1, s2);
end

%Task 1c-e
%Plot shit, analyze etc.
if Run_plot
    
end




