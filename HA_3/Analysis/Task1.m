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

f = @(x)coordinatedTurnMotion(x, T);
h = @(y)dualBearingMeasurement(y, s1, s2);

%X1 = genNonLinearStateSequence(x_01, P_01, f, 0, N);
%X2 = genNonLinearStateSequence(x_02, P_02, f, 0, N);
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

Y1_corr = [];
Y2_corr = [];
for i = 1:N-1
    A1 = [tan(Y1(1,i)) -1; 
        tan(Y1(2,i)) -1];
    b1 = [-s1(2)+tan(Y1(1,i))*s1(1);-s2(2)+tan(Y1(2,i))*s2(1)];
    Y1_corr = [Y1_corr A1\b1];
    A2 = [tan(Y2(1,i)) -1; 
        tan(Y2(2,i)) -1];
    b2 = [-s1(2)+tan(Y2(1,i))*s1(1);-s2(2)+tan(Y2(2,i))*s2(1)];
    Y2_corr = [Y2_corr A2\b2];
end
E_Y1 = mean(Y1_corr');
E_Y2 = mean(Y2_corr');
Y1_COV = cov(Y1_corr');
Y2_COV = cov(Y2_corr');
