clear all; close all;

N = 20;

A = 1;
H = 1;

Q = 1.5;
R = 2.5;

x_0 = 2;
P_0 = 6;

X = genLinearStateSequence(x_0, P_0, A, Q, N);
Y = genLinearMeasurementSequence(X, H, R);

plot(X(:,2:end), 'k')
hold all
plot(Y, 'r*')
plot(X(:,2:end)+3*sqrt(R), 'b--')
plot(X(:,2:end)-3*sqrt(R), 'b--')
legend('State sequence', 'Measurement', '3sigma')
% plot([1 length(Y)],[3*sqrt(R) 3*sqrt(R)])