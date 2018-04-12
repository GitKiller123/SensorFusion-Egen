clear all; close all;
Task = 'b';

N = 20;

A = 1;
H = 1;

Q = 1.5;
R = 2.5;

x_0 = 2;
P_0 = 6;

X = genLinearStateSequence(x_0, P_0, A, Q, N);
Y = genLinearMeasurementSequence(X, H, R);

switch Task
    case {'a'}
        plot(X(:,2:end), 'k')
        hold all
        plot(Y, 'r*')
        plot(X(:,2:end)+3*sqrt(R), 'b--')
        plot(X(:,2:end)-3*sqrt(R), 'b--')
        legend('State sequence', 'Measurement', '3sigma')
    case {'b'}
        [Xhat, P] = kalmanFilter(Y, x_0, P_0, A, Q, H, R);
        plot(Xhat, 'b')
        hold all
        plot(X(:,2:end), 'k')
        plot(Y, 'r*')
        plot(X(:,2:end)+3*sqrt(R), 'b--')
        plot(X(:,2:end)-3*sqrt(R), 'b--')
        legend('Kalman', 'State sequence', 'Measurement', '3sigma')
end
