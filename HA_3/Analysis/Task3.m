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

f = @(x)coordinatedTurnMotion(x, T);
h = @(x)dualBearingMeasurement(x, task3.s1, task3.s2);

type = 'CKF';

sigma_v = 1;
sigma_w = pi/180;
Qm = [0.01 1 100];
i_best = 3.1;
l_best = 0.08;
Q = 0.08*diag([0 0 sigma_v^2 0 3.1*sigma_w^2]);

x_error_tot = zeros(3,size(X,2)-1);
for i = 1:3
    task3.Y = genNonLinearMeasurementSequence(X, h, R);
    task3.Ypos = ang2pos(task3.Y, task3.s1, task3.s2);
    Q_temp = Qm(i)*Q;
    [task3.xf, task3.Pf, task3.xp, task3.Pp] = nonLinearKalmanFilter(task3.Y, task3.x_0, task3.P_0, f, Q_temp, h, R, type);
    figure('units','normalized','outerposition',[0 0 1 1])
    subplot(2,2,1:2)
    plot(X(1,:),X(2,:))
    hold all
    plot(task3.xf(1,:),task3.xf(2,:),'k--')
    plot(task3.s1(1),task3.s1(2),'mx')
    plot(task3.s2(1),task3.s2(2),'mx')
    title(['UKF filtered with Q = '+string(Qm(i))+'*Q_{original}'])
    legend('State sequence', 'Filtered','Sensors')
    xlabel('x')
    ylabel('y')
    subplot(2,2,3)
    plot(X(1,:),X(2,:))
    hold all
    plot(task3.xf(1,:),task3.xf(2,:),'k--')
    plot(task3.s1(1),task3.s1(2),'mx')
    for l = 1:size(task3.xf,2)/5
        xy_xf = sigmaEllipse2D(task3.xf(1:2,l*5),task3.Pf(1:2,1:2,l*5));
        plot(xy_xf(1,:),xy_xf(2,:),'--','color',[0 0.5 0])
    end
    plot(task3.s2(1),task3.s2(2),'mx')
    title(['UKF filtered with Q = '+string(Qm(i))+'*Q_{original} with 3-Sigma points'])
    legend('State sequence', 'Filtered','Sensors', '3-Sigma')
    xlabel('x')
    ylabel('y')
    subplot(2,2,4)
    plot(X(1,:),X(2,:))
    hold all
    plot(task3.Ypos(1,:),task3.Ypos(2,:),'r*')
    plot(task3.xf(1,:),task3.xf(2,:),'k--')
    plot(task3.s1(1),task3.s1(2),'mx')
    plot(task3.s2(1),task3.s2(2),'mx')
    title(['UKF filtered with Q = '+string(Qm(i))+'*Q_{original} with measurements'])
    legend('State sequence', 'Measurements', 'Filtered', 'Sensors')
    xlabel('x')
    ylabel('y')
    
    
    x_error = task3.xf(1:2,:)-X(1:2,2:end);
    for p = 1:size(x_error,2)
        x_error_tot(i,p) = sqrt(x_error(1,p)^2+x_error(2,p)^2);
    end
    
end
figure('units','normalized','outerposition',[0 0 1 1])
subplot(3,1,1)
plot(x_error_tot(1,:),'*')
axis([-inf inf 0 100])
title('Distance between X(i)-Xf(i) for Q = 0.01*Q_{original}')
xlabel('Sample number')
ylabel('X-Xf')
subplot(3,1,2)
plot(x_error_tot(2,:),'*')
axis([-inf inf 0 100])
title('Distance between X(i)-Xf(i) for Q = 1*Q_{original}')
xlabel('Sample number')
ylabel('X-Xf')
subplot(3,1,3)
plot(x_error_tot(3,:),'*')
axis([-inf inf 0 100])
title('Distance between X(i)-Xf(i) for Q = 100*Q_{original}')
xlabel('Sample number')
ylabel('X-Xf')
