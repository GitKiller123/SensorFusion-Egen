clear all; close all;
%% task 1a
% True track
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

run_plot1 = false;
run_plot2 = true;

sk = [280; -140];
S = kron(ones(1,K),sk);
sigma_r = 15;
sigma_phi = 2*pi/180;
R = diag([sigma_r^2 sigma_phi^2]);
sigma_v = 1;
sigma_w = pi/180;
Q = 0.08*diag([0 0 sigma_v^2 0 3.1*sigma_w^2]);
P0 = diag([10^2 10^2 10^2 (5*pi/180)^2 (pi/180)^2]);
type = {'UKF'};
f = @(x,T)coordinatedTurnMotion(x, T);
h = @(x,s)rangeBearingMeasurements(x, s);
Y = genNonLinearMeasurementSequence_HA_4(X, h, R, sk); %Must be updated for moving sensors
for i = 1:length(type)
    [Output.(type{i}).A.xs, Output.(type{i}).A.Ps, Output.(type{i}).A.xf, Output.(type{i}).A.Pf, Output.(type{i}).A.xp, Output.(type{i}).A.Pp] = ...
        nonLinRTSsmoother(Y, x0, P0, f, T, Q, S, h, R, @sigmaPoints, type{i});
end
if run_plot1
    figure('units','normalized','outerposition',[ 0 0 1 1])
    Y_pos = converter(Y,sk);    %Must be updated for moving sensors
    plot(Y_pos(1,:),Y_pos(2,:),'ro')
    hold all
    grid on
    plot(X(1,2:end),X(2,2:end),'LineWidth',3)
    legend('Measurements', 'True states')
    title('Measurements and true states')
    hgexport(gcf, 'task_1A_1.png', hgexport('factorystyle'), 'Format', 'png');
    figure('units','normalized','outerposition',[ 0 0 1 1])
    for i = 1:length(type)
        subplot(1,2,1)
        plot(X(1,2:end),X(2,2:end),'LineWidth',3)
        hold all
        grid on
        plot(Output.(type{i}).A.xf(1,:), Output.(type{i}).A.xf(2,:),'r','LineWidth',3)
        for l = 6:5:K
            xy_xf = sigmaEllipse2D(Output.(type{i}).A.xf(1:2,l), Output.(type{i}).A.Pf(1:2,1:2,l));
            plot(xy_xf(1,:),xy_xf(2,:),'--','color',[0 0.5 0])
        end
        title('True states and filtered states')
        legend({'True states','Filtered states','3-$\sigma$ level'},'Interpreter','Latex')
        xlabel('Position [x]')
        ylabel('Position [y]')
        subplot(1,2,2)
        plot(X(1,2:end),X(2,2:end),'LineWidth',3)
        hold all
        grid on
        plot(Output.(type{i}).A.xs(1,:), Output.(type{i}).A.xs(2,:),'r','LineWidth',3)
        for l = 6:5:K
            xy_xs = sigmaEllipse2D(Output.(type{i}).A.xs(1:2,l), Output.(type{i}).A.Ps(1:2,1:2,l));
            plot(xy_xs(1,:),xy_xs(2,:),'--','color',[0 0.5 0])
        end
        title('True states and smoothed states')
        legend({'True states','Smoothed states','3-$\sigma$ level'},'Interpreter','Latex')
        xlabel('Position [x]')
        ylabel('Position [y]')
        hgexport(gcf, 'task_1A_2.png', hgexport('factorystyle'), 'Format', 'png');
    end
    figure('units','normalized','outerposition',[ 0 0 1 1])
    for l = 30:30:K
        if l/30 > 6
           break
        end
        subplot(2,3,l/30)
        xy_xf = sigmaEllipse2D(Output.(type{i}).A.xf(1:2,l), Output.(type{i}).A.Pf(1:2,1:2,l));
        plot(xy_xf(1,:),xy_xf(2,:))
        hold all
        grid on
        xy_xs = sigmaEllipse2D(Output.(type{i}).A.xs(1:2,l), Output.(type{i}).A.Ps(1:2,1:2,l));
        plot(xy_xs(1,:),xy_xs(2,:),'r')
        title(['k = ' num2str(l)])
        legend({'Filtered 3-$\sigma$ level','Smoothed 3-$\sigma$ level'},'Interpreter','Latex')
        xlabel('Position [x]')
        ylabel('Position [y]')
    end
    hgexport(gcf, 'task_1A_3.png', hgexport('factorystyle'), 'Format', 'png');
end

%% task 1b
Y(1,150) = Y(1,150) + 100;
Y_pos = converter(Y, sk);

for i = 1:length(type)
    [Output.(type{i}).B.xs, Output.(type{i}).B.Ps, Output.(type{i}).B.xf, Output.(type{i}).B.Pf, Output.(type{i}).B.xp, Output.(type{i}).B.Pp] = ...
        nonLinRTSsmoother(Y, x0, P0, f, T, Q, S, h, R, @sigmaPoints, type{i});
end
if run_plot2
    figure('units','normalized','outerposition',[ 0 0 1 1])
    for i = 1:length(type)
        subplot(1,2,1)
        plot(X(1,131:171),X(2,131:171),'LineWidth',3)
        hold all
        grid on
        plot(Y_pos(1,130:170),Y_pos(2,130:170),'ko')
        plot(Output.(type{i}).B.xf(1,130:170), Output.(type{i}).B.xf(2,130:170),'r','LineWidth',3)
        for l = 130:5:170
            xy_xf = sigmaEllipse2D(Output.(type{i}).B.xf(1:2,l), Output.(type{i}).B.Pf(1:2,1:2,l));
            plot(xy_xf(1,:),xy_xf(2,:),'--','color',[0 0.5 0])
        end
        title('Filtered states with disturbed measurement')
        legend({'True states', 'Measurements','Filtered states','3-$\sigma$ level'},'Interpreter','Latex')
        xlabel('Position [x]')
        ylabel('Position [y]')
        subplot(1,2,2)
        plot(X(1,131:171),X(2,131:171),'LineWidth',3)
        hold all
        grid on
        plot(Y_pos(1,130:170),Y_pos(2,130:170),'ko')
        plot(Output.(type{i}).B.xs(1,130:170), Output.(type{i}).B.xs(2,130:170),'r','LineWidth',3)
        for l = 130:5:170
            xy_xs = sigmaEllipse2D(Output.(type{i}).B.xs(1:2,l), Output.(type{i}).B.Ps(1:2,1:2,l));
            plot(xy_xs(1,:),xy_xs(2,:),'--','color',[0 0.5 0])
        end
        title('Smoothed states with disturbed measurement')
        legend({'True states', 'Measurements','Smoothed states','3-$\sigma$ level'},'Interpreter','Latex')
        xlabel('Position [x]')
        ylabel('Position [y]')
        hgexport(gcf, 'task_1B_1.png', hgexport('factorystyle'), 'Format', 'png');
    end
end
