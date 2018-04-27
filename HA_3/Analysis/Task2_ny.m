clear all; close all;

x_0 = [0 0 14 0 0]';
P_0 = diag([10^2 10^2 2^2 (pi/180)^2 (5*pi/180)^2]);

s1 = [-200 100]';
s2 = [-200 -100]';
T = 1;

f = @(x)coordinatedTurnMotion(x, T);
h = @(x)dualBearingMeasurement(x, s1, s2);
type = {'EKF' 'CKF' 'UKF'};
Sigma_v = 1;
Sigma_w = (pi/180);
Sigma_s1 = [(10*pi/180) (0.5*pi/180)];
Sigma_s2 = (0.5*pi/180);
Q = diag([0 0 Sigma_v^2 0 Sigma_w^2]);

run_plot = false;
error_try = true;
N = 100;
Case = {'1' '2'};

for l = 1:2
    R = [Sigma_s1(l)^2 0;
        0 Sigma_s2^2];
    task2.(['X_' Case{l}]) = genNonLinearStateSequence(x_0, P_0, f, Q, N);
    task2.(['Y_' Case{l}]) = genNonLinearMeasurementSequence(task2.(['X_' Case{l}]), h, R);
    task2.(['Ypos_' Case{l}]) = ang2pos(task2.(['Y_' Case{l}]), s1, s2);
    for i = 1:3
        [task2.([type{i} '_' Case{l}]).xf,task2.([type{i} '_' Case{l}]).Pf,task2.([type{i} '_' Case{l}]).xp,task2.([type{i} '_' Case{l}]).Pp] = nonLinearKalmanFilter(task2.(['Y_' Case{l}]), x_0, P_0 , f, Q, h, R, type{i});
        %task2.([type{i} '_' Case{l}]).xfpos = ang2pos(task2.([type{i} '_' Case{l}]).xf, s1, s2);
    end
end

if run_plot
    for i = 1:3
        for l = 1:2
            figure(i)
            plot(task2.(['X_' Case{l}])(1,:), task2.(['X_' Case{l}])(2,:))
            header = ['Case ' Case{l} ' ' type{i}];
            title(header)
            hold all
            plot(task2.(['Ypos_' Case{l}])(1,:),task2.(['Ypos_' Case{l}])(2,:),'o')
            plot(task2.([type{i} '_' Case{l}]).xf(1,:),task2.([type{i} '_' Case{l}]).xf(2,:),'k--')
            for k = 1:20
                xy = sigmaEllipse2D( task2.([type{i} '_' Case{l}]).xf(1:2,k*5), task2.([type{i} '_' Case{l}]).Pf(1:2,1:2,k*5), 3);
                plot(xy(1,:),xy(2,:),'--','color',[0 0.5 0])
            end
            legend('True states', 'Measurements', 'Estimates','3-Sigma level')
        end
    end
end


if error_try
    for l = 1:2
        for i = 1:3
            task2.([type{i} '_Error_' Case{l}]).Error = [];
        end
    end
    for k = 1:100
        task2.X_Error = genNonLinearStateSequence(x_0, P_0, f, Q, N);
        for l = 1:2
            R = [Sigma_s1(l)^2 0;
                    0 Sigma_s2^2];
            task2.Y_Error = genNonLinearMeasurementSequence(task2.X_Error, h, R);
            for i =1:3
                try
                    [task2.([type{i} '_Error_' Case{l}]).xf,task2.([type{i} '_Error']).Pf,task2.([type{i} '_Error']).xp,task2.([type{i} '_Error']).Pp] = nonLinearKalmanFilter(task2.Y_Error, x_0, P_0 , f, Q, h, R, type{i});
                catch
%                     task2.X_Error = genNonLinearStateSequence(x_0, P_0, f, Q, N);
                    task2.Y_Error = genNonLinearMeasurementSequence(task2.X_Error, h, R);
                    [task2.([type{i} '_Error_' Case{l}]).xf,task2.([type{i} '_Error']).Pf,task2.([type{i} '_Error']).xp,task2.([type{i} '_Error']).Pp] = nonLinearKalmanFilter(task2.Y_Error, x_0, P_0 , f, Q, h, R, type{i});
                end
                task2.([type{i} '_Error_' Case{l}]).Error = [task2.([type{i} '_Error_' Case{l}]).Error; task2.([type{i} '_Error_' Case{l}]).xf(1:2,:) - task2.X_Error(1:2,2:end)];
            end
        end
    end
    for l = 1:2
        task2.([type{1} '_Error_' Case{l}]).Error = mean(task2.([type{1} '_Error_' Case{l}]).Error);
        task2.([type{2} '_Error_' Case{l}]).Error = mean(task2.([type{2} '_Error_' Case{l}]).Error);
        task2.([type{3} '_Error_' Case{l}]).Error = mean(task2.([type{3} '_Error_' Case{l}]).Error);
    end
end

