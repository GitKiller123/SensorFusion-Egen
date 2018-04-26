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

N = 100;
Case = {'1' '2'};

for l = 1:2
    R = [Sigma_s1(l)^2 0;
        0 Sigma_s2^2];
    task1.(['X_' Case{l}]) = genNonLinearStateSequence(x_0, P_0, f, Q, N);
    task1.(['Y_' Case{l}]) = genNonLinearMeasurementSequence(task1.(['X_' Case{l}]), h, R);
    task1.(['Ypos_' Case{l}]) = ang2pos(task1.(['Y_' Case{l}]), s1, s2);
    for i =1:3
        [task1.([type{i} '_' Case{l}]).xf,task1.([type{i} '_' Case{l}]).Pf,task1.([type{i} '_' Case{l}]).xp,task1.([type{i} '_' Case{l}]).Pp] = nonLinearKalmanFilter(task1.(['Y_' Case{l}]), x_0, P_0 , f, Q, h, R, type{i});
        task1.([type{i} '_' Case{l}]).xfpos = ang2pos(task1.([type{i} '_' Case{l}]).xf, s1, s2);
    end
end
%         for i = 1:3
%             figure(i+(l-1)*3)
%             plot(task1.(['X_' Case{l}])(1,:),task1.(['X_' Case{l}])(2,:))
%             header = ['Case ' Case{l} ' ' type{i}];
%             title(header)
%             hold all
%             plot(task1.(['Ypos_' Case{l}])(1,:),task1.(['Ypos_' Case{l}])(2,:))
%             plot(task1.([type{i} '_' Case{l}]).xf(1,:),task1.([type{i} '_' Case{l}]).xf(2,:),'ko')
% 
%             legend('True states', 'Measurements', 'Estimates')
%         end
for i = 1:3
    task1.([type{i} '_Error']).Error = [];
end

R = [Sigma_s1(1)^2 0;
        0 Sigma_s2^2];
for k = 1:100
    task1.X_Error = genNonLinearStateSequence(x_0, P_0, f, Q, N);
    task1.Y_Error = genNonLinearMeasurementSequence(task1.X_Error, h, R);
    task1.Ypos_Error = ang2pos(task1.Y_Error, s1, s2);
    for i =1:3
        [task1.([type{i} '_Error']).xf,task1.([type{i} '_Error']).Pf,task1.([type{i} '_Error']).xp,task1.([type{i} '_Error']).Pp] = nonLinearKalmanFilter(task1.(['Y_Error']), x_0, P_0 , f, Q, h, R, type{i});
        task1.([type{i} '_Error']).xfpos = ang2pos(task1.([type{i} '_Error']).xf, s1, s2);
        task1.([type{i} '_Error']).Error = [task1.([type{i} '_Error']).Error; task1.([type{i} '_Error']).xfpos - task1.X_Error(1:2,2:end)];
    end
end
%     
% for l = 1:2
%     for i = 1:3
%         figure(i+(l-1)*3)
%         plot(task1.(['X_' Case{l}])(1,:),task1.(['X_' Case{l}])(2,:))
%         header = ['Case ' Case{l} ' ' type{i}];
%         title(header)
%         hold all
%         plot(task1.(['Ypos_' Case{l}])(1,:),task1.(['Ypos_' Case{l}])(2,:))
%         plot(task1.([type{i} '_' Case{l}]).xf(1,:),task1.([type{i} '_' Case{l}]).xf(2,:),'ko')
% 
%         legend('True states', 'Measurements', 'Estimates')
%     end
% end
