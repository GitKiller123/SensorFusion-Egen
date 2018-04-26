clear all; close all;

x_0 = [0 0 14 0 0]';
P_0 = diag([10^2 10^2 2^2 (pi/180)^2 (5*pi/180)^2]);

s1 = [-200 100]';
s2 = [-200 -100]';
T = 1;

f = @(x)coordinatedTurnMotion(x, T);
h = @(x)dualBearingMeasurement(x, s1, s2);
type = {'EKF' 'UKF' 'CKF'};
Sigma_v = 1;
Sigma_w = (pi/180);
Sigma_s2 = (0.5*pi/180);
Q = diag([0 0 Sigma_v^2 0 Sigma_w^2]);

N = 100;
Switch_case = '1';

switch Switch_case
    case '1'
        Sigma_s1 = (10*pi/180);
        R = [Sigma_s1^2 0;
            0 Sigma_s2^2];
        Case1.X = genNonLinearStateSequence(x_0, P_0, f, Q, N);
        Case1.Y = genNonLinearMeasurementSequence(Case1.X, h, R);
        Case1.Ypos = ang2pos(Case1.Y, s1, s2);
        for i =1:3
            [Case1.('case'+Switch_case+type{i}).xf,Case1.(type{i}).Pf,Case1.(type{i}).xp,Case1.(type{i}).Pp] = nonLinearKalmanFilter(Case1.Y, x_0, P_0 , f, Q, h, R, type{i});
            Case1.(type{i}).xfpos = ang2pos(Case1.(type{i}).xf, s1, s2);
        end
        plot(Case1.X(1,:),Case1.X(2,:))
        hold all
        for i = 1:3
            figure(i)
            plot(Case1.X(1,:),Case1.X(2,:))
            hold all
            plot(Case1.Ypos(1,:),Case1.Ypos(2,:))
            plot(Case1.(type{i}).xf(1,:),Case1.(type{i}).xf(2,:))
            legend('True states', 'Measurements', 'Estimates')
        end

    case '2'
        Sigma_s1 = (0.5*pi/180);
        R = [Sigma_s1^2 0;
            0 Sigma_s2^2];
        Case2.X = genNonLinearStateSequence(x_0, P_0, f, Q, N);
        Case2.Y = genNonLinearMeasurementSequence(Case2.X, h, R);
        Case2.Ypos = ang2pos(Case2.Y, s1, s2);
        for i =1:3
            [Case2.(type{i}).xf,Case2.(type{i}).Pf,Case2.(type{i}).xp,Case2.(type{i}).Pp] = nonLinearKalmanFilter(Case2.Y, x_0, P_0 , f, Q, h, R, type{i});
            Case2.(type{i}).xfpos = ang2pos(Case2.(type{i}).xf, s1, s2);
        end
        
        for i = 1:3
            figure(i)
            plot(Case2.X(1,:),Case2.X(2,:))
            hold all
            plot(Case2.Ypos(1,:),Case2.Ypos(2,:))
            plot(Case2.(type{i}).xf(1,:),Case2.(type{i}).xf(2,:))
            legend('True states', 'Measurements', 'Estimates')
        end
end

