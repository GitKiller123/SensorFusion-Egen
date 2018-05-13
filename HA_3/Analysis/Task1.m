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
E_Y1 = mean(Y1')';
E_Y2 = mean(Y2')';
Y1_COV = cov(Y1');
Y2_COV = cov(Y2');
E_Y1_corr = mean(Y1_corr')';
E_Y2_corr = mean(Y2_corr')';
Y1_COV_corr = cov(Y1_corr');
Y2_COV_corr = cov(Y2_corr');

Run_plot1 = true;
Run_plot2 = true;

%Task 1b
for i = 1:3
    [task1.(type{i}).x1, task1.(type{i}).P1] = nonLinKFprediction(x_01, P_01, f, Q, type{i});
    [task1.(type{i}).x2, task1.(type{i}).P2] = nonLinKFprediction(x_02, P_02,f, Q, type{i});
    task1.(type{i}).x1corr = ang2pos(task1.(type{i}).x1, s1, s2);
    task1.(type{i}).x2corr = ang2pos(task1.(type{i}).x2, s1, s2);
    if Run_plot1
        xy_x1 = sigmaEllipse2D(task1.(type{i}).x1, task1.(type{i}).P1, 3);
        xy_y1 = sigmaEllipse2D(E_Y1, Y1_COV, 3);
        xy_x2 = sigmaEllipse2D(task1.(type{i}).x2, task1.(type{i}).P2, 3);
        xy_y2 = sigmaEllipse2D(E_Y2, Y2_COV, 3);
        figure('units','normalized','outerposition',[0 0 1 1])
        subplot(1,2,1)
        plot(Y1(1,:),Y1(2,:),'ro')
        hold all
        plot(xy_y1(1,:),xy_y1(2,:),'r--')
        plot(task1.(type{i}).x1(1), task1.(type{i}).x1(2),'kx')
        plot(xy_x1(1,:),xy_x1(2,:),'k--')
        header = 'Samples and densities for ' + string(type{i}) + ' case 1';
        title(header)
        legend('Measurements','3-Sigma Measurement', 'Approx mean', '3-Sigma Approx')
        xlabel('x')
        ylabel('y')
        
        subplot(1,2,2)
                plot(Y2(1,:),Y2(2,:),'ro')
        hold all
        plot(xy_y2(1,:),xy_y2(2,:),'r--')
        plot(task1.(type{i}).x2(1), task1.(type{i}).x2(2),'kx')
        plot(xy_x2(1,:),xy_x2(2,:),'k--')
        header = 'Samples and densities for ' + string(type{i}) + ' case 2';
        title(header)
        legend('Measurements','3-Sigma Measurement', 'Approx mean', '3-Sigma Approx')
        xlabel('x')
        ylabel('y')
        
        hgexport(gcf, ['Task1_fig_B_' type{i} '.eps'], hgexport('factorystyle'), 'Format', 'fig');
    end
end

if Run_plot2
    figure('units','normalized','outerposition',[0 0 1 1])
    subplot(1,2,1)
    
    plot(Ypos1(1,:),X1(2,:),'*')
    hold all
    xy_ycorr1 = sigmaEllipse2D(E_Y1_corr, Y1_COV_corr, 3);
    plot(xy_ycorr1(1,:),xy_ycorr1(2,:),'r--')    
    plot(s1(1),s1(2),'o','color',[0 0.5 0])
    plot(s2(1),s2(2),'o','color',[0 0.5 0])
    title('Samples vs Measurements 3-Sigma Case 1')
    legend('Samples', 'Measurement 3-Sigma', 'Sensors')
    xlabel('x')
    ylabel('y')
    
    subplot(1,2,2)
    plot(X2(1,:),X2(2,:),'*')
    hold all
    xy_ycorr2 = sigmaEllipse2D(E_Y2_corr, Y2_COV_corr, 3);
    plot(xy_ycorr2(1,:),xy_ycorr2(2,:),'r--')
    plot(s1(1),s1(2),'o','color',[0 0.5 0])
    plot(s2(1),s2(2),'o','color',[0 0.5 0])
    title('Samples vs Measurements 3-Sigma Case 2')
    legend('Samples', 'Measurement 3-Sigma', 'Sensors')
    xlabel('x')
    ylabel('y')
    hgexport(gcf, ['Task1_fig_C_1.eps'], hgexport('factorystyle'), 'Format', 'fig');
end
    




