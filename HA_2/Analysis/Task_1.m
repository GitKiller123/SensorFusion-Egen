clear all; close all;
Task = 'b'; %Variable to decide which task to run

N = 20;     %Setting the length of our state sequence
k = 11;

A = 1;      %Our A-Matrix (in this case scalar)
H = 1;      %Our H-Matrix (in this case scalar)

Q = 1.5;    %Our motion model noise
R = 2.5;    %measurement noise

x_0 = 2;    %Mean of x_0
P_0 = 6;    %Variance of x_0

X = genLinearStateSequence(x_0, P_0, A, Q, N);  %Creating our state sequence
Y = genLinearMeasurementSequence(X, H, R);      %Creating our measurement
                                                %sequence
[Xhat, P, v] = kalmanFilter(Y, x_0, P_0, A, Q, H, R); %Filtering our
                                                      %measurements

switch Task 
    case {'a'}
        plot(X(:,2:end), 'b')
        hold all
        plot(Y, 'r*')
        plot(X(:,2:end)+3*sqrt(R), 'b--')
        plot(X(:,2:end)-3*sqrt(R), 'b--')
        legend('State sequence', 'Measurement', '3-Sigma')
        xlabel('Sample Number')
        ylabel('Value')
    case {'b'}
        plot(0:N, X, 'b')
        hold all
        plot(0:N, [x_0 Xhat], 'k')
        plot(Y, 'r*')
        plot(0:N, [x_0+3*sqrt(P_0) Xhat+3*sqrt(P(:)')], 'b--')
        plot(0:N, [x_0-3*sqrt(P_0) Xhat-3*sqrt(P(:)')], 'b--')
        legend('State sequence', 'Kalman', 'Measurement', '3-Sigma')
        xlabel('Sample Number')
        ylabel('Value')
        %Creating z-values to base our plots on. Considering min/max value
        z = [min([Xhat(4) Xhat(9) Xhat(15)]) - 6:0.01:max([Xhat(4) Xhat(9) Xhat(15)]) + 6];
        XhatPdf_1 = normpdf(z, Xhat(:,4), P(:,:,4));
        XhatPdf_2 = normpdf(z, Xhat(:,9), P(:,:,9));
        XhatPdf_3 = normpdf(z, Xhat(:,14), P(:,:,15));
        figure(2)
        subplot(3,1,1)
        plot([X(:,5) X(:,5)], [0 0.5],'b')
        hold all
        plot(z, XhatPdf_1, 'k')
        plot([Y(:,4) Y(:,4)], [0 0.5],'r--')
        legend('True state', 'Posterior density', 'Measurement')
        subplot(3,1,2)
        plot([X(:,10) X(:,10)], [0 0.5],'b')
        hold all
        plot(z, XhatPdf_2, 'k')
        plot([Y(:,9) Y(:,9)], [0 0.5],'r--')
        legend('True state', 'Posterior density', 'Measurement')
        subplot(3,1,3)
        plot([X(:,15) X(:,15)], [0 0.5],'b')
        hold all
        plot(z, XhatPdf_3, 'k')
        plot([Y(:,14) Y(:,14)], [0 0.5],'r--')
        legend('True state', 'Posterior density', 'Measurement')
        figure(3)
        h = -6:0.01:6;
        Xerror = X(:,2:end) - Xhat;
        Xerror_pdf = normpdf(h,0,cov(Xerror));
        Xhat_cov = normpdf(h,0,P(:,:,N));
        plot(h, Xerror_pdf)
        hold all
        plot(h, Xhat_cov)
        legend('COV of X_{error}', 'COV with P_{N|N}')
    case {'c'}
        z = [min([Xhat(k-1) Xhat(k)])-8:0.01:max([Xhat(k-1) Xhat(k)])+8];
        xpdf_1 = normpdf(z,Xhat(k-1),P(:,:,k-1));
        [xpred, Ppred] = linearPrediction(Xhat(k-1), P(:,:,k-1), A, Q);
        xpdf_2 = normpdf(z, xpred, Ppred);
        xpdf_3 = normpdf(z, Xhat(k), P(:,:,k));
        plot(z, xpdf_1)
        hold all
        plot(z, xpdf_2,'k--')
        plot(z, xpdf_3,'k')
        plot([Y(k) Y(k)], [0 max([xpdf_1 xpdf_2 xpdf_3])],'r')
        legend('p(x_{k-1}|y_{1:k-1})', 'p(x_k|y_{1:k-1})', 'p(x_k|y_{1:k})', 'y_k')
        %legend('k - 1 Updated', 'k Predicted', 'k Updated', 'y_k')
    case {'d'}
        Xerror = (X(:,2:end) - Xhat);
        z = [-5:0.01:5];
        Xpdf = normpdf(z,0,P(:,:,N));
        plot(z,Xpdf)
        hold all
        histogram(Xerror,'Normalization','pdf')
        legend('X_{hat}','X_{error}')
        
        figure(2)
        autocorr(v,N-1)
    case {'f'}
        [Xhat_Inc, P_Inc] = kalmanFilter(Y, 10, P_0, A, Q, H, R);
        plot(0:N, [x_0 Xhat], 'b')
        hold all
        plot(0:N, [10 Xhat_Inc], 'r--')
        plot(0:N, X, 'k')
        legend('Kalman Correct', 'Kalman Incorrect', 'State sequence')
        xlabel('Sample Number')
        ylabel('Value')
end
