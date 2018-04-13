clear all; close all;
Task = 'f';

N = 20;
k = 11;

A = 1;
H = 1;

Q = 1.5;
R = 2.5;

x_0 = 2;
P_0 = 6;

X = genLinearStateSequence(x_0, P_0, A, Q, N);
Y = genLinearMeasurementSequence(X, H, R);
[Xhat, P, v] = kalmanFilter(Y, x_0, P_0, A, Q, H, R);

switch Task
    case {'a'}
        plot(X(:,2:end), 'k')
        hold all
        plot(Y, 'r*')
        plot(X(:,2:end)+3*sqrt(R), 'b--')
        plot(X(:,2:end)-3*sqrt(R), 'b--')
        legend('State sequence', 'Measurement', '3sigma')
    case {'b'}
        plot(Xhat, 'b')
        hold all
        plot(X(:,2:end), 'k')
        plot(Y, 'r*')
        plot(Xhat+3*sqrt(R), 'b--')
        plot(Xhat-3*sqrt(R), 'b--')
        legend('Kalman', 'State sequence', 'Measurement', '3sigma')
        z = [min([Xhat(4) Xhat(9) Xhat(15)]) - 6:0.01:max([Xhat(4) Xhat(9) Xhat(15)]) + 6];
        XhatPdf_1 = normpdf(z, Xhat(:,4), P(:,:,4));
        XhatPdf_2 = normpdf(z, Xhat(:,9), P(:,:,9));
        XhatPdf_3 = normpdf(z, Xhat(:,15), P(:,:,15));
        figure(2)
        subplot(3,1,1)
        plot(z, XhatPdf_1)
        hold all
        plot([X(:,5) X(:,5)], [0 0.5],'k')
        legend('Posterior density', 'True state')
        subplot(3,1,2)
        plot(z, XhatPdf_2)
        hold all
        plot([X(:,10) X(:,10)], [0 0.5],'k')
        legend('Posterior density', 'True state')
        subplot(3,1,3)
        plot(z, XhatPdf_3)
        hold all
        plot([X(:,15) X(:,15)], [0 0.5],'k')
        legend('Posterior density', 'True state')
    case {'c'}
        z = [min([Xhat(k-1) Xhat(k)])-8:0.01:max([Xhat(k-1) Xhat(k)])+8];
        xpdf_1 = normpdf(z,Xhat(k-1),P(:,:,k-1));
        [xpred, Ppred] = linearPrediction(Xhat(k-1), P(:,:,k-1), A, Q);
        xpdf_2 = normpdf(z, xpred, Ppred);
        xpdf_3 = normpdf(z, Xhat(k), P(:,:,k));
        plot(z, xpdf_1)
        hold all
        plot(z, xpdf_2,'r--')
        plot(z, xpdf_3,'r')
        plot([Y(k) Y(k)], [0 max([xpdf_1 xpdf_2 xpdf_3])],'k')
        legend('k - 1 Updated', 'k Predicted', 'k Updated', 'y_k')
    case {'d'}
        Xerror = (X(:,2:end) - Xhat);
        z = [-5:0.01:5];
        Xpdf = normpdf(z,0,P(:,:,N));
        plot(z,Xpdf)
        hold all
        histogram(Xerror,'Normalization','pdf')
        legend('Xpdf','hist Error')
        
        figure(2)
        autocorr(v)
    case {'f'}
        [Xhat_Inc, P_Inc] = kalmanFilter(Y, 10, P_0, A, Q, H, R);
        plot(Xhat, 'b')
        hold all
        plot(Xhat_Inc, 'r--')
        plot(X(:,2:end), 'k')
        legend('Kalman Correct', 'Kalman Incorrect', 'State sequence')
end
