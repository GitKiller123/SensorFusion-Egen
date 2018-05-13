clear all; close all;

K = 20;
N = 50;

bResample = true;
sigma = 2;

Q = 1.5;
R = 2.5;
x_0 = 2;
P_0 = 6;
A = 1;
H = 1;
f = @(x)(1*x);
h = @(x)(1*x);

X = genLinearStateSequence(x_0, P_0, A, Q, K);  %Creating our state sequence
Y = genLinearMeasurementSequence(X, H, R);      %Creating our measurement

plottype = 'Pdf';
task = 'c';
run_plot1 = false;

switch task
    case 'a'
        [Xhat, P, v] = kalmanFilter(Y, x_0, P_0, A, Q, H, R);
        plotFunc = @(k, Xk, Wk, bResample)plotPostPdf(k, Xk, Wk, Xhat, P, bResample, sigma);
        [xfp, Pfp, Xp, Wp] = pfFilter(x_0, P_0, Y, f, Q, h, R, N, bResample,plotFunc,plottype);
    
    case 'b'
        N = 5000;
        [Xhat_corr, P_corr, v_corr] = kalmanFilter(Y, x_0, P_0, A, Q, H, R);
        [xfp_t_corr, Pfp_t_corr, ~, ~] = pfFilter(x_0, P_0, Y, f, Q, h, R, N, true);
        [xfp_f_corr, Pfp_f_corr, ~, ~] = pfFilter(x_0, P_0, Y, f, Q, h, R, N, false);
        x_0 = -20;
        [Xhat, P, v] = kalmanFilter(Y, x_0, P_0, A, Q, H, R);
        [xfp_f, Pfp_f, ~, ~] = pfFilter(x_0, P_0, Y, f, Q, h, R, N, false);
        [xfp_t, Pfp_t, ~, ~] = pfFilter(x_0, P_0, Y, f, Q, h, R, N, true);
        if run_plot1
            figure('units','normalized','outerposition',[ 0 0 1 1])
            suptitle(['Particle filter correct vs incorrect prior N = ' num2str(N)])
            subplot(1,2,1)
            plot(X,'LineWidth',2.5)
            hold all
            grid on
            plot([X(1) xfp_t_corr],'k--','LineWidth',2.5)
            plot([x_0 xfp_t],'r--','LineWidth',2.5)
            plot(2:K+1, [xfp_t_corr+3*sqrt(Pfp_t_corr(:)')], '--', 'color',[0 0.5 0],'LineWidth',1.5)
            plot(2:K+1, [xfp_t+3*sqrt(Pfp_t(:)')], '--', 'color',[0.75, 0, 0.75],'LineWidth',1.5)
            plot(2:K+1, [xfp_t_corr-3*sqrt(Pfp_t_corr(:)')], '--', 'color',[0 0.5 0],'LineWidth',1.5)
            
            plot(2:K+1, [xfp_t-3*sqrt(Pfp_t(:)')], '--', 'color',[0.75, 0, 0.75],'LineWidth',1.5)
            title('WITH resample')
            legend({'True states','PF correct','PF incorrect', 'PF correct 3-$\sigma$ level', 'PF incorrect 3-$\sigma$ level'},'Interpreter','Latex','Location','southeast')
            xlabel('Time step [k]')
            ylabel('Value [-]')
            subplot(1,2,2)
            plot(X,'LineWidth',2.5)
            hold all
            grid on
            plot([X(1) xfp_f_corr],'k--','LineWidth',2.5)
            plot([x_0 xfp_f],'r--','LineWidth',2.5)
            plot(2:K+1, [xfp_f_corr+3*sqrt(Pfp_f_corr(:)')], '--', 'color',[0 0.5 0],'LineWidth',1.5)
            plot(2:K+1, [xfp_f+3*sqrt(Pfp_f(:)')], '--', 'color',[0.75, 0, 0.75],'LineWidth',1.5)
            plot(2:K+1, [xfp_f_corr-3*sqrt(Pfp_f_corr(:)')], '--', 'color',[0 0.5 0],'LineWidth',1.5)
            
            plot(2:K+1, [xfp_f-3*sqrt(Pfp_f(:)')], '--', 'color',[0.75, 0, 0.75],'LineWidth',1.5)
            title('WITHOUT resample')
            legend({'True states','PF correct','PF incorrect', 'PF correct 3-$\sigma$ level', 'PF incorrect 3-$\sigma$ level'},'Interpreter','Latex','Location','southeast')
            xlabel('Time step [k]')
            ylabel('Value [-]')
            hgexport(gcf, 'task_2B_1.png', hgexport('factorystyle'), 'Format', 'png');
            figure('units','normalized','outerposition',[ 0 0 1 1])
            suptitle(['Particle filter vs Kalman filter incorrect prior N = ' num2str(N)])
            subplot(1,2,1)
            plot(X,'LineWidth',2.5)
            hold all
            grid on
            plot([x_0 Xhat],'k--','LineWidth',2.5)
            plot([x_0 xfp_t],'r--','LineWidth',2.5)
            plot(2:K+1, [Xhat+3*sqrt(P(:)')], '--', 'color',[0 0.5 0],'LineWidth',1.5)
            plot(2:K+1, [xfp_t+3*sqrt(Pfp_t(:)')], '--', 'color',[0.75, 0, 0.75],'LineWidth',1.5)
            plot(2:K+1, [Xhat-3*sqrt(P(:)')], '--', 'color',[0 0.5 0],'LineWidth',1.5)
            
            plot(2:K+1, [xfp_t-3*sqrt(Pfp_t(:)')], '--', 'color',[0.75, 0, 0.75],'LineWidth',1.5)
            title('WITH resample')
            legend({'True states','Kalman filter','Particle filter', 'KF 3-$\sigma$ level', 'PF 3-$\sigma$ level'},'Interpreter','Latex','Location','southeast')
            xlabel('Time step [k]')
            ylabel('Value [-]')
            subplot(1,2,2)
            plot(X,'LineWidth',2.5)
            hold all
            grid on
            plot([x_0 Xhat],'k--','LineWidth',2.5)
            plot([x_0 xfp_f],'r--','LineWidth',2.5)
%             plot([x_0+3*sqrt(P_0) Xhat+3*sqrt(P(:)')], '--', 'color',[0 0.5 0])
%             plot([x_0-3*sqrt(P_0) Xhat-3*sqrt(P(:)')], '--', 'color',[0 0.5 0])
            plot([x_0+3*sqrt(P_0) xfp_f+3*sqrt(Pfp_f(:)')], '--', 'color',[0.75, 0, 0.75])
            plot([x_0-3*sqrt(P_0) xfp_f-3*sqrt(Pfp_f(:)')], '--', 'color',[0.75, 0, 0.75])
            title('WITHOUT resample')
            legend({'True states','Kalman filter','Particle filter', 'PF 3-$\sigma$ level'},'Interpreter','Latex','Location','southeast')
            xlabel('Time step [k]')
            ylabel('Value [-]')
            hgexport(gcf, 'task_2B_2.png', hgexport('factorystyle'), 'Format', 'png');
        end
        N = 5;
        x_0 = 2;
        [Xhat_corr, P_corr, v_corr] = kalmanFilter(Y, x_0, P_0, A, Q, H, R);
        [xfp_t_corr, Pfp_t_corr, ~, ~] = pfFilter(x_0, P_0, Y, f, Q, h, R, N, true);
        [xfp_f_corr, Pfp_f_corr, ~, ~] = pfFilter(x_0, P_0, Y, f, Q, h, R, N, false);
        x_0 = -20;
        [Xhat, P, v] = kalmanFilter(Y, x_0, P_0, A, Q, H, R);
        [xfp_f, Pfp_f, ~, ~] = pfFilter(x_0, P_0, Y, f, Q, h, R, N, false);
        [xfp_t, Pfp_t, ~, ~] = pfFilter(x_0, P_0, Y, f, Q, h, R, N, true);
        if run_plot1
            figure('units','normalized','outerposition',[ 0 0 1 1])
            suptitle(['Particle filter correct vs incorrect prior N = ' num2str(N)])
            subplot(1,2,1)
            plot(X,'LineWidth',2.5)
            hold all
            grid on
            plot([X(1) xfp_t_corr],'k--','LineWidth',2.5)
            plot([x_0 xfp_t],'r--','LineWidth',2.5)
            plot(2:K+1, [xfp_t_corr+3*sqrt(Pfp_t_corr(:)')], '--', 'color',[0 0.5 0],'LineWidth',1.5)
            plot(2:K+1, [xfp_t+3*sqrt(Pfp_t(:)')], '--', 'color',[0.75, 0, 0.75],'LineWidth',1.5)
            plot(2:K+1, [xfp_t_corr-3*sqrt(Pfp_t_corr(:)')], '--', 'color',[0 0.5 0],'LineWidth',1.5)
            
            plot(2:K+1, [xfp_t-3*sqrt(Pfp_t(:)')], '--', 'color',[0.75, 0, 0.75],'LineWidth',1.5)
            title('WITH resample')
            legend({'True states','PF correct','PF incorrect', 'PF correct 3-$\sigma$ level', 'PF incorrect 3-$\sigma$ level'},'Interpreter','Latex','Location','southeast')
            xlabel('Time step [k]')
            ylabel('Value [-]')
            subplot(1,2,2)
            plot(X,'LineWidth',2.5)
            hold all
            grid on
            plot([X(1) xfp_f_corr],'k--','LineWidth',2.5)
            plot([x_0 xfp_f],'r--','LineWidth',2.5)
            plot(2:K+1, [xfp_f_corr+3*sqrt(Pfp_f_corr(:)')], '--', 'color',[0 0.5 0],'LineWidth',1.5)
            plot(2:K+1, [xfp_f+3*sqrt(Pfp_f(:)')], '--', 'color',[0.75, 0, 0.75],'LineWidth',1.5)
            plot(2:K+1, [xfp_f_corr-3*sqrt(Pfp_f_corr(:)')], '--', 'color',[0 0.5 0],'LineWidth',1.5)
            
            plot(2:K+1, [xfp_f-3*sqrt(Pfp_f(:)')], '--', 'color',[0.75, 0, 0.75],'LineWidth',1.5)
            title('WITHOUT resample')
            legend({'True states','PF correct','PF incorrect', 'PF correct 3-$\sigma$ level', 'PF incorrect 3-$\sigma$ level'},'Interpreter','Latex','Location','southeast')
            xlabel('Time step [k]')
            ylabel('Value [-]')
            hgexport(gcf, 'task_2B_3.png', hgexport('factorystyle'), 'Format', 'png');
            figure('units','normalized','outerposition',[ 0 0 1 1])
            suptitle(['Particle filter vs Kalman filter incorrect prior N = ' num2str(N)])
            subplot(1,2,1)
            plot(X,'LineWidth',2.5)
            hold all
            grid on
            plot([x_0 Xhat],'k--','LineWidth',2.5)
            plot([x_0 xfp_t],'r--','LineWidth',2.5)
            plot(2:K+1, [Xhat+3*sqrt(P(:)')], '--', 'color',[0 0.5 0],'LineWidth',1.5)
            plot(2:K+1, [xfp_t+3*sqrt(Pfp_t(:)')], '--', 'color',[0.75, 0, 0.75],'LineWidth',1.5)
            plot(2:K+1, [Xhat-3*sqrt(P(:)')], '--', 'color',[0 0.5 0],'LineWidth',1.5)
            
            plot(2:K+1, [xfp_t-3*sqrt(Pfp_t(:)')], '--', 'color',[0.75, 0, 0.75],'LineWidth',1.5)
            title('WITH resample')
            legend({'True states','Kalman filter','Particle filter', 'KF 3-$\sigma$ level', 'PF 3-$\sigma$ level'},'Interpreter','Latex','Location','southeast')
            xlabel('Time step [k]')
            ylabel('Value [-]')
            subplot(1,2,2)
            plot(X,'LineWidth',2.5)
            hold all
            grid on
            plot([x_0 Xhat],'k--','LineWidth',2.5)
            plot([x_0 xfp_f],'r--','LineWidth',2.5)
%             plot([x_0+3*sqrt(P_0) Xhat+3*sqrt(P(:)')], '--', 'color',[0 0.5 0])
%             plot([x_0-3*sqrt(P_0) Xhat-3*sqrt(P(:)')], '--', 'color',[0 0.5 0])
            plot([x_0+3*sqrt(P_0) xfp_f+3*sqrt(Pfp_f(:)')], '--', 'color',[0.75, 0, 0.75])
            plot([x_0-3*sqrt(P_0) xfp_f-3*sqrt(Pfp_f(:)')], '--', 'color',[0.75, 0, 0.75])
            title('WITHOUT resample')
            legend({'True states','Kalman filter','Particle filter', 'PF 3-$\sigma$ level'},'Interpreter','Latex','Location','southeast')
            xlabel('Time step [k]')
            ylabel('Value [-]')
            hgexport(gcf, 'task_2B_4.png', hgexport('factorystyle'), 'Format', 'png');
        end
    case 'c'
        N = 50;
        bResample = true;
        plottype = 'Trajs';
        [Xhat, P, v] = kalmanFilter(Y, x_0, P_0, A, Q, H, R);
        plotFunc = @plotPartTrajs;
        figure('units','normalized','outerposition',[ 0 0 1 1])
        plot(X(2:end),'b','LineWidth',2.5)
        hold all
        [xfp, Pfp, Xp, Wp] = pfFilter(x_0, P_0, Y, f, Q, h, R, N, bResample,plotFunc,plottype);
        plot(X(2:end),'b','LineWidth',2.5)
        title('Particle trajectory with resampling')
        legend({'True states'},'Interpreter','Latex')
        xlabel('Time step [k]')
        ylabel('Value [-]')
        hgexport(gcf, 'task_2D.png', hgexport('factorystyle'), 'Format', 'png');
        figure('units','normalized','outerposition',[ 0 0 1 1])
        plot(X(2:end),'b','LineWidth',2.5)
        hold all
        [xfp, Pfp, Xp, Wp] = pfFilter(x_0, P_0, Y, f, Q, h, R, N, false,plotFunc,plottype);
        plot(X(2:end),'b','LineWidth',2.5)
        title('Particle trajectory without resampling')
        legend({'True states'},'Interpreter','Latex')
        xlabel('Time step [k]')
        ylabel('Value [-]')
        hgexport(gcf, 'task_2C.png', hgexport('factorystyle'), 'Format', 'png');
end