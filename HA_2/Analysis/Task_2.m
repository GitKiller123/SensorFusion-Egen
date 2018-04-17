clear all; close all;
Task = 'c';

N = 20;
T = 0.01;
t = linspace(0,T*(N-1),N);

A = [1 T;
     0 1];
H = [1 0];

q = 1.5;
Q = [0 0;
    0 q];
R = 2;

x_0 = [1;
       3];
P_0 =[4 0;
      0 4];

X = genLinearStateSequence(x_0, P_0, A, Q, N);
Y = genLinearMeasurementSequence(X, H, R);
[Xhat, P, v] = kalmanFilter(Y, x_0, P_0, A, Q, H, R);
Xhat_old = [];
    
switch Task
    case {'a'}
        subplot(2,1,1)
        plot(t,X(1,2:end))
        hold all
        plot(t,Y,'r*')
        legend('True position', 'Measurement')
        xlabel('Time [s]')
        ylabel('Position')
        subplot(2,1,2)
        plot(t,X(2,2:end))
        legend('True velocity')
        xlabel('Time [s]')
        ylabel('Velocity')
    case {'b'}
        plot(t,X(1,2:end))
        hold all
        plot(t,Y,'r*')
        plot(t,Xhat(1,:),'k')
        for i = 1:N
            P_pos(i) = P(i*4-3);
            P_vel(i) = P(i*4);
        end
        plot(t,Xhat(1,:)+3*sqrt(P_pos),'b--')
        plot(t,Xhat(1,:)-3*sqrt(P_pos),'b--')
        legend('True position', 'Measurement', 'Filtered Measurement','3-Sigma level')
        xlabel('Time [s]')
        ylabel('Position')
        figure(2)
        plot(t,X(2,2:end))
        hold all
        plot(t,Xhat(2,:))
        plot(t,Xhat(2,:)+3*sqrt(P_vel),'b--')
        plot(t,Xhat(2,:)-3*sqrt(P_vel),'b--')
        legend('True velocity', 'Estimated velocity', '3-Sigma level')
        xlabel('Time [s]')
        ylabel('Velocity')
    case {'c'}
        q = [0.1 1 10 1.5];
        for i = 1:length(q)
            clear [Xhat,P,v,Q]
            Xhat_old = [Xhat_old; Xhat];
            Q = [0 0;
                 0 q(i)];
            [Xhat, P, v] = kalmanFilter(Y, x_0, P_0, A, Q, H, R);
            for l = 1:N
                P_pos(l) = P(l*4-3);
                P_vel(l) = P(l*4);
            end
            figure(i)
            plot(t,X(1,2:end))
            hold all
            plot(t,Y,'y*')
            plot(t,Xhat(1,:))
            plot(t,X(1,2:end)+3*sqrt(P_pos),'b--')
            plot(t,X(1,2:end)-3*sqrt(P_pos),'b--')
            current = sprintf('Q = %0.1f', q(i)); 
            title(current)
            legend('True position', 'Measurement', 'Filtered Measurement','3-Sigma level')
            xlabel('Time [s]')
            ylabel('Position')
            figure(5)
            subplot(2,2,i)
            plot(t,X(2,2:end))
            hold all
            plot(t,Xhat(2,:))
            plot(t,X(2,2:end)+3*sqrt(P_vel),'b--')
            plot(t,X(2,2:end)-3*sqrt(P_vel),'b--')
            title(current)
            legend('True velocity', 'Estimated velocity', '3-Sigma level')
            xlabel('Time [s]')
            ylabel('Velocity')
        end
end