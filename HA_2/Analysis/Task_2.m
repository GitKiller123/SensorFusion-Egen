clear all; close all;
Task = 'b';

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
        plot(t,X(1,2:end)+3*sqrt(P(:)'),'b--')
        plot(t,X(1,2:end)-3*sqrt(P(:)'),'b--')
        legend('True position', 'Measurement', 'Filtered Measurement','3-Sigma level')
        xlabel('Time [s]')
        ylabel('Position')
        figure(2)
        plot(t,X(2,2:end))
        hold all
        plot(t,Xhat(2,:))
        legend('True velocity', 'Estimated velocity')
        xlabel('Time [s]')
        ylabel('Velocity')
    case {'c'}
        q = [0.1 1 10 1.5];
        for i = 1:length(q)
            clear [Xhat,P,v,Q]
            Q = [0 0;
                 0 q(i)];
            [Xhat, P, v] = kalmanFilter(Y, x_0, P_0, A, Q, H, R);
            figure(i)
            plot(t,X(1,2:end))
            hold all
            plot(t,Y,'y*')
            plot(t,Xhat(1,:))
            plot(t,X(1,2:end)+3*sqrt(R),'b--')
            plot(t,X(1,2:end)-3*sqrt(R),'b--')
            current = sprintf('Q = %0.1f', q(i)); 
            title(current)
            legend('True position', 'Measurement', 'Filtered Measurement','3-Sigma level')
            xlabel('Time [s]')
            ylabel('Position')
            figure(5)
            subplot(4,1,i)
            plot(t,X(2,2:end))
            hold all
            plot(t,Xhat(2,:))
            legend('True velocity', 'Estimated velocity')
            xlabel('Time [s]')
            ylabel('Velocity')
        end
end