function [x, P] = nonLinKFupdate_HA_4(x, P, y, s, h, R, SP_func, type)
%nonLinKFupdate_HA_4(x_temp, P_temp, Y(:,k), S, h, R, SP_func, type);
%NONLINKFUPDATE calculates mean and covariance of predicted state
%   density using a non-linear Gaussian model.
%
%Input:
%   x           [n x 1] Prior mean
%   P           [n x n] Prior covariance
%   y           [m x 1] measurement vector
%   h           Measurement model function handle
%               [hx,Hx]=h(x) 
%               Takes as input x (state), 
%               Returns hx and Hx, measurement model and Jacobian evaluated at x
%               Function must include all model parameters for the particular model, 
%               such as sensor position for some models.
%   R           [m x m] Measurement noise covariance
%   type        String that specifies the type of non-linear filter
%
%Output:
%   x           [n x 1] updated state mean
%   P           [n x n] updated state covariance
%

    switch type
        case 'EKF'
            
            [hx, Hx] = h(x,s);
            S = Hx*P*Hx' + R;
            K = P*Hx'*inv(S);
            
            x = x + K*(y-hx);
            P = P - K*S*K';
            
        case 'UKF'
            
            [SP,W] = SP_func(x, P, type);
            y_hat = [];
            for i = 1:size(SP,2)
                y_hat = [y_hat h(SP(:,i),s)*W(i)];
            end
            y_hat = sum(y_hat,2);
            P_xy = zeros(length(x),length(y_hat));
            S = zeros(length(y_hat));
            for i = 1:size(SP,2)
                P_xy = P_xy + (SP(:,i)-x)*(h(SP(:,i),s)-y_hat)'*W(i);
                S = S + (h(SP(:,i),s)-y_hat)*(h(SP(:,i),s)-y_hat)'*W(i);
            end
            S = S + R;
            
            x = x + P_xy*inv(S)*(y-y_hat);
            P = P - P_xy*inv(S)*P_xy';
            
        case 'CKF'
            
            [SP,W] = SP_func(x, P, type);
            y_hat = [];
            for i = 1:size(SP,2)
                y_hat = [y_hat h(SP(:,i),s)*W(i)];
            end
            y_hat = sum(y_hat,2);
            P_xy = zeros(length(x),length(y_hat));
            S = zeros(length(y_hat));
            for i = 1:size(SP,2)
                P_xy = P_xy + (SP(:,i)-x)*(h(SP(:,i),s)-y_hat)'*W(i);
                S = S + (h(SP(:,i),s)-y_hat)*(h(SP(:,i),s)-y_hat)'*W(i);
            end
            S = S + R;
            
            x = x + P_xy*inv(S)*(y-y_hat);
            P = P - P_xy*inv(S)*P_xy';
            
        otherwise
            error('Incorrect type of non-linear Kalman filter')
    end

end
