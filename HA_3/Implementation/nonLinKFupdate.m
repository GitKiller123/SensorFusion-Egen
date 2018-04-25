function [x, P] = nonLinKFupdate(x, P, y, h, R, type)
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
            
            [hx, Hx] = h(x);
            S = Hx*P*Hx' + R;
            K = P*Hx'*inv(S);
            
            x = x + K*(y-hx);
            
        case 'UKF'
            
            [SP,W] = sigmaPoints(x, P, type);
            y_hat = sum(h(SP)*W',2);
            P_xy = zeros(length(x));
            S = zeros(length(y_hat));
            for i = 1:size(SP,2)-1
                P_xy = P_xy + (SP(:,i)-x)*(h(SP(:,i))-y_hat)'*W(i);
                S = S + (h(SP(:,i))-y_hat)*(h(SP(:,i))-y_hat)'*W(i);
            end
            S = S + R;
            
            x = x + P_xy*inv(S)*(y-y_hat);
            P = P - P_xy*inv(S)*P_xy';
            
        case 'CKF'
            
            [SP,W] = sigmaPoints(x, P, type);
            y_hat = sum(h(SP)*W',2);
            P_xy = zeros(length(x));
            S = zeros(length(y_hat));
            for i = 1:size(SP,2)-1
                P_xy = P_xy + (SP(:,i)-x)*(h(SP(:,i))-y_hat)'*W(i);
                S = S + (h(SP(:,i))-y_hat)*(h(SP(:,i))-y_hat)'*W(i);
            end
            S = S + R;
            
            x = x + P_xy*inv(S)*(y-y_hat);
            P = P - P_xy*inv(S)*P_xy';
            
        otherwise
            error('Incorrect type of non-linear Kalman filter')
    end

end
