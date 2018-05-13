function [xs, Ps] = nonLinRTSSupdate(xs_kplus1, ...
                                     Ps_kplus1, ...
                                     xf_k, ... 
                                     Pf_k, ...
                                     xp_kplus1, ...
                                     Pp_kplus1, ...
                                     f, ...
                                     T, ...
                                     SP_func, ...
                                     type)
%NONLINRTSSUPDATE Calculates mean and covariance of smoothed state
% density, using a non-linear Gaussian model.
%
%Input:
%   xs_kplus1   Smooting estimate for state at time k+1
%   Ps_kplus1   Smoothing error covariance for state at time k+1
%   xf_k        Filter estimate for state at time k
%   Pf_k        Filter error covariance for state at time k
%   xp_kplus1   Prediction estimate for state at time k+1
%   Pp_kplus1   Prediction error covariance for state at time k+1
%   f           Motion model function handle
%   T           Sampling time
%   sigmaPoints Handle to function that generates sigma points.
%   type        String that specifies type of non-linear filter/smoother
%
%Output:
%   xs          Smoothed estimate of state at time k
%   Ps          Smoothed error convariance for state at time k

switch type
    case 'EKF'
        [fx, Fx] = f(xf_k, T);
        Gk = Pf_k*Fx'*inv(Pp_kplus1);
        xs = xf_k + Gk*(xs_kplus1-fx);
        Ps = Pf_k - Gk*(Pp_kplus1-Ps_kplus1)*Gk';
    case 'UKF'
        [SP, W] = SP_func(xf_k, Pf_k, type);
        Pk_kplus1_k = zeros(length(SP(:,1)));
        for i = 1:length(W)
            Pk_kplus1_k = (SP(:,i)-xf_k)*(f(SP(:,i), T)-xp_kplus1)'*W(i) + Pk_kplus1_k;
        end
        Gk = Pk_kplus1_k*inv(Pp_kplus1);
        xs = xf_k + Gk*(xs_kplus1-xp_kplus1);
        Ps = Pf_k - Gk*(Pp_kplus1 - Ps_kplus1)*Gk';
    case 'CKF'
        [SP, W] = SP_func(xf_k, Pf_k, type);
        Pk_kplus1_k = zeros(length(SP(:,1)));
        for i = 1:length(W)
            Pk_kplus1_k = (SP(:,i)-xf_k)*(f(SP(:,i), T)-xp_kplus1)'*W(i) + Pk_kplus1_k;
        end
        Gk = Pk_kplus1_k*inv(Pp_kplus1);
        xs = xf_k + Gk*(xs_kplus1-xp_kplus1);
        Ps = Pf_k - Gk*(Pp_kplus1 - Ps_kplus1)*Gk';
    otherwise
        printf('incorrect yo');
end


end