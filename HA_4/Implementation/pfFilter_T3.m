function [xfp, Pfp, Xp, Wp] = pfFilter_T3(x_0, P_0, Y, proc_f, proc_Q, meas_h, meas_R, ...
                             N, bResample, plotFunc, plottype, task)
%PFFILTER Filters measurements Y using the SIS or SIR algorithms and a
% state-space model.
%
% Input:
%   x_0         [n x 1] Prior mean
%   P_0         [n x n] Prior covariance
%   Y           [m x K] Measurement sequence to be filtered
%   proc_f      Handle for process function f(x_k-1)
%   proc_Q      [n x n] process noise covariance
%   meas_h      Handle for measurement model function h(x_k)
%   meas_R      [m x m] measurement noise covariance
%   N           Number of particles
%   bResample   boolean false - no resampling, true - resampling
%   plotFunc    Handle for plot function that is called when a filter
%               recursion has finished.
% Output:
%   xfp         [n x K] Posterior means of particle filter
%   Pfp         [n x n x K] Posterior error covariances of particle filter
%   Xp          [n x N x K] Particles for posterior state distribution in times 1:K
%   Wp          [N x K] Non-resampled weights for posterior state x in times 1:K
if strcmp(task,'e')
    SizeXY = [10;8];
    X_kmin1 = [rand(2,N).*SizeXY + [1;1];zeros(2,N)];
else
    X_kmin1 = mvnrnd(x_0,P_0,N)';
end
    
    W_kmin1 = ones(1,N)./N;
for i = 1:size(Y,2)

[X_kmin1, W_kmin1] = pfFilterStep(X_kmin1, W_kmin1, Y(:,i), proc_f, proc_Q, meas_h, meas_R);
W_kmin1 = W_kmin1.*isOnRoad(X_kmin1(1,:),X_kmin1(2,:))';
W_kmin1 = W_kmin1./sum(W_kmin1);
% Xp_unres(:,:,i) = X_kmin1;

if nargin>9
    if i> 2 && strcmp(plottype,'Pdf')
        plotFunc(i,X_kmin1,W_kmin1, bResample);
    end
end
if bResample
    [X_kmin1, W_kmin1, j] = resampl(X_kmin1, W_kmin1);
else
    j = 1:N;
end

[~, index] = mode(j);
xfp_MAP(:,i) = X_kmin1(:,index);
xfp(:,i) = sum(X_kmin1*W_kmin1',2);
Xp(:,:,i) = X_kmin1;
Pfp(:,:,i) = W_kmin1.*(X_kmin1-xfp(:,i))*(X_kmin1-xfp(:,i))';
Wp(:,i) = W_kmin1;
if nargin>9
    if i> 1 && strcmp(plottype,'Trajs')
        plotFunc(i,X_kmin1,Xp(:,:,i-1),W_kmin1,j);
    elseif i> 1 && strcmp(plottype,'Own')
        plotFunc(X_kmin1,xfp(:,i));
    end
end

end


end