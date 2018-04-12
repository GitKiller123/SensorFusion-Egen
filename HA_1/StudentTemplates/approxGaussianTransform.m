function [mu_y, Sigma_y, y_s, x] = approxGaussianTransform(mu_x, Sigma_x, f, N)
%approxGaussianTransform takes a Gaussian density and a transformation 
%function and calculates the mean and covariance of the transformed density.
%
%Inputs
%   MU_X        [m x 1] Expected value of x.
%   SIGMA_X     [m x m] Covariance of x.
%   F           [Function handle] Function which maps a [m x 1] dimensional
%               vector into another vector of size [n x 1].
%   N           Number of samples to draw.
%
%Output
%   MU_Y        [n x 1] Approximated mean of y.
%   SIGMA_Y     [n x n] Approximated covariance of y.
%   ys          [n x N] Samples propagated through f. Default = 5000.


if nargin < 4
    N = 5000;
end

x = mvnrnd(mu_x,Sigma_x,N)';
y_s = f(x);
mu_y = mean(y_s')';
Sigma_y = (y_s - mu_y)*(y_s - mu_y)'/(N-1);
%Sigma_y2 = cov(y_s');

end

