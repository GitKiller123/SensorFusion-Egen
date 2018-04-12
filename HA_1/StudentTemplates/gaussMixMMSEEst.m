function [ xHat ] = gaussMixMMSEEst( mu_1, sigma2_1, w_1, mu_2, sigma2_2, w_2 )
%GAUSSMIXMMSEEST calculates the MMSE estimate from a Gaussian mixture
%density with two components.
%
%Input
%   MU_1        Mean of first density component
%   SIGMA2_1    Covariance of first density component
%   W_1         Weight of first density component
%   MU_2        Mean of second density component
%   SIGMA2_2    Covariance of second density component
%   W_2         Weight of second density component
%
%Output
%   X_MMSE      MMSE estimate

xHat = (w_1*mu_1 + w_2*mu_2)/(w_1 + w_2);

end

