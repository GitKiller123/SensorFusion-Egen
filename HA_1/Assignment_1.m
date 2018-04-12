clear all
clc


% mu_x = [0; 1; 1; 2];
% Sigma_x = [1 3 4 5;2 2 3 10;1 2 2 4;4 5 5 4];
% 
% f = @(x)[pol2cart(x(1:2,:)), pol2cart(x(3:4,:))];
mu_x = [0; 1];
Sigma_x = [1 3;3 10];

f = @(x)pol2cart(x(1,:),x(2,:));
[mu_y, Sigma_y, y_s] = approxGaussianTransform(mu_x, Sigma_x, f);
% [mu_ref, Sigma_ref] = reference.approxGaussianTransform(mu_x, Sigma_x, f);