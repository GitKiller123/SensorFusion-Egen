close all; clear all;
h = [1 1; 1 -1];
h_func = @(x) [x(1,:)+x(2,:);...
                x(1,:)-x(2,:)];
N = 5000;
b = [0;0];
mu_x = [10;0];
Sigma_x = [0.2 0;0 8];
[mu_y_aff, Sigma_y_aff] = affineGaussianTransform(mu_x, Sigma_x, h, b);
[mu_y_app, Sigma_y_app, y_s] = approxGaussianTransform(mu_x, Sigma_x, h_func, N);
[ xy_aff ] = sigmaEllipse2D( mu_y_aff, Sigma_y_aff);
[ xy_app ] = sigmaEllipse2D( mu_y_app, Sigma_y_app);
plot(xy_aff(1,:),xy_aff(2,:))
hold all
plot(xy_app(1,:),xy_app(2,:))
plot(y_s(1,:),y_s(2,:),'x')
plot(mu_y_aff(1,:),mu_y_aff(2,:),'o','linewidth',5)
plot(mu_y_app(1,:),mu_y_app(2,:),'o','linewidth',5)
legend('Affine','Approximation','Approximation Samples','mean','Sample mean')