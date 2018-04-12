close all; clear all;
mu_x_Hafjell = 1.1;
Sigma_x_Hafjell = 0.5^2;
Sigma_r_Hafjell = 0.2^2;
mu_x_Kvitfjell = 1;
Sigma_x_Kvitfjell = 0.5^2;
Sigma_r_Kvitfjell = 1^2;
%[mu_y_aff, Sigma_y_aff] = affineGaussianTransform(mu_x, Sigma_x, h, b);

[mu_Hafjell, Sigma_Hafjell] = jointGaussian(mu_x_Hafjell, Sigma_x_Hafjell, Sigma_r_Hafjell);
[mu_Kvitfjell, Sigma_Kvitfjell] = jointGaussian(mu_x_Kvitfjell, Sigma_x_Kvitfjell, Sigma_r_Kvitfjell);

    %[ xy_aff ] = sigmaEllipse2D( mu_y_aff, Sigma_y_aff);
[ xy_Hafjell ] = sigmaEllipse2D( mu_Hafjell, Sigma_Hafjell);
[ xy_Kvitfjell ] = sigmaEllipse2D( mu_Kvitfjell, Sigma_Kvitfjell);
plot(xy_Hafjell(1,:),xy_Hafjell(2,:))
hold all
plot(xy_Kvitfjell(1,:),xy_Kvitfjell(2,:))
plot(mu_Hafjell(1),mu_Hafjell(2), 'o', 'linewidth', 5);
plot(mu_Kvitfjell(1),mu_Kvitfjell(2), 'o', 'linewidth', 5);

axis([-3 5 -3 5])
legend('Variance - Hafjell','Variance - Kvitfjell', 'Mean - Hafjell', 'Mean - Kvitfjell')