close all; clear all;
mu_x_Hafjell = 1.1;
Sigma_x_Hafjell = 0.5^2;
Sigma_r_Hafjell = 0.2^2;
y_Hafjell = 1;
mu_x_Kvitfjell = 1;
Sigma_x_Kvitfjell = 0.5^2;
Sigma_r_Kvitfjell = 1^2;
y_Kvitfjell = 2;

[mu_Hafjell, Sigma_Hafjell] = posteriorGaussian(mu_x_Hafjell, Sigma_x_Hafjell, y_Hafjell, Sigma_r_Hafjell);
[mu_Kvitfjell, Sigma_Kvitfjell] = posteriorGaussian(mu_x_Kvitfjell, Sigma_x_Kvitfjell, y_Kvitfjell, Sigma_r_Kvitfjell);

x = [0:0.01:3];
Y_Hafjell = normpdf(x, mu_Hafjell, Sigma_Hafjell);
Y_Kvitfjell = normpdf(x, mu_Kvitfjell, Sigma_Kvitfjell);
plot(x, Y_Hafjell);
hold all
plot(x, Y_Kvitfjell);

legend('Distribution - Hafjell','Distribution - Kvitfjell', 'Mean - Hafjell', 'Mean - Kvitfjell')