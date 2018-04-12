close all; clear all;


[ xHata ] = gaussMixMMSEEst( 1, 0.5, 0.1, 1, 9, 0.9 );
[ xHatb ] = gaussMixMMSEEst( 5, 2, 0.49, -5, 2, 0.51 );
[ xHatc ] = gaussMixMMSEEst( 1, 2, 0.4, 2, 1, 0.6 );

mu_theta_a = 0.1*1 + 0.9*1;
Sigma_theta_a = 0.1^2*0.5 + 0.9^2*9;

mu_theta_b = 0.49*5 + 0.51*-5;
Sigma_theta_b = 0.49^2*2 + 0.51^2*2;

mu_theta_c = 0.4*1 + 0.6*2;
Sigma_theta_c = 0.4^2*2 + 0.6^2*1;

%[mu_a, Sigma_a] = posteriorGaussian(mu_theta_a, Sigma_theta_a, xHata, Sigma_r_Hafjell);
%[mu_b, Sigma_b] = posteriorGaussian(mu_x_Kvitfjell, Sigma_x_Kvitfjell, y_Kvitfjell, Sigma_r_Kvitfjell);

x = [-25:0.01:25];
y_a = normpdf(x, mu_theta_a, Sigma_theta_a);
y_b = normpdf(x, mu_theta_b, Sigma_theta_b);
y_c = normpdf(x, mu_theta_c, Sigma_theta_c);

[~, x_posa] = max(y_a); 
[~, x_posb] = max(y_b);
[~, x_posc] = max(y_c);
subplot(3,1,1)
plot(x, y_a);
hold all
plot([xHata xHata],[0 max(y_a)])
plot([x(x_posa) x(x_posa)],[0 max(y_a)],'--')
legend('Task a','xhat', 'xmap')
subplot(3,1,2)
plot(x, y_b);
hold all
plot([xHatb xHatb],[0 max(y_b)])
plot([x(x_posb) x(x_posb)],[0 max(y_b)],'--')
legend('Task b', 'xhat', 'xmap')
subplot(3,1,3)
plot(x, y_c);
hold all
plot([xHatc xHatc],[0 max(y_c)])
plot([x(x_posc) x(x_posc)],[0 max(y_c)],'--')
legend('Task c', 'xhat', 'xmap')


