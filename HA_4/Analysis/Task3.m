clear all; close all;
load Xk.mat
task = 'a';
sigma_r = 0.1;
q = 1;
R = sigma_r^2*eye(2);
Xs(:,1) = [Xk(:,1); 0; 0];
% yk = [0; 0];
for i = 2:size(Xk,2)
   Xs(:,i) = [Xk(:,i); Xk(:,i)-Xk(:,i-1)];
   yk(:,i-1) = Xs(3:4,i)+mvnrnd(zeros(2,1), R)';
end

x_0 = Xs(:,1);
P_0 = zeros(size(Xs(:,1),1));
T = 1;
A = [1 0 T 0;0 1 0 T; 0 0 1 0; 0 0 0 1];
f = @(X)A*X;
Q = diag([0 0 q q]);
Y = yk;
h = @(X)[0 0 1 0;0 0 0 1]*X;
N = 20000;
bResample = true;

plotFunc = @(Xp, xfp)plotMap(Xs, Xp, xfp);
plottype = 'Own';

[xfp, Pfp, Xp, Wp] = pfFilter_T3(x_0, P_0, Y, f, Q, h, R, ...
                             N, bResample, plotFunc, plottype,task);