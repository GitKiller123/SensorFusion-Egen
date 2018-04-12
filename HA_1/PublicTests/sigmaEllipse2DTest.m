% randomize a 2D Gaussian distribution
mu = 5*randn(2,1);
s = 2*rand(2);
P = s'*s;

%% There should be two rows in the output
xy = sigmaEllipse2D(mu, P);
assert(size(xy,1) == 2, 'There should be two rows in the output');

%% Assuming roughly equidistant points on the ellipse, is the mean of the ellipse correct?
xy = sigmaEllipse2D(mu, P);
xy = unique(xy', 'rows')';
assert(all(abs(mean(xy, 2) - mu)./norm(mu) < 1e-1), ...
    'The mean of the ellipse points should roughly coincide with the distribution mean');

%% Are the generated points on an ellipse of the right shape
xy = sigmaEllipse2D(mu, P);
n = size(xy,2);
r2 = zeros(n,1);
for k = 1:n
    r2(k) = (xy(:,k)-mu)'/P*(xy(:,k)-mu);
end
mr2 = mean(r2);
assert(all(abs(r2-mr2) < 1e-5), 'The ellipse has the wrong shape');

%% Are the generated points on an ellipse on the right distance from the center?
xy = sigmaEllipse2D(mu, P);
n = size(xy,2);
r2 = zeros(n,1);
for k = 1:n
    r2(k) = (xy(:,k)-mu)'/P*(xy(:,k)-mu);
end
mr = sqrt(mean(r2));
assert(abs(mr-3) < 1e-3, 'The ellipse has the wrong size. Default value of the third input parameter (level) should be 3.');

%% Number of generated points
level = 1+rand*2;
npoints = 100+round(25*rand);
xy = sigmaEllipse2D(mu, P, level, npoints);
n = size(xy,2);
assert(npoints == n, 'Number of generated points should be controlled by the fourth input argument')

%% Check size and that the level parameter affects the size
mu = 5*randn(2,1);
s = 2*rand(2);
P = s'*s;
level = 1+rand*2;
xy = sigmaEllipse2D(mu, P, level);
n = size(xy,2);
r2 = zeros(n,1);
for k = 1:n
    r2(k) = (xy(:,k)-mu)'/P*(xy(:,k)-mu);
end
mr = sqrt(mean(r2));
assert(abs(mr-level) < 1e-3, 'The ellipse has the wrong size. The third parameter should control the size of the ellipse.');
