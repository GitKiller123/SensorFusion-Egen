% Test linearUpdate

tol = 1e-5;

%% scalar model

n = 1;

% Measurement Parameters
H = 1;
R = .5^2;

% Prior
xPrior  = mvnrnd(zeros(n,1)', diag(ones(n)))';
V       = rand(n,n);
PPrior  = V*2*diag(rand(n,1))*V';

% Genereate measurement
y = mvnrnd(H*xPrior, H*PPrior*H' + R);

% Perform update
[xUpd, PUpd] = linearUpdate(xPrior, PPrior, y, H, R);
%[xUpd_ref, PUpd_ref] = reference.linearUpdate(xPrior, PPrior, y, H, R);
[xUpd_ref, PUpd_ref] = linearUpdate(xPrior, PPrior, y, H, R);

% Plot results
figure(1); clf; hold on;
x = linspace(xUpd - 4*sqrt(PPrior), xUpd + 4*sqrt(PPrior),100);
plot(x, normpdf(x,xUpd, sqrt(PUpd)));
plot(x, normpdf(x,xPrior, sqrt(PPrior)));
plot(y,0,'sk', 'LineWidth', 2, 'MarkerSize', 10);
title('Your solution')
xlabel('x');
ylabel('p(x)')
legend('Updated density', 'Prior density', 'Measurement');

figure(2); clf; hold on;
x = linspace(xUpd_ref - 4*sqrt(PPrior), xUpd_ref + 4*sqrt(PPrior),100);
plot(x, normpdf(x, xUpd_ref, sqrt(PUpd_ref)));
plot(x, normpdf(x, xPrior, sqrt(PPrior)));
plot(y,0,'sk', 'LineWidth', 2, 'MarkerSize', 10);
title('Reference solution')
xlabel('x');
ylabel('p(x)')
legend('Updated density', 'Prior density', 'Measurement');


% Assert resutls
assert(isequal(size(xPrior),[n 1]), 'Dimension of prior and predicted mean need to be the same.');
assert(isequal(size(PPrior),[n n]), 'Dimension of prior and predicted covariance need to be the same.');
assert(all(abs(xUpd-xUpd_ref)<tol), 'Updated mean is not within tolerance.');
assert(all(all(abs(PUpd-PUpd_ref)<tol)), 'Updated covarinace is not within tolerance.');

[~, p] = chol(PUpd);
assert(p == 0 || trace((PUpd)) ~= 0, 'Posterior covariance is not positive semi definite covarinace matrix');


%% 2D CV model

tol = 1e-5;

% General parameters
n = 2;

% maesurement Parameters
H = [1 0];
R = .5^2;

% Prior
xPrior  = mvnrnd(zeros(n,1)', diag([1, 1]))';
V       = rand(n,n);
PPrior  = V*diag(rand(n,1))*V';

% Genereate measurement
y = mvnrnd(H*xPrior, H*PPrior*H' + R);

[xUpd, PUpd] = linearUpdate(xPrior, PPrior, y, H, R);
%[xUpd_ref, PUpd_ref] = reference.linearUpdate(xPrior, PPrior, y, H, R);
[xUpd_ref, PUpd_ref] = linearUpdate(xPrior, PPrior, y, H, R);

% Plot results
figure(1); clf; hold on;

th  = linspace(0, 2*pi, 100);
z   = 3*[cos(th); sin(th)];

% Plot prior
xy  = bsxfun(@plus, xPrior, sqrtm(PPrior)*z);

plot(xPrior(1), xPrior(2), '*k');
plot(xy(1,:), xy(2,:), 'k');

% Plot your solution
xy  = bsxfun(@plus, xUpd, sqrtm(PUpd)*z);

plot(xUpd(1), xUpd(2), '*r');
plot(xy(1,:), xy(2,:), 'r');

title('Your solution')
xlabel('x')
ylabel('y')
legend('Prior mean', 'Prior 3-sigma', 'Updated mean', 'Updated 3-sigma')
axis equal

% Plot refrence solution
figure(2); clf; hold on;

% Plot prior
xy  = bsxfun(@plus, xPrior, sqrtm(PPrior)*z);

plot(xPrior(1), xPrior(2), '*k');
plot(xy(1,:), xy(2,:), 'k');

% Plot reference solution
xy  = bsxfun(@plus, xUpd_ref, sqrtm(PUpd_ref)*z);

plot(xUpd_ref(1), xUpd_ref(2), '*r');
plot(xy(1,:), xy(2,:), 'r');

title('Reference solution')
xlabel('x')
ylabel('y')
legend('Prior mean', 'Prior 3-sigma', 'Updated mean', 'Updated 3-sigma')
axis equal

% Assert results
assert(isequal(size(xPrior),[n 1]), 'Dimension of prior and predicted mean need to be the same.');
assert(isequal(size(PPrior),[n n]), 'Dimension of prior and predicted covariance need to be the same.');
assert(all(abs(xUpd-xUpd_ref)<tol), 'Updated mean is not within tolerance.');
assert(all(all(abs(PUpd-PUpd_ref)<tol)), 'Updated covarinace is not within tolerance.');

[~, p] = chol(PUpd);
assert(p == 0 || trace((PUpd)) ~= 0, 'Posterior covariance is not positive semi definite covarinace matrix');


%% random scenario

tol = 1e-5;

% General parameters
n = 5;
m = 3;

% maesurement Parameters
H = randn(m,n);
V = rand(m,m);
R = V*diag(10*rand(m,1))*V';

% Prior
V       = rand(n,n);
PPrior  = V*diag(rand(n,1))*V';
xPrior  = mvnrnd(zeros(n,1)', PPrior)';

% Genereate measurement
y = mvnrnd(H*xPrior, H*PPrior*H' + R)';

[xUpd, PUpd] = linearUpdate(xPrior, PPrior, y, H, R);
%[xUpd_ref, PUpd_ref] = reference.linearUpdate(xPrior, PPrior, y, H, R);
[xUpd_ref, PUpd_ref] = linearUpdate(xPrior, PPrior, y, H, R);

% Assert results
assert(isequal(size(xPrior),[n 1]), 'Dimension of prior and predicted mean need to be the same.');
assert(isequal(size(PPrior),[n n]), 'Dimension of prior and predicted covariance need to be the same.');
assert(all(abs(xUpd-xUpd_ref)<tol), 'Updated mean is not within tolerance.');
assert(all(all(abs(PUpd-PUpd_ref)<tol)), 'Updated covarinace is not within tolerance.');

[~, p] = chol(PUpd);
assert(p == 0 || trace((PUpd)) ~= 0, 'Posterior covariance is not positive semi definite covarinace matrix');

