% Test linearPrediction

tol = 1e-5;

%% scalar model

n = 1;
T = 1;

% Motion Parameters
A = 2;
Q = T*1;

% Prior
xPrior  = mvnrnd(zeros(n,1)', diag(ones(n)))';
V       = rand(n,n);
PPrior  = V*diag(rand(n,1))*V';

[xPred, PPred] = linearPrediction(xPrior, PPrior, A, Q);
%[xPred_ref, PPred_ref] = reference.linearPrediction(xPrior, PPrior, A, Q);
[xPred_ref, PPred_ref] = linearPrediction(xPrior, PPrior, A, Q);

figure(1); clf; hold on;
x = linspace(xPred - 4*sqrt(PPred), xPred + 4*sqrt(PPred),100);
plot(x, normpdf(x,xPred, sqrt(PPred)));
plot(x, normpdf(x,xPrior, sqrt(PPrior)));
title('Your solution')
xlabel('x');
ylabel('p(x)')
legend('Predicted density', 'Prior density');

figure(2); clf; hold on;
x = linspace(xPred_ref - 4*sqrt(PPred_ref), xPred_ref + 4*sqrt(PPred_ref),100);
plot(x, normpdf(x, xPred_ref, sqrt(PPred_ref)));
plot(x, normpdf(x, xPrior, sqrt(PPrior)));
title('Reference solution')
xlabel('x');
ylabel('p(x)')
legend('Predicted density', 'Prior density');


assert(isequal(size(xPrior),size(xPred)), 'Dimenstion of prior and predicted mean need to be the same.');
assert(isequal(size(PPrior),size(PPred)), 'Dimenstion of prior and predicted covariance need to be the same.');
assert(all(abs(xPred-xPred_ref)<tol), 'Predicted mean is not within tolerance.');
assert(all(all(abs(PPred-PPred_ref)<tol)), 'Predicted covarinace is not within tolerance.');

[~, p] = chol(PPred);
assert(p == 0 || trace((PPred)) ~= 0, 'Posterior covariance is not positive semi definite covarinace matrix');


%% 2D CV model

% General parameters
n = 2;
T = 1;

% Motion Parameters
A = [1 T; 0 1];
Q = T*[0 0;0 1];

% Prior
xPrior  = mvnrnd(zeros(n,1)', diag([1, 1]))';
V       = rand(n,n);
PPrior  = V*diag(rand(n,1))*V';

[xPred, PPred] = linearPrediction(xPrior, PPrior, A, Q);
%[xPred_ref, PPred_ref] = reference.linearPrediction(xPrior, PPrior, A, Q);
[xPred_ref, PPred_ref] = linearPrediction(xPrior, PPrior, A, Q);

figure(1); clf; hold on;

th  = linspace(0, 2*pi, 100);
z   = 3*[cos(th); sin(th)];

figure(1);clf;hold on;

% Plot prior
xy  = bsxfun(@plus, xPrior, sqrtm(PPrior)*z);

plot(xPrior(1), xPrior(2), '*k');
plot(xy(1,:), xy(2,:), 'k');

% Plot your solution
xy  = bsxfun(@plus, xPred, sqrtm(PPred)*z);

plot(xPred(1), xPred(2), '*r');
plot(xy(1,:), xy(2,:), 'r');

title('Your solution')
xlabel('x');
ylabel('y');
legend('Prior mean', 'Prior 3-sigma', 'Predicted mean', 'Predicted 3-sigma')

axis equal

% Plot reference solution
figure(2);clf;hold on;
% Plot prior
xy  = bsxfun(@plus, xPrior, sqrtm(PPrior)*z);

plot(xPrior(1), xPrior(2), '*k');
plot(xy(1,:), xy(2,:), 'k');

xy  = bsxfun(@plus, xPred_ref, sqrtm(PPred_ref)*z);

plot(xPred_ref(1), xPred_ref(2), '*r');
plot(xy(1,:), xy(2,:), 'r');

title('Your solution')
xlabel('x');
ylabel('y');
legend('Prior mean', 'Prior 3-sigma', 'Predicted mean', 'Predicted 3-sigma')

axis equal

assert(isequal(size(xPrior),size(xPred)), 'Dimenstion of prior and predicted mean need to be the same.');
assert(isequal(size(PPrior),size(PPred)), 'Dimenstion of prior and predicted covariance need to be the same.');
assert(all(abs(xPred-xPred_ref)<tol), 'Predicted mean is not within tolerance.');
assert(all(all(abs(PPred-PPred_ref)<tol)), 'Predicted covarinace is not within tolerance.');

[~, p] = chol(PPred);
assert(p == 0 || trace((PPred)) ~= 0, 'Posterior covariance is not positive semi definite covarinace matrix');
