% Test kalmanFilter

tol = 1e-5;

%% scalar model

N = 10;
n = 1;
T = 1;

% Motion Parameters
A = 1;
Q = T*.5^2;

% Measurement Parameters
H = 1;
R = 1^2;

% Prior
xPrior  = 0;
PPrior  = 2^2;

% genereate measurement sequence
measuremetnSequence = 10*ones(1,N);

% Filter
[stateSequence, covarianceSequence] = kalmanFilter(measuremetnSequence, xPrior, PPrior, A, Q, H, R);
%[stateSequence_ref, covarianceSequence_ref] = reference.kalmanFilter(measuremetnSequence, xPrior, PPrior, A, Q, H, R);
[stateSequence_ref, covarianceSequence_ref] = kalmanFilter(measuremetnSequence, xPrior, PPrior, A, Q, H, R);

% plot results
figure(1); clf; hold on;
plot([1:N], measuremetnSequence, '*r');

plot([0:N], [xPrior stateSequence], 'b');
plot([0:N], [xPrior stateSequence] + 3*sqrt([PPrior covarianceSequence(:)']), '--b');
plot([0:N], [xPrior stateSequence] - 3*sqrt([PPrior covarianceSequence(:)']), '--b');

title('Your solution')
xlabel('k');
ylabel('x');
legend('measurements', 'state estimate', '+3-sigma level', '-3-sigma level','Location','southeast');

figure(2); clf; hold on;
plot(measuremetnSequence, '*r');

plot([0:N], [xPrior stateSequence_ref], 'b');
plot([0:N], [xPrior stateSequence_ref] + 3*sqrt([PPrior covarianceSequence_ref(:)']), '--b');
plot([0:N], [xPrior stateSequence_ref] - 3*sqrt([PPrior covarianceSequence_ref(:)']), '--b');

title('Reference solution');
xlabel('k');
ylabel('x');
legend('measurements', 'state estimate', '+3-sigma level', '-3-sigma level','Location','southeast');

assert(isequal(size(stateSequence),[n, N]), 'Dimension of estimated state sequence should be [n x N].');
assert(isequal(size(covarianceSequence),[n,n,N]), 'Dimension of sequence estimate error covariance should be [n x n x N].');
assert(all(abs(stateSequence-stateSequence_ref)<tol), 'Estimated state sequence is not within tolerance.');
assert(all(all(abs(covarianceSequence-covarianceSequence_ref)<tol)), 'Estimated error covariance is not within tolerance.');