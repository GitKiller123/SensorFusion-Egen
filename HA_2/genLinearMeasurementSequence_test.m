
%% 1D scenario

absTol = 1e-1;
relTol = 5e-2;

N = 50000;

n = randi(1,1);
m = randi(n,1);

% Define state sequence
X = zeros(n,N+1);

% Define measurement model
H = 1;
R = .5^2;

% Generate measurements
Y = genLinearMeasurementSequence(X, H, R);

% PLot results
figure(1);clf;hold on;
plot(0:10,X(1,1:11), '--k');
plot(1:10, Y(1,1:10), '*r');
legend('State sequence', 'Measurements')
title('Your solution');
xlabel('k');
ylabel('position');

assert(size(Y,1) == m, 'Y has the wrong measurement dimension');
assert(size(Y,2) == N, 'Y should have N columns');

Rest = cov((Y-H*X(:, 2:N+1))');
assert(all(all((mean(Y-H*X(:, 2:N+1)) < absTol))), 'Measurement noise is not zeros mean');
assert(all(all((abs(Rest-R) < relTol*R))), 'Measurement noise covariance is not within tolerances');

%% 2D scenario

absTol = 1e-1;
relTol = 5e-2;

N = 50000;

% Define prior
x_0     = [0;0]; 
n       = length(x_0); 
P_0     = diag(ones(n,1));

% Define process model
A       = [1 1; 0 1];
Q       = diag(ones(n,1));

% generate state sequence
X = genLinearStateSequence(x_0, P_0, A, Q, N);

% Define measurement model
H = [1 0];
m = size(H,1);
V = 3*rand(m,m);
R = V*diag(10*rand(m,1))*V';

% Generate measurements
Y = genLinearMeasurementSequence(X, H, R);

assert(size(Y,1) == m, 'Y has the wrong measurement dimension');
assert(size(Y,2) == N, 'Y should have N columns');

Rest = cov((Y-H*X(:, 2:N+1))');
assert(all(all((mean(Y-H*X(:, 2:N+1),2) < absTol))), 'Measurement noise is not zeros mean');
assert(all(all((abs(Rest-R) < relTol*R))), 'Measurement noise covariance is not within tolerances');

% Plot results
nPlot = 30;
figure(2);clf;hold on;
subplot(2,1,1);
plot([0:nPlot],X(1,1:nPlot+1));hold on;
plot([1:nPlot],Y(:,1:nPlot),'*r');
title('Your solution');
legend('state sequence', 'measurement sequence')
xlabel('k');
ylabel('x-position');
subplot(2,1,2);plot(X(2,1:nPlot+1));
xlabel('k');
ylabel('speed');



%% random scenario

absTol = 1e-1;
relTol = 5e-2;


N = 10000;

n = randi(5,1);
m = randi(n,1);

N = m*N;

% Define state sequence
X = rand(n,N+1);

% Define measurement model
H = randn(m,n);
V = rand(m,m);
R = V*diag(10*rand(m,1))*V';

% Generate measurements
Y = genLinearMeasurementSequence(X, H, R);

Rest = cov((Y-H*X(:, 2:N+1))');
assert(size(Y,1) == m, 'Y has the wrong measurement dimension');
assert(size(Y,2) == N, 'Y should have N columns');

Rest = cov((Y-H*X(:, 2:N+1))');
assert(all(all((mean(Y-H*X(:, 2:N+1),2) < absTol))), 'Measurement noise is not zeros mean');
assert(all(all((abs(Rest-R) < relTol*R))), 'Measurement noise covariance is not within tolerances');
