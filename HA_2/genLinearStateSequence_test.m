% Test genLinearStateSequence

% Common data

% Tolerance
tol = 1e-1;

%% Scalar scenario (random walk)

N = 5000;

% Define prior
x_0     = [0]; 
n       = length(x_0); 
P_0     = diag(ones(n,1));

% Define process model
A       = diag(ones(n,1));
Q       = diag(ones(n,1));

% generate state sequence
s = rng;
X   = genLinearStateSequence(x_0, P_0, A, Q, N);
rng(s);
%X_ref = reference.genLinearStateSequence(x_0, P_0, A, Q, N);
X_ref = genLinearStateSequence(x_0, P_0, A, Q, N);

% Plot results
figure(1);clf;hold on;
subplot(2,1,1);plot(X);
title('Your solution');
xlabel('k');
ylabel('x');
subplot(2,1,2);plot(X_ref);
title('Reference solution');
xlabel('k');
ylabel('x');

% Test results
assert(size(X,1) == n, 'X should have the same number of rows as elements in the state vector');
assert(size(X,2) == N+1, 'X should have N+1 columns');

Qest = cov((X(:,2:end)-A*X(:,1:end-1))');
qMean = mean((X(:,2:end)-A*X(:,1:end-1)),2);

assert(all(abs(qMean-zeros(n,1)) < tol), 'Sample process noise is not zero mean.')
assert(all(abs(Qest-Q) < tol), 'Sample process noise covariance is not within tolerances.')

%% Vector scenario (CV)

N = 5000;

% Define prior
x_0     = [0;0]; 
n       = length(x_0); 
P_0     = diag(ones(n,1));

% Define process model
A       = [1 1; 0 1];
Q       = diag(ones(n,1));

% generate state sequence
s = rng;
X = genLinearStateSequence(x_0, P_0, A, Q, N);
rng(s);
%X_ref = reference.genLinearStateSequence(x_0, P_0, A, Q, N);
X_ref = genLinearStateSequence(x_0, P_0, A, Q, N);

% Plot results
figure(2);clf;hold on;
subplot(2,1,1);plot(X(1,:));
title('Your solution');
xlabel('k');
ylabel('x-position');
subplot(2,1,2);plot(X(2,:));
xlabel('k');
ylabel('speed');

figure(3);clf;hold on;
subplot(2,1,1);plot(X_ref(1,:));
title('Reference solution');
xlabel('k');
ylabel('x-position');
subplot(2,1,2);plot(X_ref(2,:));
xlabel('k');
ylabel('speed');

% Test result
assert(size(X,1) == n, 'X should have the same number of rows as elements in the state vector');
assert(size(X,2) == N+1, 'X should have N+1 columns');

% Whiteness
Qest = cov((X(:,2:end)-A*X(:,1:end-1))');
qMean = mean((X(:,2:end)-A*X(:,1:end-1)),2);
assert(all(abs(qMean-zeros(n,1)) < tol), 'Sample process noise is not zero mean.')
assert(all(all(abs(Qest-Q) < tol)), 'Sample process noise covariance is not within tolerances.')

%% Random scenario

N = 50000;

% Ranom dimension
n   = 5;

% Define prior
x_0     = 10*rand(n,1);

% Generate random positive definite matrix
V       = rand(n,n);
P_0     = V*diag(10*rand(n,1))*V'; 

% Define process model
A       = rand(n,n).*triu(ones(n,n));

% Generate random positive definite matrix
V       = rand(n,n);
Q       = V*diag(10*rand(n,1))*V';

% generate state sequence
X   = genLinearStateSequence(x_0, P_0, A, Q, N);


assert(size(X,1) == n, 'X should have the same number of rows as elements in the state vector');
assert(size(X,2) == N+1, 'X should have N+1 columns');

Qest = cov((X(:,2:end)-A*X(:,1:end-1))');
qMean = mean((X(:,2:end)-A*X(:,1:end-1)),2);

assert(all(abs(qMean-zeros(n,1)) < tol), 'Sample process noise is not zero mean.')
assert(all(all(abs(Qest-Q) < 3*tol)), 'Sample process noise covariance is not within tolerances.');

%% Prior distribution

N = 50000;

% Ranom dimension
n   = 5;

% Define prior
x_0     = 10*rand(n,1);

% Generate random positive definite matrix
V       = rand(n,n);
P_0     = V*diag(10*rand(n,1))*V'; 

% Define process model
A       = rand(n,n).*triu(ones(n,n));

% Generate random positive definite matrix
V       = rand(n,n);
Q       = V*diag(10*rand(n,1))*V';

% generate state sequence
X = zeros(n,N);
for i = 1:N
    X(:,i)   = genLinearStateSequence(x_0, P_0, A, Q, 0);
end


assert(size(X,1) == n, 'X should have the same number of rows as elements in the state vector');

Pest = cov((X-x_0)');
xMean = mean(X,2);

assert(all(abs(xMean-x_0) < tol), 'Prior state realizations do not have the correct mean.')
assert(all(all(abs(Pest-P_0) < 3*tol)), 'Prior state realizations do not have the correct covariance.');
