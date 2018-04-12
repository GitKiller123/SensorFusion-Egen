% random 1-D Gaussian distribution
mu1 = 5*randn;
P1 = 3*rand;
A1 = rand(2,1);
b1 = rand(2,1);

% random 2-D Gaussian distribution
mu2 = 5*randn(2,1);
s2 = 3*rand(2);
P2 = s2'*s2;
A2 = rand(2,2);
b2 = rand(2,1);

% random 5-D Gaussian distribution
mu5 = 5*randn(5,1);
s5 = 3*rand(5);
P5 = s5'*s5;
A5 = rand(3,5);
b5 = rand(3,1);

%Tolerance
tol = 1e-5;


%% compare resulting mean and cov to reference 1-D
% transform Gaussian distribution and calculate mean and covariance
[x_mean, x_cov] = affineGaussianTransform(mu1, P1, A1, b1);
[ref_mean, ref_cov] = reference.affineGaussianTransform(mu1, P1, A1, b1);
assert(all(abs(ref_mean - x_mean) < tol), 'mean is outside of tolerance by %f', max(abs(ref_mean - x_mean)))
assert(all(abs(ref_cov(:) - x_cov(:)) < tol), 'covariance is outside of tolerance by %f', max(abs(ref_cov(:) - x_cov(:))))

%% compare resulting mean and cov to reference 2-D
% transform Gaussian distribution and calculate mean and covariance
[x_mean, x_cov] = affineGaussianTransform(mu2, P2, A2, b2);
[ref_mean, ref_cov] = reference.affineGaussianTransform(mu2, P2, A2, b2);
assert(all(abs(ref_mean - x_mean) < tol), 'mean is outside of tolerance by %f', max(abs(ref_mean - x_mean)))
assert(all(abs(ref_cov(:) - x_cov(:)) < tol), 'covariance is outside of tolerance by %f', max(abs(ref_cov(:) - x_cov(:))))

%% compare resulting mean and cov to reference 5-D
% transform Gaussian distribution and calculate mean and covariance
[x_mean, x_cov] = affineGaussianTransform(mu5, P5, A5, b5);
[ref_mean, ref_cov] = reference.affineGaussianTransform(mu5, P5, A5, b5);
assert(all(abs(ref_mean - x_mean) < tol), 'mean is outside of tolerance by %f', max(abs(ref_mean - x_mean)))
assert(all(abs(ref_cov(:) - x_cov(:)) < tol), 'covariance is outside of tolerance by %f', max(abs(ref_cov(:) - x_cov(:))))

