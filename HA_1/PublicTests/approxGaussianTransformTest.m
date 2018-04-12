% test with a 2D Gaussian distribution
mu_x = [rand*2*pi; 5];
Sigma_x = diag([0.2; 0.5]);

% define the transformation as polar to cartesian conversion
f = @f_pol2cart;

% transform Gaussian distribution and calculate mean and covariance by sampling
[mu_y, Sigma_y, xs] = approxGaussianTransform(mu_x, Sigma_x, f);


%% compare resulting mean and cov to reference
[mu_ref, Sigma_ref] = reference.approxGaussianTransform(mu_x, Sigma_x, f);
dm = max(abs(mu_ref - mu_y));
assert(all(abs(mu_ref - mu_y) < 0.1), 'mean is outside of tolerance by %f', dm)
ds = max(abs(Sigma_ref(:) - Sigma_y(:)));
assert(all(abs(Sigma_ref(:) - Sigma_y(:)) < 0.2), 'covariance is outside of tolerance by %f', ds)

%% Check that unbiased estimate of variance is calculated when using few samples
N = 1;
[mu_y, Sigma_y, ys] = approxGaussianTransform(mu_x, Sigma_x, f, N);

dElems = diag(Sigma_y);
assert(all(~(dElems < 1e-10)), 'Covariance should not be zero when using only one sample point')

%% Check that number of returned samples is correct
N = 10+round(rand*1000);
[mu_y, Sigma_y, ys] = approxGaussianTransform(mu_x, Sigma_x, f, N);

assert(size(ys,2) == N, 'Number of samples used and returned should be controlled by the fourth input parameter')
