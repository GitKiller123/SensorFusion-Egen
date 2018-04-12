% parameters as given in problem
mu_x = 19;
sigma2_x = 5^2;
sigma2_r = 2^2;


% random test case 
mu_x1 = 5+randn*10;
P_x1 = (randn*3)^2;
sigma2_r1 = (randn*2)^2;

% tolerance
tol = 1e-8;

%% parameters from problem
[mu, Sigma] = jointGaussian(mu_x, sigma2_x, sigma2_r);
[mu_ref, Sigma_ref] = reference.jointGaussian(mu_x, sigma2_x, sigma2_r);
assert(all(abs(mu-mu_ref) < tol), 'mean is wrong');
assert(all(abs(Sigma(:)-Sigma_ref(:)) < tol), 'covariance is wrong');

%% random test parameters
[mu, Sigma] = jointGaussian(mu_x1, P_x1, sigma2_r1);
[mu_ref, Sigma_ref] = reference.jointGaussian(mu_x1, P_x1, sigma2_r1);
assert(all(abs(mu-mu_ref) < tol), 'mean is wrong');
assert(all(abs(Sigma(:)-Sigma_ref(:)) < tol), 'covariance is wrong');


