% A few random test vectors
n = 5;
mu_p = 5*randn(1,n);
sigma2_p = 3*rand(1,n);
y = mu_p+5*randn(1,n);
sigma2_r = 3*rand(1,n);
tol = 1e-8;

%% Correct result
for k = 1:n
    [mu1,sigma1] = posteriorGaussian(mu_p(k), sigma2_p(k), y(k), sigma2_r(k));
    [mu2,sigma2] = reference.posteriorGaussian(mu_p(k), sigma2_p(k), y(k), sigma2_r(k));
    assert(abs(mu1-mu2) < tol, 'mean is not correct');
    assert(abs(sigma1-sigma2) < tol, 'variance is not correct');
end

