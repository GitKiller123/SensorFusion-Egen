%%	Resampling
tol = 0.01;
% Define p(x) normal distribution x ~ N(p_mean, P)
sigma = 10;
p_mean = 100*rand(2,1); % Generate random mean for p(x)
P = [sigma^2 0;
     0 sigma^2];
 
% Generate samples from a uniform proposal distribution centered in 
% mean - 1.5, with sides 4*sigma.

N = 20000;
X = rand(2,N) * sigma * 8 + p_mean - sigma * 4 - 1.5; % From a square with sides sigma*8

% Importance sample p(x): w(i)~ = p(x(i))/q(x(i)). q(x(i)) is
% 1/(16*sigma^2) for all samples (constant) so unnormalized weights are 
% simply: w(i)~ = p(x(i))
W = exp(sum(-0.5 * ((X-p_mean)'/P)'.*(X-p_mean),1)); % Unnormalized weights
Wn = W/sum(W,2); % Normalize

% resample
[Xr, Wr, j] = resampl(X, Wn);

% check that dimensions are correct
assert((size(Xr,1) == size(X,1)) && (size(Xr,2) == size(X,2)), ...
                                    'Xr output dimension differs from dimension of X');
assert((size(Wr,1) == size(W,1)) && (size(Wr,2) == size(W,2)), ...
                                    'Wr output dimension differs from dimension of W');
assert(length(j) == length(Wr), 'Wr output dimension differs from dimension of W');
% check that weights are equal and normalized
assert((Wr(10) == Wr(20)), 'Output weights are not equal');
assert(norm(sum(Wr) - 1) < tol, 'weights are not normalized');
% check that mean is within tolerance
meanAppr = Xr * Wr';
assert(norm(meanAppr-p_mean) < 0.5, ...
        'resampled particles or weights do not approximate original density well enough');