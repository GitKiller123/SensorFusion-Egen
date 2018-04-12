nRand = 5;
t.m1 = [1; 3; 1; 5*randn(nRand,1)];
t.s1 = [2; 1; 2; 2*rand(nRand,1)];
t.w1 = [1; 0.5; 0.4; rand(nRand,1)];
t.m2 = [0; -3; 2; 5+2*randn(nRand,1)];
t.s2 = [1; 1; 1; 2*rand(nRand,1)];
t.w2 = 1-t.w1;

tol = 1e-8;

%%
for k = 1:length(t.m1)
    [x_est] = gaussMixMMSEEst(t.m1(k), t.s1(k), t.w1(k), t.m2(k), t.s2(k), t.w2(k));
    [x_est_true] = reference.gaussMixMMSEEst(t.m1(k), t.s1(k), t.w1(k), t.m2(k), t.s2(k), t.w2(k));
    assert(abs(x_est - x_est_true) < tol, 'incorrect estimate');
end