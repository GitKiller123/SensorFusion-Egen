function Y = genNonLinearMeasurementSequence_HA_4(X, h, R, s)
%GENNONLINEARMEASUREMENTSEQUENCE generates observations of the states 
% sequence X using a non-linear measurement model.
%
%Input:
%   X           [n x N+1] State vector sequence
%   h           Measurement model function handle
%               [hx,Hx]=h(x) 
%               Takes as input x (state) 
%               Returns hx and Hx, measurement model and Jacobian evaluated at x
%   R           [m x m] Measurement noise covariance
%
%Output:
%   Y           [m x N] Measurement sequence
%

n = size(X,2);
Y = [];
zero_vec = zeros(size(R,1), 1);
for k = 2:n
    Y = [Y h(X(:,k),s)+mvnrnd(zero_vec, R)'];
end

end