function [SP,W] = sigmaPoints(x, P, type)
% SIGMAPOINTS computes sigma points, either using unscented transform or
% using cubature.
%
%Input:
%   x           [n x 1] Prior mean
%   P           [n x n] Prior covariance
%
%Output:
%   SP          [n x 2n+1] matrix with sigma points
%   W           [1 x 2n+1] vector with sigma point weights 

Psq = sqrtm(P);

    switch type        
        case 'UKF'
            SP = x;
            W = 1-length(x)/3;
            for i = 1:length(x)
                SP = [SP x+sqrt(3)*Psq(:,i)];
                SP = [SP x-sqrt(3)*Psq(:,i)];
                W = [W 1/6];
                W = [W 1/6];
            end
        case 'CKF'
            SP = [];
            W = [];
            for i = 1:length(x)
                SP = [SP x+sqrt(length(x))*Psq(:,i)];
                SP = [SP x-sqrt(length(x))*Psq(:,i)];
                W = [W 1/(2*length(x))];
                W = [W 1/(2*length(x))];
            end
        otherwise
            error('Incorrect type of sigma point')
    end

end