function x_corr = ang2pos(Y, s1, s2)
N = size(Y,2);
x_corr = [];
for i = 1:N
    A1 = [-tan(Y(1,i)) 1; 
        -tan(Y(2,i)) 1];
    b1 = [s1(2)-tan(Y(1,i))*s1(1);s2(2)-tan(Y(2,i))*s2(1)];
    x_corr = [x_corr A1\b1];
end