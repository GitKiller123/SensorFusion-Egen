function [Y_pos] = converter(Y, sk)
N = size(Y,2);
for i = 1:N
   Y_pos(:,i) = [Y(1,i)*cos(Y(2,i))+sk(1);
                Y(1,i)*sin(Y(2,i))+sk(2)];
end