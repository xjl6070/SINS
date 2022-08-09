function accel_bias = acc_calibra6(accel)
format long
%  X = [accel_x accel_y accel_z 1];
%  Y = [ax ay az];
%  beta = [c1      c2      c3
%          c4      c5      c6
%          c7      c8      c9
%          offsetx offsety offsetz]
%  X * beta = Y;
%  beta = inv(X'*X)*X'*Y;
g = 9.7962;
X = [accel,1];
Y1 = [-g,0,0]; Y2 = [0,g,0]; Y3 = [0,0,g];
% beta = X'*X\X'*Y3;
beta1 = inv(X'*X)*X'*Y1;
% beta2 = pinv(X'*X)*X'*Y2;
% beta3 = pinv(X'*X)*X'*Y3;

accel_bias = beta1(4,:);

