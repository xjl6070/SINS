clc;
clear INS_Calculate process;
clear
close all;
delete(instrfind)
format long
%%              说明
% 使用SBG IMU 计算艾伦方差
% 日期：2022/07/23
%% Parameters
g = 9.7962;
tolT = 50;
N = tolT * 200;
gvar;
s = Serial_IMU_Data;
pause(1)
[T ,  t , TimeStamp , delta_T] = deal(zeros(N,1));
[gyro , accel] = deal(zeros(3,N));
accel_bias = zeros(N,3);
%% ----------零偏校正-------------------
clear process
s.Read_IMU_Data;
TimeStamp_init = vpa(s.TimeStamp)/1e6;
tic
% Main loop
for k = 2 : N
    process(k,N)
    t(k) = toc;
    T(k) = t(k)-t(k-1);
    e = mod(k,nn);
    %% 捷联惯导解算
    s.Read_IMU_Data;
    TimeStamp(k) = vpa(s.TimeStamp)/1e6 - TimeStamp_init;
    delta_T(k) = TimeStamp(k) - TimeStamp(k-1);
    gyro(:,k) = [s.omega.y , s.omega.x , -s.omega.z]';  %右前上
    accel(:,k) = [s.acc.y , s.acc.x , -s.acc.z]';
    
end
% 六面校准法
accel_bias = acc_calibra6(mean(accel',1));
% 分别拟合
am = accel';
G= g^2 * ones(N,1);
f=@(a,am)(a(1)*am(:,1)+a(2)).^2+(a(3)*am(:,2)+a(4)).^2+(a(5)*am(:,3)+a(6)).^2;
a0=[1 0 1 0 1 0];
a=lsqcurvefit(f,a0,am,G);
% 艾伦方差
[sigma, tau, Err] = allan_variance(gyro(3,:), 0.005);
fclose(s.s);
disp(['t = ',num2str(TimeStamp(end))])
filename = FunctionNowFilename([],'.mat');
save(['data/' filename])
save z2data.mat accel
delete(s)