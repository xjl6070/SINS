clc
clear all
close all
delete(instrfind)
%% Parameters
tolT = 60;
N=tolT * 200;
gvar;
s = Serial_IMU_Data;
[T , t, TimeStamp , delta_T]=deal(zeros(N,1));
[gyro,accel,accelb]=deal(zeros(3,N));
avp = zeros(N,15);
%% ----------陀螺仪零偏校正-------------------
gyro_bias = de_gyro_bias(s);
clear process
%% -------------初始粗对准-------------------
avp(1,:) = INS_Init(s,gyro_bias);
%% ----------加速度计零偏校正-----------------
s.Read_IMU_Data;
TimeStamp_init = vpa(s.TimeStamp)/1e6;
tic
clear process
% Main loop
for k = 2 : N
    process(k,N)
    t(k)=toc;
    T(k)=t(k)-t(k-1);
    e = mod(k,nn);
    %% 捷联惯导解算
    s.Read_IMU_Data;
    TimeStamp(k) = vpa(s.TimeStamp)/1e6 - TimeStamp_init;
    delta_T(k) = TimeStamp(k) - TimeStamp(k-1);
    gyro(:,k) = [s.omega.y , s.omega.x , -s.omega.z]';  %右前上
    accel(:,k) = [s.acc.y , s.acc.x , -s.acc.z]';
    avp(k,:) = INS_Calculate(gyro(:,k),accel(:,k),delta_T(k-nn+1:k),TimeStamp(k),avp(k-1,:),avp(1,:),e);
    eth = earth(avp(k,7:9),avp(k,4:6));
    accelb(:,k) = qmulv(qconj(a2qua(avp(k,1:3))) , -eth.gn);
end
accel_bias = mean(accel - accelb , 2);
disp(['gyro_bias=' num2str(gyro_bias')])
disp(['accel_bias=' num2str(accel_bias')])
clear s
