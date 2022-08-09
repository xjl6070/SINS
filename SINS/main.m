clc;
clear INS_Calculate process;
clear
close all;
delete(instrfind)
format long
%%              说明
% 使用SBG IMU 测试捷联惯导算法
% 导航坐标系（n系）与初始机体坐标系重合
% 载体坐标系（b系）定义为"前—左—上"（FLU）坐标系
% 初始粗对准两种方案：解析粗对准INS_Init  间接粗对准INS_Init2 (2022/06/16 by Xu Jinli)
% 添加失准角误差实时估计，导航结果与psins工具箱误差相近。（工具箱需使用相同对准角）
% 日期：2022/06/21
%% Parameters
tolT = 100;
N = tolT * 200;
gvar;
pos = [deg2rad(34);deg2rad(108);425]; vn = [0;0;0];
eth = earth(pos, vn);
s = Serial_IMU_Data;
pause(1)
[T ,  t , TimeStamp , delta_T] = deal(zeros(N,1));
[gyro , accel , v1 , pos1] = deal(zeros(3,N));
avp = zeros(N,15);
%% ----------零偏校正-------------------
gyro_bias = 1.0e-03 * [0.534501467645168 0.011672413349152  -0.419274739921093]';
acc_k = [1.000176837779749     1.000125258364939      1.000084220510630  ]';
acc_b = [-0.009958614457753    0.002694164129709      -0.005980779784955 ]';
% gyro_bias = [0 0 0]';
% accel_bias = [0 0 0]';
%% -------------初始粗对准-------------------
[avp(1,:),att_mark] = INS_Init(s,gyro_bias,acc_k,acc_b);
%% --------------捷联更新--------------------
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
    gyro(:,k) = [s.omega.y , s.omega.x , -s.omega.z]' - gyro_bias;  %右前上
    accel(:,k) = acc_k.*[s.acc.y , s.acc.x , -s.acc.z]' + acc_b;
    avp(k,:) = INS_Calculate(gyro(:,k),accel(:,k),delta_T(k-nn+1:k),TimeStamp(k), avp(k-1,:),avp(1,:),e);
    %% 简单积分解算
    v1(:,k) = v1(:,k-1) + (qmulv(a2qua(avp(k,1:3)), accel(:,k)) + eth.gn) * delta_T(k);
    pos1(:,k) = pos1(:,k-1) + v1(:,k-1)*delta_T(k) + 0.5 * (qmulv(a2qua(avp(k,1:3)), accel(:,k)) + eth.gn) * delta_T(k)^2;
end
disp(['t = ',num2str(TimeStamp(end))])
data = [avp(:,10:15).*delta_T TimeStamp];
save data/data.dat -ascii data
filename = FunctionNowFilename([],'.mat');
save(['data/' filename])
plotfile(TimeStamp,avp,att_mark)
delete(s)