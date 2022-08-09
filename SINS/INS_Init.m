function [avp,att_mark] = INS_Init(s,gyro_bias,acc_k,acc_b)
disp('开始粗对准')
tolT = 20;
N = tolT * 200;
[gyro,accel]=deal(zeros(3,N));
att_mark = zeros(3,tolT);
i = 0;
for k = 1 : N
    process(k,N);
    s.Read_IMU_Data;
    gyro(:,k) = [s.omega.y , s.omega.x , -s.omega.z]' - gyro_bias;
    accel(:,k) = acc_k.*[s.acc.y , s.acc.x , -s.acc.z]' + acc_b;
    if mod(k,200) == 0
        i = i+1;
        accelk = mean(accel(:,1:k),2); gyrok = mean(gyro(:,1:k),2);
        mat = [-cross(accelk,gyrok)' / norm(cross(accelk,gyrok));
            cross(cross(accelk,gyrok),accelk)' / norm(cross(cross(accelk,gyrok),accelk));
            accelk' / norm(accelk)];
        att_mark(:,i) = m2att(mat)';
    end
end
accelk = mean(accel(:,1:k),2); gyrok = mean(gyro(:,1:k),2);
mat = [-cross(accelk,gyrok)' / norm(cross(accelk,gyrok));
    cross(cross(accelk,gyrok),accelk)' / norm(cross(cross(accelk,gyrok),accelk));
    accelk' / norm(accelk)];
att = m2att(mat);
% att(3) = 0;                         % 这里把偏航角设为0，存在问题
pos = [deg2rad(34);deg2rad(108);425]; vn = [0;0;0];
avp = [att;vn;pos;gyrok;accelk]';
disp(['粗对准结束,姿态角分别为：' num2str(rad2deg([att(2),-att(1),att(3)]))])