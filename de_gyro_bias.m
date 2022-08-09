function gyro_bias = de_gyro_bias(s)
N = 1000;
gyro = [0;0;0];
for k = 1 : N
    process(k,N)
    s.Read_IMU_Data;
    gyro = gyro + [s.omega.y ; s.omega.x ; -s.omega.z];
end
gyro_bias = gyro / N;