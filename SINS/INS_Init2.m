function avp = INS_Init2(s,gyro_bias)
disp('开始粗对准')
global nn
%% 
N = 10000;
flag = 1;
[T,t]=deal(zeros(N,1));
[gyro1,accel1,gyro2,accel2,accel]=deal(zeros(3,N));
[cn0n , cb0b] = deal(zeros(3,3,N));
cn0n(:,:,1) = eye(3);   cb0b(:,:,1) = eye(3);
% init
pos = [deg2rad(34);deg2rad(108);425]; vn = [0;0;0];
att1 = m2att(eye(3));     att2 = m2att(eye(3));
eth = earth(pos,vn);
gyro1(:,1) = qmulv(qconj(a2qua(att1)) , eth.wnie);
s.Read_IMU_Data;
gyro2(:,1) = [s.omega.y , s.omega.x , -s.omega.z]'-gyro_bias;
avp1(1,:) = [att1;vn;pos;gyro1(:,1);accel1(:,1)]';
avp2(1,:) = [att2;vn;pos;gyro2(:,1);accel2(:,1)]';
Fb02 = [0;0;0];
Gn02 = [0;0;0];
tic
for k = 2 : N
    process(k,N)
    e = mod(k,nn);
    t(k) = toc;
    T(k) = t(k)-t(k-1);
    s.Read_IMU_Data;
    gyro1(:,k) = eth.wnie;
    gyro2(:,k) = [s.omega.y , s.omega.x , -s.omega.z]';
    accel(:,k) = [s.acc.y , s.acc.x , -s.acc.z]';
    avp1(k,:) = INS_Calculate(gyro1(:,k),accel(:,k),T(k-nn+1:k),avp1(k-1,:),e);
    avp2(k,:) = INS_Calculate(gyro2(:,k),accel(:,k),T(k-nn+1:k),avp2(k-1,:),e);
    cn0n(:,:,k) = a2mat(avp1(k,1:3));
    cb0b(:,:,k) = a2mat(avp2(k,1:3));
    Fb02 = cb0b(:,:,k)*accel(:,k)*T(k) + Fb02;
    Gn02 = -cn0n(:,:,k)*eth.gcc*T(k) + Gn02;
    if k >= N/2 && flag == 1
        Fb01 = Fb02;    Gn01 = Gn02;
        flag = 0;
    end
end
cn0b0 = [Gn01/norm(Gn01) cross(Gn01,Gn02)/norm(cross(Gn01,Gn02)) cross(cross(Gn01,Gn02),Gn01)/norm(cross(cross(Gn01,Gn02),Gn01))]*...
    [Fb01'/norm(Fb01) ; cross(Fb01,Fb02)'/norm(cross(Fb01,Fb02)) ; cross(cross(Fb01,Fb02),Fb01)'/norm(cross(cross(Fb01,Fb02),Fb01))];
cnb = inv(cn0n(:,:,k))*cn0b0*cb0b(:,:,k);
avp = [m2att(cnb) ; vn ; pos ; gyro2(:,k) ; accel(:,k)]';
% avp(3) = 0;
disp(['粗对准结束,姿态角分别为：' num2str([rad2deg(avp(2)),-rad2deg(avp(1)),rad2deg(avp(3))])])









