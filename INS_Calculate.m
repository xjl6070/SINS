function avp = INS_Calculate(gyro,accel,T,t,avp,avp_init,e)
% 地球导航参数计算
global nn 
persistent wbk1 vbk1
if isempty(wbk1)||isempty(vbk1)
    wbk1 = 0; vbk1 = 0;
end
avp(10:15) = [gyro;accel]';
qnb = a2qua(avp(1:3)');  vn = avp(4:6)';  pos = avp(7:9)';
wbk = gyro'*T(nn);                                     % 角度增量
vbk = accel'*T(nn);                                    % 速度增量
phi = sum(T)*qq2phi(a2qua(avp(1:3)),a2qua(avp_init(1:3)))/t; % 实时估计失准角误差
qnb = qdelphi(qnb, phi);        % 失准角误差补偿
if e == 0
    wbk1 = wbk;                                        % 角度增量
    vbk1 = vbk;                                       % 速度增量
    return
else
    wbk2 = wbk;                                        % 角度增量
    vbk2 = vbk;                                       % 速度增量
    wnnbk=[wbk1;wbk2];   vnnbk=[vbk1;vbk2];
    [qnb, vn, pos] = INS_Update(qnb, vn, pos, wnnbk, vnnbk, sum(T));      % 捷联惯导数值更新算法
    avp(1:9) = [q2att(qnb); vn; pos]';
end