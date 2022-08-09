function [qnb, vn, pos, eth] = INS_Update(qnb, vn, pos, wnnbk, vnnbk, ts)  % 捷联惯导数值更新算法
% input：四元数、上时刻速度、上时刻位置、角速度增量、速度增量
[phim, dvbm] = cnscl(wnnbk, vnnbk);      % 圆锥误差/划船误差补偿 phim:等效旋转矢量
eth = earth(pos, vn);            % 地球相关参数计算
dvn = (eye(3)-askew(eth.wnin)*ts/2) * qmulv(qnb, dvbm);          % 导航系比力速度增量（4.1.50） 
dvncor = (-cross((2*eth.wnie + eth.wnen),vn) + eth.gcc) * ts;       % 有害加速度的速度增量（4.1.27）
vn1 = vn + dvn + dvncor;  % 速度更新（4.1.26）
vn = (vn+vn1) / 2;  % (4.1.60)
pos = pos + [vn(1)/eth.RMh; vn(2)/eth.clRNh; vn(3)] * ts;  % 位置更新（梯形积分）
vn = vn1;
qnb = qmul(rv2q(-eth.wnin * ts), qmul(qnb , rv2q(phim)));  % 姿态更新  （4.1.8）
qnb = qnormlz(qnb);