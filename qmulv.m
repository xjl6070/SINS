function vo = qmulv(q, vi)  % 四元数乘矢量，即三维矢量的四元数坐标变换（2.4-26）
    qi = [0;vi]; 
    qo = qmul(qmul(q,qi),qconj(q)); % qmul 四元数相乘（2.4-6）  qconj 共轭四元数（2.4-13）
    vo = qo(2:4,1);
    % vo = q2mat(q)*vi;
