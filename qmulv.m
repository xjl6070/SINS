function vo = qmulv(q, vi)  % ��Ԫ����ʸ��������άʸ������Ԫ������任��2.4-26��
    qi = [0;vi]; 
    qo = qmul(qmul(q,qi),qconj(q)); % qmul ��Ԫ����ˣ�2.4-6��  qconj ������Ԫ����2.4-13��
    vo = qo(2:4,1);
    % vo = q2mat(q)*vi;
