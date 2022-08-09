function [qnb, vn, pos, eth] = INS_Update(qnb, vn, pos, wnnbk, vnnbk, ts)  % �����ߵ���ֵ�����㷨
% input����Ԫ������ʱ���ٶȡ���ʱ��λ�á����ٶ��������ٶ�����
[phim, dvbm] = cnscl(wnnbk, vnnbk);      % Բ׶���/�������� phim:��Ч��תʸ��
eth = earth(pos, vn);            % ������ز�������
dvn = (eye(3)-askew(eth.wnin)*ts/2) * qmulv(qnb, dvbm);          % ����ϵ�����ٶ�������4.1.50�� 
dvncor = (-cross((2*eth.wnie + eth.wnen),vn) + eth.gcc) * ts;       % �к����ٶȵ��ٶ�������4.1.27��
vn1 = vn + dvn + dvncor;  % �ٶȸ��£�4.1.26��
vn = (vn+vn1) / 2;  % (4.1.60)
pos = pos + [vn(1)/eth.RMh; vn(2)/eth.clRNh; vn(3)] * ts;  % λ�ø��£����λ��֣�
vn = vn1;
qnb = qmul(rv2q(-eth.wnin * ts), qmul(qnb , rv2q(phim)));  % ��̬����  ��4.1.8��
qnb = qnormlz(qnb);