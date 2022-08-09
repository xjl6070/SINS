function [phim, dvbm] = cnscl(wnnbk, vnnbk)                        % Բ׶���/��������
    cs = [  [2,    0,    0,    0,    0    ]/3
            [9,    27,   0,    0,    0    ]/20
            [54,   92,   214,  0,    0    ]/105
            [250,  525,  650,  1375, 0    ]/504
            [2315, 4558, 7296, 7834, 15797]/4620  ];         % ��������ϵ��   ��2.6.3
    wmm = sum(wnnbk,1);  vmm = sum(vnnbk,1);
    n = size(wnnbk, 1);  % ������
    dphim = zeros(1,3); scullm = zeros(1,3);
    if n > 1
        csw = cs(n-1,1:n-1)*wnnbk(1:n-1,:); csv = cs(n-1,1:n-1)*vnnbk(1:n-1,:);
        dphim = cross(csw,wnnbk(n,:));                          % Բ׶������ ��4.1.10��
        scullm = cross(csw,vnnbk(n,:)) + cross(csv,wnnbk(n,:));    % ���������� (4.1-44)
    end
	phim = (wmm+dphim)';                                     % ��Ч��תʸ����4.1.10��
    dvrot=0.5*cross(wmm,vmm);                                % ��ת������ ��4.1.36��
	dvbm = (vmm+dvrot+scullm)';                              %  (4.1-50)