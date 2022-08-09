function eth = earth(pos, vn)  % ������ز�������
global Re ff wie g0   % wie��������ת������
    ee = sqrt(2*ff-ff^2);  e2 = ee^2;  % ��һƫ����
    eth.sl = sin(pos(1));  eth.cl = cos(pos(1));  eth.tl = eth.sl/eth.cl;
    eth.sl2 = eth.sl*eth.sl;  sl4 = eth.sl2*eth.sl2;
    sq = 1-e2*eth.sl2;  sq2 = sqrt(sq);
    eth.RMh = Re*(1-e2)/sq/sq2+pos(3);   % ����Ȧ�����ʰ뾶+h
    eth.RNh = Re/sq2+pos(3);     % î��Ȧ������Ȧ�������ߣ������ʰ뾶+h
    eth.clRNh = eth.cl*eth.RNh; 
    eth.wnie = wie*[0; eth.cl; eth.sl];           % ������ת������
    eth.vn = vn;
    eth.wnen = [-vn(2)/eth.RMh; vn(1)/eth.RNh; vn(1)/eth.RNh*eth.tl]; % ����������������Ľ�����
    eth.wnin = eth.wnie + eth.wnen;
    eth.wnien = eth.wnie + eth.wnin;  % 2wie+wen
    gLh = g0*(1+5.27094e-3*eth.sl2+2.32718e-5*sl4)-3.086e-6*pos(3); % grs80����ģ��
    eth.gn = [0;0;-gLh];
    eth.gcc = eth.gn - cross(eth.wnien,vn); % ��������/������/������

