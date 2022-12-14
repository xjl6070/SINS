function eth = earth(pos, vn)  % 地球相关参数计算
global Re ff wie g0   % wie：地球自转角速率
    ee = sqrt(2*ff-ff^2);  e2 = ee^2;  % 第一偏心率
    eth.sl = sin(pos(1));  eth.cl = cos(pos(1));  eth.tl = eth.sl/eth.cl;
    eth.sl2 = eth.sl*eth.sl;  sl4 = eth.sl2*eth.sl2;
    sq = 1-e2*eth.sl2;  sq2 = sqrt(sq);
    eth.RMh = Re*(1-e2)/sq/sq2+pos(3);   % 子午圈主曲率半径+h
    eth.RNh = Re/sq2+pos(3);     % 卯酉圈（东西圈或主垂线）主曲率半径+h
    eth.clRNh = eth.cl*eth.RNh; 
    eth.wnie = wie*[0; eth.cl; eth.sl];           % 地球自转角速率
    eth.vn = vn;
    eth.wnen = [-vn(2)/eth.RMh; vn(1)/eth.RNh; vn(1)/eth.RNh*eth.tl]; % 因地球表面弯曲引起的角速率
    eth.wnin = eth.wnie + eth.wnen;
    eth.wnien = eth.wnie + eth.wnin;  % 2wie+wen
    gLh = g0*(1+5.27094e-3*eth.sl2+2.32718e-5*sl4)-3.086e-6*pos(3); % grs80重力模型
    eth.gn = [0;0;-gLh];
    eth.gcc = eth.gn - cross(eth.wnien,vn); % 考虑重力/哥氏力/向心力

