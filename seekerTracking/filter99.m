%
% filter99 definition
% CMKF filter
%


function filter_handle = filter99(filterParameters)
% obtaining parameters of filter
if ~isempty(filterParameters)
    T = filterParameters.T;
    SigmaRan = filterParameters.SigmaRan;
    SigmaAzi = filterParameters.SigmaAzi;
    SigmaEle = filterParameters.SigmaEle;
else    
     T = 0.02;
     SigmaRan = 100;
     SigmaAzi = 0.01;
     SigmaEle = 0.01;
end

% motion model definition
alpha = 0.1;
hd_mtnmodel_x = b1model.singer(alpha,T);                                      % x轴运动模型
hd_mtnmodel_y = b1model.singer(alpha,T);                                      % y轴运动模型
hd_mtnmodel_z = b1model.singer(alpha,T);
hd_mtnmodel   = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);

% measurement model definition
StateSym = hd_mtnmodel.StateSym;
R = blkdiag(SigmaRan^2,SigmaAzi^2,SigmaEle^2);
hd_msmodel = msmodel.drbe(R, StateSym);

% filter1 definition using CMKF
filter_handle = mttfilter.cmkf(hd_mtnmodel, hd_msmodel);
end