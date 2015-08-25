%
% filter2 definition
% IMM filter
%


function filter_handle = filter2(filterParameters)
% obtaining parameters of filter1
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

% motion model 1 definition
alpha = 0.1;
hd_mtnmodel_x = b1model.singer(alpha,T);                                      % x轴运动模型
hd_mtnmodel_y = b1model.singer(alpha,T);                                      % y轴运动模型
hd_mtnmodel_z = b1model.singer(alpha,T);
hd_mtnmodel1  = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);

% motion model 2 definition
alpha = 0.9;
hd_mtnmodel_x = b1model.singer(alpha,T);                                      % x轴运动模型
hd_mtnmodel_y = b1model.singer(alpha,T);                                      % y轴运动模型
hd_mtnmodel_z = b1model.singer(alpha,T);
hd_mtnmodel2  = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);

% measurement model definition
StateSym = hd_mtnmodel1.StateSym;
R = blkdiag(SigmaRan^2,SigmaAzi^2,SigmaEle^2);
hd_msmodel = msmodel.drbe(R, StateSym);

% filter definition using IMM
MatrixPi = [0.95,0.05;0.05,0.95];
hd_filter1 = mttfilter.ukf(hd_mtnmodel1, hd_msmodel);
hd_filter2 = mttfilter.ukf(hd_mtnmodel2, hd_msmodel);
filter_handle  = mttfilter.imm({hd_filter1, hd_filter2}, MatrixPi);
end