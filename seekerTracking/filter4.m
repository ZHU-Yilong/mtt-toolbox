%
% filter4 definition
% MSPDAF filter
%


function filter_handle = filter4(filterParameters)
% obtaining parameters of filter
if ~isempty(filterParameters)
    T = filterParameters{1}.T;
    SigmaRan1 = filterParameters{1}.SigmaRan;
    SigmaAzi1 = filterParameters{1}.SigmaAzi;
    SigmaEle1 = filterParameters{1}.SigmaEle;
%     T2 = filterParameters{2}.T;
    SigmaRan2 = filterParameters{2}.SigmaRan;
    SigmaAzi2 = filterParameters{2}.SigmaAzi;
    SigmaEle2 = filterParameters{2}.SigmaEle;
else    
     T = 0.02;
%      T2 = 0.02;
     SigmaRan1 = 100;
     SigmaRan2 = 100;
     SigmaAzi1 = 0.01;
     SigmaAzi2 = 0.01;
     SigmaEle1 = 0.01;
     SigmaEle2 = 0.01;
end

% motion model definition
alpha = 0.1;
hd_mtnmodel_x = b1model.singer(alpha,T);
hd_mtnmodel_y = b1model.singer(alpha,T); 
hd_mtnmodel_z = b1model.singer(alpha,T);
hd_mtnmodel   = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);

% measurement model definition
StateSym = hd_mtnmodel.StateSym;
R1 = blkdiag(SigmaRan1^2,SigmaAzi1^2,SigmaEle1^2);
hd_msmodel1 = msmodel.drbe(R1, StateSym);

R2 = blkdiag(SigmaRan2^2,SigmaAzi2^2,SigmaEle2^2);
hd_msmodel2 = msmodel.drbe(R2, StateSym);

% filter definition using MSPDAF
lambda = 0.0004;
gamma  = 100;
Pg     = 0.9997;
Pd     = 1;
para   = [lambda,lambda;gamma,gamma;Pg,Pg;Pd,Pd];

hd_filter1 = mttfilter.ukf(hd_mtnmodel, hd_msmodel1);
hd_filter2 = mttfilter.ukf(hd_mtnmodel, hd_msmodel2);
filter_handle = mttfilter.mspdaf({hd_filter1;hd_filter2}, para);

end