%
% filter5 definition
% IMM/MSPDAF filter
%


function filter_handle = filter5(filterParameters)
% obtaining parameters of filter1
if ~isempty(filterParameters)
    T1 = filterParameters{1}.T;
    SigmaRan1 = filterParameters{1}.SigmaRan;
    SigmaAzi1 = filterParameters{1}.SigmaAzi;
    SigmaEle1 = filterParameters{1}.SigmaEle;
    T2 = filterParameters{2}.T;
    SigmaRan2 = filterParameters{2}.SigmaRan;
    SigmaAzi2 = filterParameters{2}.SigmaAzi;
    SigmaEle2 = filterParameters{2}.SigmaEle;
else    
     T1 = 0.02;
     T2 = 0.02;
     SigmaRan1 = 100;
     SigmaRan2 = 100;
     SigmaAzi1 = 0.01;
     SigmaAzi2 = 0.01;
     SigmaEle1 = 0.01;
     SigmaEle2 = 0.01;
end

% motion model definition
alpha = 0.1;
hd_mtnmodel_x = b1model.singer(alpha,T1);
hd_mtnmodel_y = b1model.singer(alpha,T1); 
hd_mtnmodel_z = b1model.singer(alpha,T1);
hd_mtnmodel1  = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);

alpha = 0.9;
hd_mtnmodel_x = b1model.singer(alpha,T2);
hd_mtnmodel_y = b1model.singer(alpha,T2);
hd_mtnmodel_z = b1model.singer(alpha,T2);
hd_mtnmodel2  = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);

% measurement model definition
StateSym1 = hd_mtnmodel1.StateSym;
R1 = blkdiag(SigmaRan1^2,SigmaAzi1^2,SigmaEle1^2);
hd_msmodel1 = msmodel.drbe(R1, StateSym1);

StateSym2 = hd_mtnmodel2.StateSym;
R2 = blkdiag(SigmaRan2^2,SigmaAzi2^2,SigmaEle2^2);
hd_msmodel2 = msmodel.drbe(R2, StateSym2);

% filter definition using IMM/MSPDAF
lambda = 0.0004;
gamma  = 100;
Pg     = 0.9997;
Pd     = 1;
para   = [lambda,lambda;gamma,gamma;Pg,Pg;Pd,Pd];

hd_filter11 = mttfilter.ukf(hd_mtnmodel1, hd_msmodel1);
hd_filter12 = mttfilter.ukf(hd_mtnmodel1, hd_msmodel2);
hd_filter21 = mttfilter.ukf(hd_mtnmodel2, hd_msmodel1);
hd_filter22 = mttfilter.ukf(hd_mtnmodel2, hd_msmodel2);
hd_filter1 = mttfilter.mspdaf({hd_filter11;hd_filter12}, para);
hd_filter2 = mttfilter.mspdaf({hd_filter21;hd_filter22}, para);
filter_handle = mttfilter.immmspdaf({hd_filter1, hd_filter2});
end