function [rate] = succrate(MeasureTrue,MeasureEstimate,MeasureNoisy,Alpha)

errz1 = MeasureTrue-MeasureEstimate;
errz2 = MeasureTrue-MeasureNoisy;

NumStep = size(MeasureTrue,2);
NumMC = size(MeasureTrue,3);

ratio = zeros(NumStep,NumMC);
% rate = zeros(1,NumStep);
for jj = 1:1:NumStep
    for kk = 1:1:NumMC    
        ratio(jj,kk) = (errz1(:,jj,kk).'*errz1(:,jj,kk))./(errz2(:,jj,kk).'*errz2(:,jj,kk)) < Alpha;
    end    
end
rate = mean(ratio,2);