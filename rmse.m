function [err] = rmse(errx)

NumStep = size(errx,2);
NumMC = size(errx,3);
errsum = zeros(1,NumStep);
for jj = 1:1:NumStep
    for kk = 1:1:NumMC
        errsum(jj) = errsum(jj)+errx(:,jj,kk).'*errx(:,jj,kk);
    end
end
err = sqrt(errsum./NumMC);