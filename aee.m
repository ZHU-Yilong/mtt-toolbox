function [err] = aee(errx)

NumStep = size(errx,2);
NumMC = size(errx,3);
errsum = zeros(1,NumStep);
for jj = 1:1:NumStep
    for kk = 1:1:NumMC
        errsum(jj) = errsum(jj)+sqrt(errx(:,jj,kk).'*errx(:,jj,kk));
    end
end
err = errsum./NumMC;