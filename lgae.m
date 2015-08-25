function [err] = lgae(errx)

NumStep = size(errx,2);
NumMC = size(errx,3);
errlsum = zeros(1,NumStep);
for jj = 1:1:NumStep
    for kk = 1:1:NumMC
        errlsum(jj) = errlsum(jj)+log10(sqrt(errx(:,jj,kk).'*errx(:,jj,kk)));
    end
end
err = errlsum./2./NumMC;