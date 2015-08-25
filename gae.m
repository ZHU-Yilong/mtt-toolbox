function [err] = gae(errx)

NumStep = size(errx,2);
NumMC = size(errx,3);
errprod = ones(1,NumStep);
for jj = 1:1:NumStep
    for kk = 1:1:NumMC
        errprod(jj) = errprod(jj)*sqrt(errx(:,jj,kk).'*errx(:,jj,kk));
    end
end
err = errprod.^(1./NumMC);