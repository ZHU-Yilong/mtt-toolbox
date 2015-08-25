function [ratio] = merf2(MeasureTrue,MeasureEstimate,MeasureNoisy)

errz1 = MeasureTrue-MeasureEstimate;
errz2 = MeasureTrue-MeasureNoisy;

ratio = gae(errz1)./gae(errz2);