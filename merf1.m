function [ratio] = merf1(MeasureTrue,MeasureEstimate,MeasureNoisy)

errz1 = MeasureTrue-MeasureEstimate;
errz2 = MeasureTrue-MeasureNoisy;

ratio = aee(errz1)./aee(errz2);