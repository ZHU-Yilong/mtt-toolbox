function [ratio] = lmerf(MeasureTrue,MeasureEstimate,MeasureNoisy)

errz1 = MeasureTrue-MeasureEstimate;
errz2 = MeasureTrue-MeasureNoisy;

ratio = lgae(errz1)-lgae(errz2);