function [ratio] = eerf1(StateTrue,StateEstimate,StateNoisy)

errz1 = StateTrue-StateEstimate;
errz2 = StateTrue-StateNoisy;

ratio = aee(errz1)./aee(errz2);