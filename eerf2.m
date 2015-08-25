function [ratio] = eerf2(StateTrue,StateEstimate,StateNoisy)

errz1 = StateTrue-StateEstimate;
errz2 = StateTrue-StateNoisy;

ratio = gae(errz1)./gae(errz2);