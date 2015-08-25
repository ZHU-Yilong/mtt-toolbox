function [ratio] = leerf(StateTrue,StateEstimate,StateNoisy)

errz1 = StateTrue-StateEstimate;
errz2 = StateTrue-StateNoisy;

ratio = lgae(errz1)-lgae(errz2);