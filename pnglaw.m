function [AccCommand] = pnglaw(PosTarget, VelTarget, PosMissile, VelMissile, N)
% target position 
% missile position
% target velocity
% missile velocity
% last azimuth and elevation angle
 
PosRelative = PosTarget-PosMissile;
VelRelative = VelTarget-VelMissile;

Range = sqrt(PosRelative(1)^2+PosRelative(2)^2+PosRelative(3)^2);
AccCommand = N*dot(PosRelative,VelRelative)/(Range^4).*cross(PosRelative,cross(PosRelative,VelRelative));