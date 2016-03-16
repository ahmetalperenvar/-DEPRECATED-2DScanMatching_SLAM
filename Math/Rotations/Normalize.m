function [out] = Normalize(angle)
out = angle+2*pi*floor((pi-angle)/(2*pi));