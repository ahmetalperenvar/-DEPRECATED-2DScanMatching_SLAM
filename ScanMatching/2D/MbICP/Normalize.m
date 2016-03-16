function [out] = Normalize(angle) %FOR BACK COMPATIBILITY....% TO REMOVE
out = angle+2*pi*floor((pi-angle)/(2*pi));