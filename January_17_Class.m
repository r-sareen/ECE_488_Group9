clear all
close all
% 
% A = [1 0 0;
%      0 3 -4;
%      0 1 -1];
%  
%  [V, D] = eig(A);
%  
 A = [1.1 0 0;
     0 3 -4.05;
     0 1.004 -1];
 
 [V, D] = eig(A);
 
 A = A^-1;
 A = A^-1;