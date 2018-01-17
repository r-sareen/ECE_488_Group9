%Rishab Sareen - 20505101
%Conrad Montor - 20460296

clear all
close all

% A = sym([1 0 1 2 3;
%      1 1 0 3 3;
%      2 -1 3 3 6]);
% 
% colspace(A)

%% i) No Solution
y = sym([2; 9]);
A = sym([2 4 6; 3 6 9]);

colspace(A)
colspace(cat(2,y,A))

%% ii) One Solution


%% iii) Infinite Solutions
y = sym([1; 0]);
A = sym([1 2 3; 4 5 6]);

null(y)