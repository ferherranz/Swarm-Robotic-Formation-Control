close all;
clear all;
clc;

% ---------------------------------------------------
% data - data.p - T x (nR x 3)  mat  - [x, y]       - true poses of robots
%      - data.u - T x (nR x 2)  mat  - [vx, vy]     - motion control
%      - data.z - T x (nR)      cell - n x [id]     - readings
%      - data.zO- T x (nR)      cell - n x [x, y, siz]
% @ t, use z(t-1) to get u(t) then p(t)
% ---------------------------------------------------

warning off;
Simulator();