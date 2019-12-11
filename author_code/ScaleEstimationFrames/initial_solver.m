
% linear slove the n and d0 according to the R and t and the Homography
% matrix H
% inputs: R,t, relative camera pose;
%         H, homography matrix
%          
% Outputs: do, the distance from camera centere to ground plane
%          n, the nomarl direction of the ground plane


function [d0, n] = initial_solver(R,t,H)

global param;

h = inv(param.K)*H*param.K;
AA = [-h(1,1) t(1,1) 0 0; 
    -h(1,2) 0 t(1,1) 0;
    -h(1,3) 0 0 t(1,1) ;

    -h(2,1) t(2,1) 0 0;
    -h(2,2) 0 t(2,1) 0; 
    -h(2,3) 0 0 t(2,1) ;

    -h(3,1) t(3,1) 0 0;
    -h(3,2) 0 t(3,1) 0;
    -h(3,3) 0 0 t(3,1) ];
B  = [R(1,1);R(1,2);R(1,3);R(2,1);R(2,2);R(2,3);R(3,1);R(3,2);R(3,3)];
B = -B;
x  =  AA\B;
d0 = 1.0/norm(x(2:4,:));
n  =  x(2:4,1);
n = n/norm(n);