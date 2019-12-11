function [p1,p2,m,ep2] = FundamentalMatrixToCameraMatrix(f)
% Extracts the cameras from the fundamental matrix.
%
% Input:
%       - f is the 3x4 fundamental matrix.
%
% Output:
%       - p1 = [I | 0] is the first camera matrix.
%       - p2 is the second camera matrix.
%
%----------------------------------------------------------
%
% From 
%    Book: "Multiple View Geometry in Computer Vision",
% Authors: Hartley and Zisserman, 2006, [HaZ2006]
% Section: "Retrieving the camera matrices", 
% Chapter: 9
%    Page: 253
%
%----------------------------------------------------------
%      Author: Diego Cheda
% Affiliation: CVC - UAB
%        Date: 03/06/2008
%----------------------------------------------------------

% P1 = [I | 0]
p1 = eye(3,4);

% By result 9.14, pág. 256
% P2 = [ [e2]_x*F | e2 ]

% Compute the epipole e2 
% l2 = Fx and contains the epipole e2. 
% Thus, e2 satisfies e2' * (Fx) = (e2' * F) x = 0. Then, e2' * F = 0, i.e.
% e2 is the left null-vector of F. Idem to Fe1 = 0, i.e. e1 is the right
% null vector of F.
[ep1,ep2] = Epipoles(f);

m = (SkewSymmetricMatrix(ep2)*f);

p2 = [ m  ep2 ];

