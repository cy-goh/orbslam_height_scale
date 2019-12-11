function f = FundamentalMatrix(p1,p2)
%
% Input:
%       p1 and p2 are 2xN (or 3xN in homogeneous coordiantes) of
%       correspondings corners. 
% 
% Output:
%       f is the fundamental matrix.    
%   
%----------------------------------------------------------
%
% From 
%    Book: "Multiple View Geometry in Computer Vision",
% Authors: Hartley and Zisserman, 2006, [HaZ2006]
% Section: "Computation of the Fundamental Matrix F",  specifically "The normalised 8-point algorithm"
% Chapter: 11
%    Page: 279
%
%----------------------------------------------------------
%      Author: Diego Cheda
% Affiliation: CVC - UAB
%        Date: 03/06/2008
%----------------------------------------------------------

%p1 = HomogeneousCoordinates(p1,'2D');
%p2 = HomogeneousCoordinates(p2,'2D');

% Normalization
% Transform the image coordinates according to x^_i = Tx_i and x'^_i =
% T'x'_i where T and T' are normalizing transformation consisting of a
% translation and scaling.
[p1,t1] = Normalise2DPts(p1);
[p2,t2] = Normalise2DPts(p2);


% (x,y)
x1 = p1(1,:)';
y1 = p1(2,:)';
%z1 = p1(3,:)';

x2 = p2(1,:)';
y2 = p2(2,:)';
%z2 = p2(3,:)';

% Number of points
numPts = size(p1,2);

% Af = 0
% Computes A, the constraint matrix of numpts x 9
a = [x2.*x1 x2.*y1 x2 y2.*x1 y2.*y1 y2 x1 y1 ones(numPts,1)];
%a = [x1.*x2 x1.*y2 x1 y1.*x2 y1.*y2 y1 x2 y2 ones(numPts,1)];

% Singular Value Decomposition of A.
% [U,S,V] = SVD(A) produces a diagonal matrix S, of the same dimension as A
% and with nonnegative diagonal elements in decreasing order, and unitary
% matrices U and V so that A = U*S*V'.
[u, d, v] = svd(a);

% Linear solution. Obtain F from 9th column of V (the smallest singular
% value of A).
%f = reshape(v(:,9),3,3)';
f = v(:,9);
f = [f(1) f(2) f(3); f(4) f(5) f(6); f(7) f(8) f(9)];

% Constraint enforcement. 
[u, d, v] = svd(f);
d(3,3) = 0;
f = u * d * v';

f = t2' * f * t1;