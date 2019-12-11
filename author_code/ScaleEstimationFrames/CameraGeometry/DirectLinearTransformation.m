function h = DirectLinearTransformation(p1,p2)
% Computes Direct Linear Transformation (DTL) algorithm
%
% Input:
%       p1 and p2 are 2xN (or 3xN in homogeneous coordiantes) of
%       correspondings corners. 
%
% Output:
%       h is an estimation of the projective transformation such that 
%       h * x_i= x'_i
%
%----------------------------------------------------------
%
% From 
%    Book: "Multiple View Geometry in Computer Vision",
% Authors: Hartley and Zisserman, 2006, [HaZ2006]
% Section: "The Direct Linear Transformation (DTL) algorithm"
% Chapter: 4
%    Page: 89 and 109
%
%----------------------------------------------------------
%      Author: Diego Cheda
% Affiliation: CVC - UAB
%        Date: 03/06/2008
%----------------------------------------------------------

% Convert to homogeneous coordinates.
if (size(p1,1) ~= 3)
    p1 = padarray(p1,[1 0],1,'post');
    p2 = padarray(p2,[1 0],1,'post');
end

% Normalization
% Transform the image coordinates according to x^_i = Tx_i and x'^_i =
% T'x'_i where T and T' are normalizing transformation conssiting of a
% translation and scaling.
[p1,t1] = Normalise2DPts(p1);
[p2,t2] = Normalise2DPts(p2);

x2 = p2(1,:);
y2 = p2(2,:);
z2 = p2(3,:);

% Ah = 0
a = [];
for i=1:size(p1,2)
    a = [a; zeros(3,1)'     -z2(i)*p1(:,i)'   y2(i)*p1(:,i)'; ...
            z2(i)*p1(:,i)'   zeros(3,1)'     -x2(i)*p1(:,i)'];
           %-y2*p1     x2*p1      zeros(1,3)
end

% Obtain the SVD of A. The unit singular vector corresponding to the
% smallest singular value is the solucion h. A = UDV' with D diagonal with
% positive entries, arranged in descending order down the diagonal, then h
% is the last column of V.
[u,d,v] = svd(a);

h = reshape(v(:,9),3,3)';

% Desnormalization
h = inv(t2) * h * t1;
