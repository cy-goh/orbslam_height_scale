function e = EssentialMatrixFromCameraParams(r,c)
% Computes the Essential Matrix by
% E = [t]_x R 
%    where t = -RC and [t]_x is the matrix representation of the cross
%    product.
%  
% Input:
%       - r is a 3x3 rotation matrix that represents the orientation of the
%         camera coordinate frame.
%
%       - c is 1x3 or a 3x1 a coordinates of the camera in the world
%         coordiante frame.
%
% Output:
%	- e is the essential matrix.
%
%----------------------------------------------------------
%
% From 
%    Book: "Multiple View Geometry in Computer Vision",
% Authors: Hartley and Zisserman, 2006, [HaZ2006]
% Section: "The essential matrix", 
% Chapter: 9
%    Page: 257
%
%----------------------------------------------------------
%      Author: Diego Cheda
% Affiliation: CVC - UAB
%        Date: 03/06/2008
%----------------------------------------------------------

t = -r * c;
s = [ 0    -t(3)  t(2);  ...
      t(3)  0    -t(1);  ...
	 -t(2)  t(1)  0 ];
	 
e = s * r;
