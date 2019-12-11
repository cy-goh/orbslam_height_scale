function x2D = Projection3DTo2D(x3D,k,r,c,filter)
% Computes the x 2D points applying the following projection: 
% x = KR[I|-C]X
% where x = (x,y) is the 2D-point in 2D-image plane
%       K is a camera calibration matrix
%       R is a 3x3 rotation matrix that represents the orientation of the
%       camera coordinate frame.
%       I is the identity matrix
%       C is a coordinates of the camera in the world coordiante frame. 
%       X is the 3D-point in 3D-world cordinate frame 
%
% Input:
%       - x3D is a 4x1 matrix with 3D world points.
%	  
%       - k is a 3x3 calibration matrix.
%               _           _
%              | f       px |
%          K = |     f   py |
%              |_         1 _|
%
%         where 
%              - f is the focal length,
%
%              - (px,py) are the principal point where the principal axis
%                meets the image plane.
%       
%       - r is a 3x3 rotation matrix that represents the orientation of the
%         camera coordinate frame.
%
%       - c is 1x3 or a 3x1 a coordinates of the camera in the world
%         coordiante frame.
%
%       - filter is a flag to indicate if the points that are in back of
%       the camera are shown. filter = 0 by default.
%
%  Another usage is Projection3DTo2D(x3D, p)
%	where p is the 3x4 camera matrix.
%
% Output:
%        x2D = [x ; y] a matrix with 2D image point coordinates.
%       
%        p is the camera matrix.
%
%----------------------------------------------------------
%
% From 
%    Book: "Multiple View Geometry in Computer Vision",
% Authors: Hartley and Zisserman, 2006, [HaZ2006]
% Section: "Camera Models", 
% Chapter: 6
%    Page: 153
%
%----------------------------------------------------------
%      Author: Diego Cheda
% Affiliation: CVC - UAB
%        Date: 03/06/2008
%----------------------------------------------------------

x3D = HomogeneousCoordinates(x3D,'3D');

if (nargin == 5 | nargin == 4)
	if (size(c,2) == 3) %is a 1x3 matrix
	    c = c';
	end

	%x = KR[I|-C]x3D
	p =  k * r * [eye(3) -c];
else
    if (nargin == 2)
        p = k;
    end
end

if (nargin < 5)
    filter = 0;
end 

x2D = p * x3D;


% Filter:  z < 0 in x2D
% In order to not show the points in back of the camara.s
if (filter)
    index = find(x2D(3,:) > 0);
    x2D = [x2D(1,index); ...
           x2D(2,index); ...
           x2D(3,index); ];
end
