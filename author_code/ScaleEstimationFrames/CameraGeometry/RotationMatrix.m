function r = RotationMatrix(xAngle, yAngle, zAngle, unitAngle)
% Computes the rotation matrix
% 
% Input:
%       - xAngle is the rotation over the x axis in radians/degrees.
%
%       - yAngle is the rotation over the y axis in radians/degrees.
%
%       - zAngle is the rotation over the x axis in radians/degrees.
%
%       - unitAngle to indicate if the input angles are in radians and degrees.
%         unitAngle = 0 => radians and unitAngle = 1 => degrees. By default, degrees.
%
% Output:
%       r is a 3x3 matrix with the rotation.
%
%----------------------------------------------------------
%
% From 
%    Book: "Machine Vision",
% Authors: Jain, Kasturi and Schunck, 2006, [JKS1995]
% Section: "Rotation Matrices", 
% Chapter: 12
%    Page: 316
%
%----------------------------------------------------------
%      Author: Diego Cheda
% Affiliation: CVC - UAB
%        Date: 03/06/2008
%----------------------------------------------------------

% angle_radians = angle_degrees * pi / 180
% angle_degrees = angle_radians * 180 / pi
if (nargin == 3 || (nargin > 3 && unitAngle == 1))
    xAngle = xAngle * pi / 180;
    yAngle = yAngle * pi / 180;
    zAngle = zAngle * pi / 180;
end

cosX = cos(xAngle);
sinX = sin(xAngle);

cosY = cos(yAngle);
sinY = sin(yAngle);

cosZ = cos(zAngle);
sinZ = sin(zAngle);

rxx = cosY * cosZ;
rxy = sinX * sinY * cosZ + cosX * sinZ;
rxz = -cosX * sinY * cosZ + sinX * sinZ;

ryx = -cosY * sinZ;
ryy = -sinX * sinY * sinZ + cosX * cosZ;
ryz = cosX * sinY * sinZ + sinX * cosZ;

rzx = sinY;
rzy = -sinX * cosY;
rzz = cosX * cosY;

r = [ rxx rxy rxz; ...
      ryx ryy ryz; ... 
	  rzx rzy rzz ];
  
% Another way to compute the same
% rx = [1 0 0; 0 cosX sinX; 0 -sinX cosX];
% ry = [cosY 0 -sinY; 0 1 0; sinY 0 cosY];
% rz = [cosZ sinZ 0; -sinZ cosZ 0; 0 0 1];
% 
% r = rz * ry * rx;
