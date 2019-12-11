function [] = Draw3DCamera2(c,xAngle,yAngle,zAngle,cameraName,color)
% Draws the camera position and axis rotation in the 3D scene.
% 
% Input:
%	- c is the coordinates of the camera.
%
%	- xAngle,yAngle and zAngle are the angles of the rotation (in degree).
%
%	- cameraName is a text displayed at left of the camera. It is optional.
%
%   - color is the color of z-axis (principal axis).
%
%----------------------------------------------------------
%      Author: Diego Cheda
% Affiliation: CVC - UAB
%        Date: 03/06/2008
%----------------------------------------------------------

if (nargin < 6)
    color = 'r';
end

r = RotationMatrix(-xAngle,-yAngle,-zAngle); 
%p =  r * [eye(3) -c]; <-- incorrect

% Plot a circle on the 3D coordinates of the camera
plot3(c(1),c(3),c(2),'o','MarkerSize',10,'Color',color,'Linewidth',1);
axis equal;

% Plot the rotation (?)
x = quiver3(c(1),c(3),c(2),r(1,1),r(3,1),r(2,1),50);
set(x(:),'LineWidth',1); 
set(x(:),'Color','black'); 

y = quiver3(c(1),c(3),c(2),r(1,2),r(3,2),r(2,2),50);
set(y(:),'LineWidth',1); 
set(y(:),'Color','black'); 

z = quiver3(c(1),c(3),c(2),r(1,3),r(3,3),r(2,3),50);
set(z(:),'LineWidth',1); 
set(z(:),'Color',color); 


% Display a text with the camera name.
% if (nargin >= 3)
% 	text(c(1),c(3),c(2),strcat('  \leftarrow ',cameraName),...
%      'HorizontalAlignment','left','Color',color)
% end

