function [] = Draw3DCamera(p,cameraName,color)
% Draws the camera position and axis rotation in the 3D scene.
% 
% Input:
%	- p is a matrix at the form [r c] where
%            c is the coordinates of the camera, and
%	         r is the rotation matrix.
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

if (nargin < 3)
    color = 'r';
end

c = p(:,4);

% Plot a circle on the 3D coordinates of the camera
plot3(c(1),c(3),c(2),'o','MarkerSize',10,'Color',color,'Linewidth',1);
axis equal;


% coordinates of the vectors in the init frame
i0 = [1 0 0]';
j0 = [0 1 0]';
k0 = [0 0 1]';

vx = p(1:3,1:3)*i0;
vy = p(1:3,1:3)*j0;
vz = p(1:3,1:3)*k0;

% Plot the rotation (?)
x = quiver3(c(1),c(3),c(2),vx(1),vx(3),vx(2),50);
set(x(:),'LineWidth',1); 
set(x(:),'Color','black'); 

y = quiver3(c(1),c(3),c(2),vy(1),vy(3),vy(2),50);
set(y(:),'LineWidth',1); 
set(y(:),'Color','black'); 

z = quiver3(c(1),c(3),c(2),vz(1),vz(3),vz(2),50);
set(z(:),'LineWidth',1); 
set(z(:),'Color',color); 


% Display a text with the camera name.
if (nargin >= 3)
	text(c(1),c(3),c(2),strcat('  \leftarrow ',cameraName),...
     'HorizontalAlignment','left','Color',color)
end

