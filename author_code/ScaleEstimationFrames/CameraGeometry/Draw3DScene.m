function [] = Draw3DScene(x3D,line,drawTitle)
% Draws the 3D scene.
%
% Input:
%	- x3D are the 3D points.
%
%   - line is the kind of line. By default, the only points are
%     diplayed.
%
%	- drawTitle is the title name of the draw. It is optional.
%
%----------------------------------------------------------
%      Author: Diego Cheda
% Affiliation: CVC - UAB
%        Date: 03/06/2008
%----------------------------------------------------------

if (nargin < 2)
    line = '.';
end


% Convert to homogeneous coordinates
x3D = HomogeneousCoordinates(x3D,'3D');


% 3D view
% OJO: hay que tener cuidado con el uso de los ejes. Matlab utiliza diferente los ejes en la graficación.
% Matlab -> Ejes nuestros
% X -> X
% Y -> Z
% Z -> Y
plot3(x3D(1,:),x3D(3,:),x3D(2,:),line,'MarkerSize',10,'Color','k', 'LineWidth', 1);

% image points in world coordinates (use hom coordinates)
% [r,c] = find(x1>0);
% for i=1:length(c)
%     %plot3([X(1,i) camera3d1(1)],[X(2,i) camera3d1(2)],[X(3,i) camera3d1(3)], 'Color','r', 'LineWidth', 1);
%     plot3([X(1,c(i)) camera3d1(1)],[X(2,c(i)) camera3d1(2)],[X(3,c(i)) camera3d1(3)], 'Color','r', 'LineWidth', 1);
% end

% Axis
%axis equal;

% Axis names
xlabel('x', 'FontSize', 12); zlabel('y', 'FontSize', 12); ylabel('z', 'FontSize', 12);

% Graph title
if (nargin == 3)
	title(drawTitle, 'FontSize', 12);
end

% Grid
grid on;