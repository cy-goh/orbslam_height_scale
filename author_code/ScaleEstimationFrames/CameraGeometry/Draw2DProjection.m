function [] = Draw2DProjection(x,drawTitle,color,line,xAxisMin,xAxisMax,yAxisMin,yAxisMax)
% Draws the 2D points from the projection computed.
% 
% Input:
%		- x are the 2D points.
%
%		- drawTitle is the title name of the draw. By default, the title is
%		not displayed.
%
%       - color is the color of the draw. By default, the color is red.
%
%       - line is the kind of line. By default, the only points are
%       diplayed.
%
%		- xAxisMin, xAxisMax, yAxisMin, yAxisMax are used to set scaling
%		for the x- and y-axes on the current plot. By default, xAxisMin=0,
%		xAxisMax=640, yAxisMin=0, yAxisMax=480.
%
%----------------------------------------------------------
%      Author: Diego Cheda
% Affiliation: CVC - UAB
%        Date: 03/06/2008
%----------------------------------------------------------

if (nargin < 5) 
    xAxisMin = 0;
	xAxisMax = 640;
	yAxisMin = 0;
    yAxisMax = 480;    
end

if (nargin < 4)
    line = '.';
else if (nargin < 3)
        color = 'r';
        line = '.';
    end
end

% Filter: z < 0 in x (2D)
% In order to not show the points in back of the camara.
index = find(x(3,:) > 0);
x = [x(1,index); ...
     x(2,index); ...
     x(3,index)];

% Convert to homogeneous coordinates
x = HomogeneousCoordinates(x,'2D');

% Plot 2D points
% OJO: hay que tener cuidado con los ejes. plot(X,Y) donde X es el eje X e
% Y es el eje Y.
plot(x(1,:),x(2,:),line,'MarkerSize',10, 'LineWidth', 1,'Color',color); 

% Axes names
ylabel('y', 'FontSize', 12); xlabel('x', 'FontSize', 12);

% Draw title
if (nargin >= 2)
	title(drawTitle, 'FontSize', 12);
end

% Scene clipping -> image size content
axis([xAxisMin xAxisMax yAxisMin yAxisMax]);
axis xy;

% Grid
grid on;

shg;
