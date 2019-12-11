function [] = DrawEpipolarLines(f,pts,imageWidth,leftOrRight)
% Draws the epipolar lines computed by l' = F * x1 and l= F' * x2
%
% Input:
%       f is the fundamental matrix.
%
%       pts are the 2D points.
%
%       imageWidth is the width size of the image.
%
%       leftOrRight draw the epipolar lines to the left or the right image.
%
%----------------------------------------------------------
%
% From 
%    Book: "Multiple View Geometry in Computer Vision",
% Authors: Hartley and Zisserman, 2006, [HaZ2006] Section: "Fundamental
% Matrix Properties", Chapter: 9
%    Page: 246
%
%----------------------------------------------------------
%      Author: Diego Cheda
% Affiliation: CVC - UAB
%        Date: 03/06/2008
%----------------------------------------------------------

%Fundamental matrix must be computed using only x1 and x2 in homogeneous
%coordinates.
pts = HomogeneousCoordinates(pts,'2D');

for i=1:size(pts,2)
    % Epipolar line
    if (strcmp(leftOrRight,'right'))
        l = f * pts(:,i);
    else %left
        l = f' * pts(:,i);
    end
    
    %ax + by + c = 0 ==> y = (-ax - c) / b
    %x coordinate in the image
    x = [0 imageWidth];
    y = (-l(1)*x - l(3)) / l(2);
    
    % Convert from normalized to unnormalized coords
    %epLine = k * epLine;
    
    ci = mod(i,6)+1;
    
    colors = ['r','c','y','g','m','k'];    
    
	plot(x, y, colors(ci));
end

