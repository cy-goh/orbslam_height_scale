function [el1,el2] = EpipolarLines(f,x1,x2)
% Computes the epipolar lines
%
% Input:
%       - f is the fundamental matrix.
%
%       - x1 and x2 are 3xn the corresponding 2D points.
%
% Output:
%       - el1 and el2 are the epipolar lines.
%
%----------------------------------------------------------
%
% From 
%    Book: "Multiple View Geometry in Computer Vision",
% Authors: Hartley and Zisserman, 2006, [HaZ2006]
% Section: "Properties of the fundamental matrix", 
% Chapter: 9
%    Page: 245
%
%----------------------------------------------------------
%      Author: Diego Cheda
% Affiliation: CVC - UAB
%        Date: 03/06/2008
%----------------------------------------------------------

el1 = f' * x2;

el2 = f * x1;
