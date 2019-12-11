function [ep1,ep2] = Epipoles(f)
% Computes the epipoles as null vectors of the fundamental matrix.
%
% Input:
%		- f is the fundamental matrix.
%
% Output:
%		- ep1 and ep2 are the epipoles.
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


% en algunas ocasiones puede ser necesario usar 'r' como parámetro de null
% (no tengo claro cuando).
%e1 = null(f,'r');
%e2 = null(f','r');

% epipole computation
[dummy, dummy, v] = svd(f);
ep1 = v(:,3)/v(3,3);

[dummy, dummy, v] = svd(f');
ep2 = v(:,3)/v(3,3);

