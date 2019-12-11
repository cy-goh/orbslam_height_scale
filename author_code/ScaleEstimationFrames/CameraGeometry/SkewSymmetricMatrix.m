function s = SkewSymmetricMatrix(matrixA)
% Input:
%       - matrixA is a 3x1 or a 1x3 vector.
% 
% Output:
%       - S is a 3x3 matrix with the form:
%         _        _
%        |0 -a3  a2 |
%        |a3  0 -a1 |
%        |-a2 a1  0 |
%        -          -
%
%----------------------------------------------------------
%      Author: Diego Cheda
% Affiliation: CVC - UAB
%        Date: 03/06/2008
%----------------------------------------------------------

if (size(matrixA,2)==3)
    matrixA = matrixA'
end

s = [ 0           -matrixA(3)   matrixA(2); ...
      matrixA(3)   0           -matrixA(1); ... 
	 -matrixA(2)   matrixA(1)   0 ];
