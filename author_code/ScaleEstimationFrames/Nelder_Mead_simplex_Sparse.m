function [x,fval] =  Nelder_Mead_simplex_Sparse(srcPoints, disPoints,R,t,K,x0)

options = optimset('Display','final');

[x,fval] = fminsearch(@(x) myfun(srcPoints, disPoints,R,t,K, x), x0);
n2 = x(1,1);
n3 = x(1,2);
if(n2*n2+n3*n3<1)
    n1 = sqrt(1-n2*n2-n3*n3);
else
    n2 = n2/sqrt(n2*n2+n3*n3);
    n3 = n3/sqrt(n2*n2+n3*n3);
    n1= 0;
end
 x(1,1) = n2;
 x(1,2) = n3;
 
 x = x(1,1:3);



function F = myfun(srcPoints, disPoints,R,t,K, x)
h  = x(1,3);
n2 = x(1,1);
n3 = x(1,2);
lambda = 1;
% lambda = 1.0;
if(n2*n2+n3*n3<1)
    n1 = sqrt(1-n2*n2-n3*n3);
else
    n2 = n2/sqrt(n2*n2+n3*n3);
    n3 = n3/sqrt(n2*n2+n3*n3);
    n1= 0;
end

n = [n1;n2;n3;];
H = lambda*K*(R + t*n'/h)*inv(K);
invH = inv(H+0.001*eye(3,3));
% forward transform
H = H*1.0/H(3,3);
homog_srcPoints = [srcPoints;ones(1,size(srcPoints,2))];
homog_projected = H*homog_srcPoints;
projected      = homog_projected(1:2,:)./[homog_projected(3,:);homog_projected(3,:)];

% backwark transform 

invH = invH*1.0/invH(3,3);

homog_disPoints = [disPoints;ones(1,size(disPoints,2))];
homog_projecteddis = invH*homog_disPoints;
projecteddis      = homog_projecteddis(1:2,:)./[homog_projecteddis(3,:);homog_projecteddis(3,:)];

errorforward  = (projected(1:2,:) - disPoints).*(projected(1:2,:) - disPoints);
errorbackward = (projecteddis(1:2,:) - srcPoints).*(projecteddis(1:2,:) - srcPoints);
%   error = sqrt(errorforward(1,:) + errorforward(2,:)    + errorbackward(1,:) + errorbackward(2,:));
 error = sqrt(errorforward(1,:) + errorforward(2,:)); 
medianerror = median(error);
index = find(error>5*medianerror);
error(:,index) = 5*medianerror;
 F = sum(error);


