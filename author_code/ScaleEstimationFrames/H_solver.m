%% this function is used to estimate the homography matrix in a predifined ROI
% inputs: srcPoints: 
%         disPoints: matched points between two frames
% Outputs: H and inliers index of the features
function [H, inliersindex] = H_solver(srcPoints, disPoints)

%% predifine the region of interes
%  the ROI is defined as a Trapezoid
x1 = 500; % left top points x coordinate
y1 = 230; % left top points y coordinate
w1 = 200; % the top base length of the Trapezoid
w2 = 400; % the bottom base length of the Trapezoid
h = 150;  % the height of the Trapezoid
%% compute the four coners of the Trapezoid
x2 = x1+w1;
y2 = y1;
x3 = x1-0.5*(w2-w1);
y3 = y1+h ;
x4 = x2+0.5*(w2-w1);
y4 = y3;

A1 = y3 -y1; B1 = x1 - x3; C1 = y1*x3-x1*y3;
A2 = y4 -y2; B2 = x2 - x4; C2 = y2*x4-x2*y4;
p_matched = [srcPoints;disPoints];
%% find the featues inside the ROI
indexROI = zeros(size(p_matched,2),1);
for k = 1:size(p_matched,2)
i = p_matched(1,k);
j = p_matched(2,k);
if(A1*i+B1*j+C1>0 && A2*i+B2*j+C2<0 && j > y1)
    indexROI(k,1) = 1;
end
end
index             = find(indexROI>0);
p_matched         = p_matched(:,index);
srcPoints         =  p_matched(1:2,:);
disPoints         =  p_matched(3:4,:);

%% Calculate homography matrix by using the feature points in ROI
% Compute the homography matrix
H = cv.findHomography(srcPoints', disPoints', 'Method', 'Ransac', 'RansacReprojThreshold',1.5);
%% find the inliers
% forward transform
homog_srcPoints = [srcPoints;ones(1,size(srcPoints,2))];
homog_projected = H*homog_srcPoints;
projected      = homog_projected(1:2,:)./[homog_projected(3,:);homog_projected(3,:)];
invH = inv(H);
invH = invH*1.0/invH(3,3);
% backwark transform
invH = inv(H);
invH = invH*1.0/invH(3,3);
homog_disPoints = [disPoints;ones(1,size(disPoints,2))];
homog_projecteddis = invH*homog_disPoints;
projecteddis      = homog_projecteddis(1:2,:)./[homog_projecteddis(3,:);homog_projecteddis(3,:)];
errorforward  = (projected(1:2,:) - disPoints).*(projected(1:2,:) - disPoints);
errorbackward = (projecteddis(1:2,:) - srcPoints).*(projecteddis(1:2,:) - srcPoints);
error = sqrt(errorforward(1,:) + errorforward(2,:) + errorbackward(1,:) + errorbackward(2,:));
inliers = find(error<2);
inliersindex = index(inliers',:)';

