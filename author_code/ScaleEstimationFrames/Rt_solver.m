%% This function is estimate the camera pose up to scale
% inputs: srcPoints: 
%         disPoints: matched points between two frames
% Outputs: R,t and inliers index of the features

function      [R,t,inlierindex]      =       Rt_solver(srcPoints,disPoints)
        global param;
        [fRANSAC, mask] = cv.findFundamentalMat(srcPoints, disPoints, 'Method','Ransac','Param1',0.5,'Param2',0.999);
       
        inliersIndex = find(mask>0);
        E = param.K'*fRANSAC*param.K;
        [rot,t] = EssentialMatrixToCameraMatrix(E);
        srcPoints_full_inliers = srcPoints(inliersIndex,:);
        disPoints_full_inliers = disPoints(inliersIndex,:);
        [R,t,correct,x3D] = SelectCorrectEssentialCameraMatrix(rot,t,HomogeneousCoordinates(srcPoints_full_inliers','2Dforce'),HomogeneousCoordinates(disPoints_full_inliers','2Dforce'),param.K);
        t = t/norm(t);
       %% refine matched points  by using epipolar constraint
        homog_srcPoints = [srcPoints';ones(1,size(srcPoints,1))];
        homog_disPoints = [disPoints';ones(1,size(disPoints,1))];
        F_dis= zeros(size(srcPoints,1),1);

        for i = 1:size(srcPoints,1)
        abc = fRANSAC*homog_srcPoints(:,i);
        den = sqrt(abc(1,1)*abc(1,1)+ abc(2,1)*abc(2,1));
        dis = homog_disPoints(:,i)'*abc;
        F_dis(i,1) = sqrt(dis*dis/den);
        end
        inlierindex = find(F_dis<0.5);
   