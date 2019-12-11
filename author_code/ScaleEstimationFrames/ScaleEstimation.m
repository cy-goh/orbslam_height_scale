
%% This code is used to compute the absoulte scale based on 
close all;
clear all;
clc;
addpath 'CameraGeometry'
global param;

%% matching parameters
param.nms_n                  = 5;   % non-max-suppression: min. distance between maxima (in pixels)
param.nms_tau                = 50;  % non-max-suppression: interest point peakiness threshold
param.match_binsize          = 100;  % matching bin width/height (affects efficiency only)
param.match_radius           = 200; % matching radius (du/dv in pixels)
param.match_disp_tolerance   = 1;   % du tolerance for stereo matches (in pixels)
param.outlier_disp_tolerance = 3;   % outlier removal: disparity tolerance (in pixels)
param.outlier_flow_tolerance = 10;   % outlier removal: flow tolerance (in pixels)
param.multi_stage            = 1;   % 0=disabled,1=multistage matching (denser and faster)
param.half_resolution        = 0;   % 0=disabled,1=match at half resolution, refine at full resolution
param.refinement             = 2;   % refinement (0=none,1=pixel,2=subpixel)

%% camera intrinsic parameters  
param.f  = 707.09;
param.u0 = 601.89;
param.v0 = 183.11;
param.h  = 1.65;
% Intrinsic Matrix
param.K = [param.f 0 param.u0;
    0 param.f  param.v0;
    0       0      1  ];
param.KalmanWindow = 5;

%% define the kalman filter
kf = cv.KalmanFilter(4,3);
kf.statePre = [0;1;0;1.25]; % initial state prediction
kf.statePost = [0;1;0;1.25];
kf.transitionMatrix = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 -1 1];
kf.measurementMatrix  = [0 1 0 0 ; 0 0 1 0; 0 0 0 1];
kf.processNoiseCov = eye(4) * 1e-2;
kf.measurementNoiseCov = eye(3) * 1e-1;
kf.errorCovPost = eye(4) * 1e-2;

nprior = [0 1 0];                   % camera height

img_folder = './04/image_0/';
pose = load('./poses/04.txt');
numFrames = 10;
scaleg  = -ones(1,numFrames-1);
scales  = -ones(1,numFrames-1);
KalmanEstimation = zeros(param.KalmanWindow,4);
n0 = [0 1 0 ];
weight = zeros(1,param.KalmanWindow);
start_frame = 1;

for frame = start_frame:numFrames
    
if(mod(frame,100)==0)
frame
end
    
% read images
I1 = imread([img_folder  num2str(frame-1,'%06d') '.png']);
I2 = imread([img_folder  num2str(frame,'%06d') '.png']);
[R1,t1,Tr]   = Absolute_to_RelativePose(pose,frame,frame+1);
scaleg(1,frame) = norm(t1);
scaleg(1,frame) = norm(t1);
t1 = t1/norm(t1);
imgheight = size(I1,1);

 
%% features points detection and matching 
% init matcher
matcherMex('init',param);
% push back images
matcherMex('push',I1);
tic
matcherMex('push',I2);
% disp(['Feature detection: ' num2str(toc) ' seconds']);
% match images
tic; matcherMex('match',0); matcherMex('bucketing',50,50,5);
p_matched = matcherMex('get_matches',0);
% disp(['Feature matching:  ' num2str(toc) ' seconds']);
% close matcher
matcherMex('close');
% show matching results
% disp(['Number of matched points: ' num2str(length(p_matched))]);

%% display the matching points
I = [I1;I2];
figure;
imshow(I);
hold on;
[img_height,img_width] = size(I1);
p_matched1 = p_matched + repmat([0;0;0;img_height],1,size(p_matched,2));
plot(p_matched1(1,:) ,p_matched1(2,:),'+', 'LineWidth',2,'MarkerSize',2,'Color', [1,0,0]);
plot(p_matched1(3,:) ,p_matched1(4,:),'+', 'LineWidth',2,'MarkerSize',2,'Color', [1,0,0]);

p_matched_full    = p_matched;
srcPoints_full    = p_matched_full(1:2,:)';
disPoints_full    = p_matched_full(3:4,:)';

%% relative pose estimaiton
[R,t,inlierindex]      =       Rt_solver(srcPoints_full,disPoints_full); 
p_matched =  p_matched_full(:,inlierindex);
%% homography matrix estimation in a predifend Region of Interest
[H, inliersindex] = H_solver(p_matched(1:2,:), p_matched(3:4,:));
I = [I1;I2];
figure;
imshow(I);
hold on;
[img_height,img_width] = size(I1);
p_matched1 = p_matched(:,inliersindex) + repmat([0;0;0;img_height],1,size(inliersindex,2));
plot(p_matched1(1,:) ,p_matched1(2,:),'+', 'LineWidth',2,'MarkerSize',2,'Color', [1,0,0]);
plot(p_matched1(3,:) ,p_matched1(4,:),'+', 'LineWidth',2,'MarkerSize',2,'Color', [1,0,0]);

%% solver the initial value for d(distance from  the camera center to ground plane) and n(normal direction)  
[d0, n] = initial_solver(R,t,H);   

 %% set the initial value for simplex optimization solver
        PriorInf = [n' d0];
        if(frame-start_frame>1 && frame-start_frame < param.KalmanWindow)
            if(frame-start_frame>2)
              PriorInf  = mean(KalmanEstimation(1:frame-start_frame-1,:),1);
            else
              PriorInf  = KalmanEstimation(1,:);
            end
        end
         w = sum(weight);
        if( frame-start_frame > param.KalmanWindow)
            w = 0;
              PriorInf = zeros(1,4);
            for i = 1:5
              if( KalmanEstimation(i,4)>0)
                  PriorInf = PriorInf + KalmanEstimation(i,:)*weight(i);
                  w = w + weight(i);
              end
            end
            if(w>0)
              PriorInf = PriorInf/w;
            else
              PriorInf = [n' 1.0/norm(x(2:4,:))];  
            end
        end
        PriorNormal = PriorInf(1,1:3);
        intersectionAngle  = rad2deg(acos(dot(PriorNormal,nprior)/norm(nprior)/norm(PriorNormal)));
        intersectionAngle1 = rad2deg(acos(dot(nprior,n)/norm(n)/norm(nprior)));
        if(frame-start_frame>param.KalmanWindow && intersectionAngle < 3 && abs(d0-PriorInf(1,4))<0.3)
            x0s = [PriorNormal(1,2) PriorNormal(1,3) PriorInf(1,4)];
        elseif (intersectionAngle1<3)
            x0s = [n(2,1) n(3,1) d0];
        else
            Normal_estimated = n;
            scale = param.h / kf.statePost(4, 1);
            KalmanEstimation(mod(frame-start_frame - 1, param.KalmanWindow)+1, 1) = Normal_estimated(1, 1);
            KalmanEstimation(mod(frame-start_frame - 1, param.KalmanWindow)+1, 2) = Normal_estimated(2, 1);
            KalmanEstimation(mod(frame-start_frame - 1, param.KalmanWindow)+1, 3) = Normal_estimated(3, 1);
            KalmanEstimation(mod(frame-start_frame - 1, param.KalmanWindow)+1, 4) = d0;

           %% update the weight parameters
            if (frame-start_frame >= param.KalmanWindow)
                weight(mod(frame-start_frame - 1, param.KalmanWindow)+1) = 1.0;
                weight(mod(frame-start_frame - 2, param.KalmanWindow)+1) = 0.8;
                weight(mod(frame-start_frame - 3, param.KalmanWindow)+1) = 0.6;
                weight(mod(frame-start_frame - 4, param.KalmanWindow)+1) = 0.4;
                weight(mod(frame-start_frame - 5, param.KalmanWindow)+1) = 0.2;
            end  
          continue;
        end
        %% optimization process
        [xsi,f] = Nelder_Mead_simplex_Sparse(p_matched(1:2,inliersindex),p_matched(3:4,inliersindex),R,t,param.K,x0s);
        
        scale = param.h/xsi(1,3);
        normN = norm(xsi(1,1:2));
        if ((normN - 1)<1e-9)
            optimalnormal(1, 1) = sqrt(1 - normN);
            optimalnormal(2, 1) = xsi(1, 1);
            optimalnormal(3, 1) = xsi(1, 2);
        else
            optimalnormal(1, 1) = 0;
            optimalnormal(2, 1) = xsi(1, 1)/normN;
            optimalnormal(3, 1) = xsi(1, 2)/normN;
        end
         d_estimated =xsi(1,3);
        %% if the loss is bigger we will not trust the estimation in this frame
       if(f>300)
            d_estimated = 0;
            d_kalman = kf.statePost(4, 1);
            Normal_estimated = n;
            scale = param.h / kf.statePost(4, 1);
            KalmanEstimation(mod(frame-start_frame - 1, param.KalmanWindow)+1,1) = Normal_estimated(1, 1);
            KalmanEstimation(mod(frame-start_frame - 1, param.KalmanWindow)+1,2) = Normal_estimated(2, 1);
            KalmanEstimation(mod(frame-start_frame - 1, param.KalmanWindow)+1,3) = Normal_estimated(3, 1);
            KalmanEstimation(mod(frame-start_frame - 1, param.KalmanWindow)+1,4) = d0;  
        %% update the weight parameters
        if (frame-start_frame >= param.KalmanWindow)
            weight(mod(frame-start_frame - 1, param.KalmanWindow)+1) = 1.0;
            weight(mod(frame-start_frame - 2, param.KalmanWindow)+1) = 0.8;
            weight(mod(frame-start_frame - 3, param.KalmanWindow)+1) = 0.6;
            weight(mod(frame-start_frame - 4, param.KalmanWindow)+1) = 0.4;
            weight(mod(frame-start_frame - 5, param.KalmanWindow)+1) = 0.2;
        end
          continue;
       end
        dToplane = d_estimated;
        predictState = kf.predict();
        Measurement(1, 1) = optimalnormal(2, 1);
        Measurement(2, 1) = optimalnormal(3, 1);
        Measurement(3, 1) = dToplane; 
        Estimated = kf.correct(Measurement);
        refinedToplane  =  Estimated(4, 1);
	    d_kalman = refinedToplane;
        scale            = param.h / refinedToplane;
        normN = norm(Estimated(1:3,1));
        if ((normN - 1)<1e-9)
            Normal_estimated(1, 1) = sqrt(1 - normN);
            Normal_estimated(2, 1) = Estimated(2, 1);
            Normal_estimated(3, 1) = Estimated(3, 1);
        else
            Normal_estimated(1, 1) = 0;
            Normal_estimated(2, 1) = Estimated(2, 1);
            Normal_estimated(3, 1) = Estimated(3, 1)/normN;
        end
            KalmanEstimation(mod(frame-start_frame-1,param.KalmanWindow)+1, 1) = Normal_estimated(1, 1);
            KalmanEstimation(mod(frame-start_frame-1,param.KalmanWindow)+1, 2) = Normal_estimated(2, 1);
            KalmanEstimation(mod(frame-start_frame-1,param.KalmanWindow)+1, 3) = Normal_estimated(3, 1);
            KalmanEstimation(mod(frame-start_frame-1,param.KalmanWindow)+1, 4) = refinedToplane;  

    %% update the weight parameters
        if (frame-start_frame >= param.KalmanWindow)
            weight(mod(frame-start_frame - 1, param.KalmanWindow)+1) = 1.0;
            weight(mod(frame-start_frame - 2, param.KalmanWindow)+1) = 0.8;
            weight(mod(frame-start_frame - 3, param.KalmanWindow)+1) = 0.6;
            weight(mod(frame-start_frame - 4, param.KalmanWindow)+1) = 0.4;
            weight(mod(frame-start_frame - 5, param.KalmanWindow)+1) = 0.2;
        end
   %% Normal_estimated = _scalepara.KF.statePost;
        kf.transitionMatrix = eye(4,4);
        kf.transitionMatrix(1:3,1:3) = R';	
        kf.transitionMatrix(4, 1:3)  = t';
        
     scales(1,frame) = scale;   

end

figure;
plot(scales,'r.');
hold on;
plot(scaleg,'g*');
axis([0 size(scaleg,2) 0 3]);
grid on;
xlabel('FrameNumber');
ylabel('Scale Estimation');
legend('Estimation Scale','Ground Truth');


